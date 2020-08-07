#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/time_synchronizer.h>
#include <ros/ros.h>

#include <opencv2/core/core.hpp>

#include <rosbag/bag.h>
#include <rosbag/view.h>

#include <std_msgs/Float64.h>
#include <std_msgs/Int8.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>

#include <opencv2/core/core.hpp>

#include <deque>
#include <mutex>
#include <string>
#include <vector>

#include "../../../include/System.h"

int test_bagview(int argc, char** argv) {
  std::string bagfile = "/home/pkok/test.bag";
  rosbag::Bag bag;
  bag.open(bagfile);

  std::vector<std::string> topics;
  topics.push_back("/test1");
  // topics.push_back("/test2");
  topics.push_back("/test3");
  rosbag::View view(bag, rosbag::TopicQuery(topics));

  unsigned int i = 0;
  for (rosbag::MessageInstance const& m : view) {
    std_msgs::Int8Ptr int1 = m.instantiate<std_msgs::Int8>();
    std::string t1 = m.getTopic();

    // std_msgs::Int8Ptr int2 = m.instantiate<std_msgs::Int8>();
    // std::string t2 = m.getTopic();

    std_msgs::Float64Ptr flt = m.instantiate<std_msgs::Float64>();
    std::string t3 = m.getTopic();
    std::cout << (float)flt->data << std::endl;
    break;

    std::cout << "topics:  " << t1
              << ", "
              //<< t2 << ", "
              << t3 << std::endl;

    std::cout << "i:       " << i << "\n"
              << "int 1:   " << (int)int1->data
              << "\n"
              //<< "int 2:   " << (int) int2->data << "\n"
              << "float:   " << (float)flt->data
              << "\n"
              //<< "1 == 2?  " << (int1 == int2)
              << std::endl;
    ++i;
    if (i > 15) break;
  }
  return EXIT_SUCCESS;
}

class SynchroBuffer {
 public:
  enum class Status { WAITING, INITIALIZING, NORMAL };

  using CallbackType = cv::Mat(const cv::Mat&, const cv::Mat&,
                               const std::vector<ORB_SLAM2::IMUData>&,
                               const double&);

  SynchroBuffer(ros::NodeHandle& nh, const ORB_SLAM2::ConfigParam& config,
                std::function<CallbackType> data_processor)
      : status(Status::WAITING),
        image1_subscriber(nh, config._imageTopicLeft, 4),
        image2_subscriber(nh, config._imageTopicLeft, 4),
        imu_subscriber(nh, config._imuTopic, 200),
        sync(sync_pol(10), image1_subscriber, image2_subscriber,
             imu_subscriber),
        data_processor(data_processor),
        gravity_multiplier(config.GetAccMultiply9p8() ? config.GetG() : 1.0),
        map_1x(config._map_1x),
        map_1y(config._map_1y),
        map_2x(config._map_2x),
        map_2y(config._map_2y) {
    auto _1 = std::placeholders::_1;
    auto _2 = std::placeholders::_2;
    auto _3 = std::placeholders::_3;
    imu_subscriber.registerCallback(
        std::bind(&SynchroBuffer::store_imu, this, _1));
    sync.registerCallback(
        std::bind(&SynchroBuffer::store_synchronized_data, this, _1, _2, _3));
    std::cout << "  Setup of SynchroBuffer was succesful" << std::endl;
  }

  bool is_cleared;

  // Callback for storing IMU messages in a buffer.
  // These are not collected in synchronization with the image messages.
  // After storing, the system attempts to process_data().
  void store_imu(const sensor_msgs::ImuConstPtr& msg) {
    {
      std::unique_lock<std::mutex> lock(buffer_mutex);
      std::cout << "  Entering store_imu..." << std::endl;
      if (is_cleared) std::cout << "    imu_buffer was cleared!" << std::endl;
      switch (status) {
        case Status::WAITING:
          imu_time_start = msg->header.stamp;
          status = Status::INITIALIZING;
          // fall through
        case Status::INITIALIZING:
          // fall through
        case Status::NORMAL:
          imu_buffer.emplace_back(imu_msg2data(msg));
          break;
      }
      is_cleared = false;
    }
    std::cout << "  Exiting store_imu, continuing with process_data"
              << std::endl;
    process_data();
  }

  // Callback for storing synchronized messages.
  // After storing, the system attempts to process_data().
  void store_synchronized_data(const sensor_msgs::ImageConstPtr& msg1,
                               const sensor_msgs::ImageConstPtr& msg2,
                               const sensor_msgs::ImuConstPtr& msg3) {
    {
      std::cout << "  Entering store_synchronized_data..." << std::endl;
      std::unique_lock<std::mutex> lock(synchronize_mutex);
      switch (status) {
        case Status::WAITING:
          return;
        case Status::INITIALIZING:
          status = Status::NORMAL;
          // fall through
        case Status::NORMAL:
          image1 = msg1;
          image2 = msg2;
          synced_imu_stamp = msg3->header.stamp.toSec();
          break;
      }
    }
    std::cout
        << "  Exiting store_synchronized_data, continuing with process_data"
        << std::endl;
    process_data();
  }

  // Process stored data, up until the timestamp of the synchronized data.
  // Also cleans up the used data in the process.
  bool process_data() {
    // check if the buffered IMU msgs contain the IMU msg
    // that came in through synchronization.  We can't just add
    // the one through synchronization, as we might still have to store
    // some non-synchronized IMU messages.
    std::cout << "  Entering process_data" << std::endl;
    std::vector<ORB_SLAM2::IMUData> copied_buffer;
    cv::Mat copied_image1;
    cv::Mat copied_image2;
    double timestamp;
    {
      std::unique_lock<std::mutex> lock1(buffer_mutex);
      std::unique_lock<std::mutex> lock2(synchronize_mutex);
      auto pos = std::find_if(std::begin(imu_buffer), std::end(imu_buffer),
                              std::bind(&SynchroBuffer::imu_msg_equal, this,
                                        std::placeholders::_1));
      if (pos == std::end(imu_buffer)) {
        std::cout << "    Done; nothing to do" << std::endl;
        return false;
      }
      try {
        cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(image1);
        cv::remap(cv_ptr->image, copied_image1, map_1x, map_1y,
                  cv::INTER_LINEAR);
        cv_ptr = cv_bridge::toCvShare(image2);
        cv::remap(cv_ptr->image, copied_image2, map_2x, map_2y,
                  cv::INTER_LINEAR);
      } catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
      }

      std::copy(std::begin(imu_buffer), pos, std::back_inserter(copied_buffer));
      // Clean up the used data.  First delete the buffer of IMU messages,
      // and *then* reset the smart pointers.  Otherwise, you can't find
      // the used synced_imu in the buffer.
      while (!imu_buffer.empty() && !imu_msg_equal(imu_buffer.front()))
        imu_buffer.pop_front();
      if (imu_msg_equal(imu_buffer.front())) imu_buffer.pop_front();
      image1.reset();
      image2.reset();
      is_cleared = true;
    }

    // process
    data_processor(copied_image1, copied_image2, copied_buffer, timestamp);

    std::cout << "    Done; succesfully exiting" << std::endl;
    return true;
  }

 private:
  bool imu_msg_equal(const ORB_SLAM2::IMUData& d) {
    return synced_imu_stamp == d._t;
  };

  ORB_SLAM2::IMUData imu_msg2data(const sensor_msgs::ImuConstPtr& msg) {
    double ax = gravity_multiplier * msg->linear_acceleration.x;
    double ay = gravity_multiplier * msg->linear_acceleration.y;
    double az = gravity_multiplier * msg->linear_acceleration.z;
    return ORB_SLAM2::IMUData(msg->angular_velocity.x, msg->angular_velocity.y,
                              msg->angular_velocity.z, ax, ay, az,
                              msg->header.stamp.toSec());
  }

  std::mutex buffer_mutex;
  std::mutex synchronize_mutex;

  Status status;
  ros::Time imu_time_start;

  std::deque<ORB_SLAM2::IMUData> imu_buffer;

  sensor_msgs::ImageConstPtr image1;
  sensor_msgs::ImageConstPtr image2;
  double synced_imu_stamp;

  message_filters::Subscriber<sensor_msgs::Image> image1_subscriber;
  message_filters::Subscriber<sensor_msgs::Image> image2_subscriber;
  message_filters::Subscriber<sensor_msgs::Imu> imu_subscriber;

  using sync_pol = message_filters::sync_policies::ApproximateTime<
      sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::Imu>;
  message_filters::Synchronizer<sync_pol> sync;

  std::function<CallbackType> data_processor;

  double gravity_multiplier;
  cv::Mat map_1x;
  cv::Mat map_1y;
  cv::Mat map_2x;
  cv::Mat map_2y;
};

int test_msgfilter(int argc, char** argv) {
  std::cout << "starting" << std::endl;
  ros::init(argc, argv, "Test");
  ros::start();
  if (argc != 3) {
    cerr << endl
         << "Usage: rosrun ORB_SLAM2 Stereo path_to_vocabulary path_to_settings"
         << endl;
    ros::shutdown();
    return EXIT_FAILURE;
  }
  ros::NodeHandle nh;
  ORB_SLAM2::System SLAM(argv[1], argv[2], ORB_SLAM2::System::STEREO, true);
  ORB_SLAM2::ConfigParam config(argv[2]);
  std::cout << "  1" << std::endl;
  SynchroBuffer sb(nh, config,
                   std::bind(&ORB_SLAM2::System::TrackStereoVI, &SLAM,
                             std::placeholders::_1, std::placeholders::_2,
                             std::placeholders::_3, std::placeholders::_4));
  std::cout << "  end" << std::endl;
  ros::spin();
  ros::shutdown();
  return EXIT_SUCCESS;
}


int timestamp_collector(int argc, char** argv) {
  ros::init(argc, argv, "Test");
  std::ofstream oss(argv[2]);
  ros::NodeHandle nh;
  message_filters::Subscriber<sensor_msgs::Image> sub(nh, argv[1], 2);
  sub.registerCallback([&](const sensor_msgs::ImageConstPtr& m) { oss << m->header.stamp << "\n"; });
  ros::spin();
  ros::shutdown();
  return EXIT_SUCCESS;
}

//int main(int argc, char** argv) { return test_msgfilter(argc, argv); }
int main(int argc, char** argv) { return timestamp_collector(argc, argv); }
