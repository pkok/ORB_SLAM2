#pragma once

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <queue>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <mutex>

using namespace std;

namespace ORBVIO
{
class StereoMsgSynchronizer
{
public:
    enum Status{
        NOTINIT = 0,
        INIT,
        NORMAL_LEFT,
        NORMAL_RIGHT,
        NORMAL,
    };

    StereoMsgSynchronizer(const double& imagedelay = 0.);
    ~StereoMsgSynchronizer();

    // add messages in callbacks
    void addImageMsgs(const sensor_msgs::ImageConstPtr &left_img, const sensor_msgs::ImageConstPtr &right_img);
    void addLeftImageMsg(const sensor_msgs::ImageConstPtr &left_img);
    void addRightImageMsg(const sensor_msgs::ImageConstPtr &right_img);
    void addImuMsg(const sensor_msgs::ImuConstPtr &imumsg);

    // loop in main function to handle all messages
    bool getRecentMsgs(sensor_msgs::ImageConstPtr &left_img, sensor_msgs::ImageConstPtr &right_img, std::vector<sensor_msgs::ImuConstPtr> &vimumsgs);

    void clearMsgs(void);

    // for message callback if needed
    void imagesCallback(const sensor_msgs::ImageConstPtr &left_img, const sensor_msgs::ImageConstPtr &right_img);
    void leftImageCallback(const sensor_msgs::ImageConstPtr &left_img);
    void rightImageCallback(const sensor_msgs::ImageConstPtr &right_img);
    void imuCallback(const sensor_msgs::ImuConstPtr& msg);

    //
    inline Status getStatus(void) {return _status;}

    double getImageDelaySec(void) const {return _imageMsgDelaySec;}

private:
    double _imageMsgDelaySec;  // image message delay to imu message, in seconds
    std::mutex _mutexImageQueue;
    std::queue<sensor_msgs::ImageConstPtr > _leftImageMsgQueue;
    std::queue<sensor_msgs::ImageConstPtr > _rightImageMsgQueue;
    std::mutex _mutexIMUQueue;
    std::queue<sensor_msgs::ImuConstPtr> _imuMsgQueue;
    ros::Time _imuMsgTimeStart;
    Status _status;
    int _dataUnsyncCnt;
};

}
