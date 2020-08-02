#include "StereoMsgSynchronizer.h"
#include "../../../include/IMU/configparam.h"

namespace ORBVIO
{

StereoMsgSynchronizer::StereoMsgSynchronizer(const double& imagedelay):
    _imageMsgDelaySec(imagedelay), _status(NOTINIT),
    _dataUnsyncCnt(0)
{
    printf("image delay set as %.1fms\n",_imageMsgDelaySec*1000);
}

StereoMsgSynchronizer::~StereoMsgSynchronizer()
{

}


bool StereoMsgSynchronizer::getRecentMsgs(sensor_msgs::ImageConstPtr &left_img, sensor_msgs::ImageConstPtr &right_img, std::vector<sensor_msgs::ImuConstPtr> &vimumsgs)
{
    //unique_lock<mutex> lock2(_mutexIMUQueue);
    unique_lock<mutex> lock1(_mutexImageQueue);

    if(_status == NOTINIT || _status == INIT)
    {
        //ROS_INFO("synchronizer not inited");
        return false;
    }
    if(_leftImageMsgQueue.empty())
    {
        //ROS_INFO("no image stored in queue currently.");
        return false;
    }
    if(_imuMsgQueue.empty())
    {
        //ROS_WARN("no imu message stored, shouldn't");
        return false;
    }

    {
        sensor_msgs::ImageConstPtr imsg;
        sensor_msgs::ImuConstPtr bmsg;

        //
        imsg = _leftImageMsgQueue.back();
        bmsg = _imuMsgQueue.front();

        // Check dis-continuity, tolerance 3 seconds
        if(imsg->header.stamp.toSec()-_imageMsgDelaySec + 3.0 < bmsg->header.stamp.toSec() )
        {
            ROS_ERROR("Data dis-continuity, > 3 seconds. Buffer cleared");
            clearMsgs();
            return false;
        }

        //
        imsg = _leftImageMsgQueue.front();
        bmsg = _imuMsgQueue.back();

        // Wait imu messages in case communication block
        if(imsg->header.stamp.toSec()-_imageMsgDelaySec > bmsg->header.stamp.toSec())
        {
            return false;
        }

        // Check dis-continuity, tolerance 3 seconds
        if(imsg->header.stamp.toSec()-_imageMsgDelaySec > bmsg->header.stamp.toSec() + 3.0)
        {
            ROS_ERROR("Data dis-continuity, > 3 seconds. Buffer cleared");
            clearMsgs();
            return false;
        }

        // Wait until the imu packages totolly com
        if(_leftImageMsgQueue.size()<10 && _imuMsgQueue.size()<15
           && imsg->header.stamp.toSec()-_imageMsgDelaySec>bmsg->header.stamp.toSec() )
        {
            //ROS_WARN_STREAM("here return, last imu time "<<);
            return false;

        }

    }

    // get image message
    left_img = _leftImageMsgQueue.front();
    _leftImageMsgQueue.pop();
    right_img = _rightImageMsgQueue.front();
    _rightImageMsgQueue.pop();

    // clear imu message vector, and push all imu messages whose timestamp is earlier than image message
    vimumsgs.clear();
    while(true)
    {
        // if no more imu messages, stop loop
        if(_imuMsgQueue.empty())
            break;

        // consider delay between image and imu serial
        sensor_msgs::ImuConstPtr tmpimumsg = _imuMsgQueue.front();
        if(tmpimumsg->header.stamp.toSec() < left_img->header.stamp.toSec() - _imageMsgDelaySec)
        {
            // add to imu message vector
            vimumsgs.push_back(tmpimumsg);
            {
                unique_lock<mutex> lock(_mutexIMUQueue);
                _imuMsgQueue.pop();
            }

            _dataUnsyncCnt = 0;
        }
        else
        {
            if(_dataUnsyncCnt++>10)
            {
                _dataUnsyncCnt = 0;
                //_imuMsgQueue = std::queue<sensor_msgs::ImuConstPtr>();
                clearMsgs();
                ROS_ERROR("data unsynced many times, reset sync");
                return false;
            }
            // stop loop
            break;
        }
    }

    // the camera fps 20Hz, imu message 100Hz. so there should be not more than 5 imu messages between images
    if(vimumsgs.size()>10)
        ROS_WARN("%lu imu messages between images, note",vimumsgs.size());
    if(vimumsgs.size()==0)
        ROS_ERROR("no imu message between images!");

    return true;
}

void StereoMsgSynchronizer::addImuMsg(const sensor_msgs::ImuConstPtr &imumsg)
{
    unique_lock<mutex> lock(_mutexIMUQueue);

    if(_imageMsgDelaySec>=0) {
        _imuMsgQueue.push(imumsg);
        if(_status == NOTINIT)
        {
            _imuMsgTimeStart = imumsg->header.stamp;
            _status = INIT;
        }
    }
    else {
        // if there's no image messages, don't add image
        if(_status == NOTINIT)
            return;
        else if(_status == INIT)
        {
            // ignore all image messages with no imu messages between them
            // only add below images
            if(imumsg->header.stamp.toSec() + _imageMsgDelaySec > _imuMsgTimeStart.toSec())
            {
                _imuMsgQueue.push(imumsg);
                _status = NORMAL;
            }
        }
        else
        {
            // push message into queue
            _imuMsgQueue.push(imumsg);
        }
    }


}

void StereoMsgSynchronizer::addImageMsgs(const sensor_msgs::ImageConstPtr &left_img, const sensor_msgs::ImageConstPtr &right_img)
{
    unique_lock<mutex> lock(_mutexImageQueue);

    if(_imageMsgDelaySec >= 0) {
        // if there's no imu messages, don't add image
        if(_status == NOTINIT)
            return;
        else if(_status == INIT)
        {
            // ignore all image messages with no imu messages between them
            // only add below images
            if(left_img->header.stamp.toSec() - _imageMsgDelaySec > _imuMsgTimeStart.toSec())
            {
                _leftImageMsgQueue.push(left_img);
                _rightImageMsgQueue.push(right_img);
                _status = NORMAL;
            }
        }
        else
        {
            // push message into queue
            _leftImageMsgQueue.push(left_img);
            _rightImageMsgQueue.push(right_img);
        }
    }
    else {  // start by image message
        if(_status == NOTINIT)
        {
            _imuMsgTimeStart = left_img->header.stamp;
            _status = INIT;
        }
        else
        {   // no image data if there's no imu message
            _leftImageMsgQueue.push(left_img);
            _rightImageMsgQueue.push(right_img);
        }

    }

    if(ORB_SLAM2::ConfigParam::GetRealTimeFlag())
    {
        // Ignore earlier frames
        if(_leftImageMsgQueue.size()>2)
        {
            _leftImageMsgQueue.pop();
            _rightImageMsgQueue.pop();
        }
    }
}


void StereoMsgSynchronizer::addLeftImageMsg(const sensor_msgs::ImageConstPtr &left_img)
{
    unique_lock<mutex> lock(_mutexImageQueue);

    if(_imageMsgDelaySec >= 0) {
        // if there's no imu messages, don't add image
        if(_status == NOTINIT)
            return;
        else if(_status == INIT || _status == NORMAL_LEFT || _status == NORMAL_RIGHT)
        {
            // ignore all image messages with no imu messages between them
            // only add below images
            if(left_img->header.stamp.toSec() - _imageMsgDelaySec > _imuMsgTimeStart.toSec())
            {
                _leftImageMsgQueue.push(left_img);
                if(_status == NORMAL_RIGHT) {
                  _status = NORMAL;
                }
                else
                {
                  _status = NORMAL_LEFT;
                }
            }
        }
        else
        {
            // push message into queue
            _leftImageMsgQueue.push(left_img);
        }
    }
    else {  // start by image message
        if(_status == NOTINIT)
        {
            _imuMsgTimeStart = left_img->header.stamp;
            _status = INIT;
        }
        else
        {   // no image data if there's no imu message
            _leftImageMsgQueue.push(left_img);
        }

    }

    if(ORB_SLAM2::ConfigParam::GetRealTimeFlag())
    {
        // Ignore earlier frames
        if(_leftImageMsgQueue.size()>2)
        {
            _leftImageMsgQueue.pop();
        }
    }
}


void StereoMsgSynchronizer::addRightImageMsg(const sensor_msgs::ImageConstPtr &right_img)
{
    unique_lock<mutex> lock(_mutexImageQueue);

    if(_imageMsgDelaySec >= 0) {
        // if there's no imu messages, don't add image
        if(_status == NOTINIT)
            return;
        else if(_status == INIT || _status == NORMAL_LEFT || _status == NORMAL_RIGHT)
        {
            // ignore all image messages with no imu messages between them
            // only add below images
            if(right_img->header.stamp.toSec() - _imageMsgDelaySec > _imuMsgTimeStart.toSec())
            {
                _rightImageMsgQueue.push(right_img);
                if(_status == NORMAL_RIGHT) {
                  _status = NORMAL;
                }
                else
                {
                  _status = NORMAL_RIGHT;
                }
            }
        }
        else
        {
            // push message into queue
            _rightImageMsgQueue.push(right_img);
        }
    }
    else {  // start by image message
        if(_status == NOTINIT)
        {
            _imuMsgTimeStart = right_img->header.stamp;
            _status = INIT;
        }
        else
        {   // no image data if there's no imu message
            _rightImageMsgQueue.push(right_img);
        }

    }

    if(ORB_SLAM2::ConfigParam::GetRealTimeFlag())
    {
        // Ignore earlier frames
        if(_rightImageMsgQueue.size()>2)
        {
            _rightImageMsgQueue.pop();
        }
    }
}


void StereoMsgSynchronizer::imagesCallback(const sensor_msgs::ImageConstPtr &left_img, const sensor_msgs::ImageConstPtr &right_img)
{
    addImageMsgs(left_img, right_img);
}

void StereoMsgSynchronizer::leftImageCallback(const sensor_msgs::ImageConstPtr &left_img)
{
    addLeftImageMsg(left_img);
}

void StereoMsgSynchronizer::rightImageCallback(const sensor_msgs::ImageConstPtr &right_img)
{
    addRightImageMsg(right_img);
}

void StereoMsgSynchronizer::imuCallback(const sensor_msgs::ImuConstPtr &msg)
{
    addImuMsg(msg);
}

void StereoMsgSynchronizer::clearMsgs(void)
{
    _imuMsgQueue = std::queue<sensor_msgs::ImuConstPtr>();
    _leftImageMsgQueue = std::queue<sensor_msgs::ImageConstPtr>();
    _rightImageMsgQueue = std::queue<sensor_msgs::ImageConstPtr>();
}

}

