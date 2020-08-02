#include "IMU/configparam.h"

namespace ORB_SLAM2
{
double ConfigParam::_g = 9.80665;

Eigen::Matrix4d ConfigParam::_EigTbc = Eigen::Matrix4d::Identity();
cv::Mat ConfigParam::_MatTbc = cv::Mat::eye(4,4,CV_32F);
Eigen::Matrix4d ConfigParam::_EigTcb = Eigen::Matrix4d::Identity();
cv::Mat ConfigParam::_MatTcb = cv::Mat::eye(4,4,CV_32F);
int ConfigParam::_LocalWindowSize = 10;
double ConfigParam::_ImageDelayToIMU = 0;
bool ConfigParam::_bAccMultiply9p8 = false;
std::string ConfigParam::_tmpFilePath = "";
double ConfigParam::_nVINSInitTime = 15;
bool ConfigParam::_bRealTime = true;

ConfigParam::ConfigParam(std::string configfile)
{
    cv::FileStorage fSettings(configfile, cv::FileStorage::READ);

    std::cout<<std::endl<<std::endl<<"Parameters: "<<std::endl;

    _testDiscardTime = fSettings["test.DiscardTime"];
    _nVINSInitTime = fSettings["test.VINSInitTime"];
    std::cout<<"VINS initialize time: "<<_nVINSInitTime<<std::endl;
    std::cout<<"Discart time in test data: "<<_testDiscardTime<<std::endl;

    fSettings["test.InitVIOTmpPath"] >> _tmpFilePath;
    std::cout<<"save tmp file in "<<_tmpFilePath<<std::endl;

    fSettings["bagfile"] >> _bagfile;
    std::cout<<"open rosbag: "<<_bagfile<<std::endl;
    fSettings["imutopic"] >> _imuTopic;
    fSettings["imagetopic"] >> _imageTopic;
    std::cout<<"imu topic: "<<_imuTopic<<std::endl;
    std::cout<<"image topic: "<<_imageTopic<<std::endl;
    fSettings["imagetopic_left"] >> _imageTopicLeft;
    fSettings["imagetopic_right"] >> _imageTopicRight;

    _LocalWindowSize = fSettings["LocalMapping.LocalWindowSize"];
    std::cout<<"local window size: "<<_LocalWindowSize<<std::endl;

    _ImageDelayToIMU = fSettings["Camera.delaytoimu"];
    std::cout<<"timestamp image delay to imu: "<<_ImageDelayToIMU<<std::endl;

    {
        cv::FileNode Tbc_ = fSettings["Camera.Tbc"];
        Eigen::Matrix<double,3,3> R;
        R << Tbc_[0], Tbc_[1], Tbc_[2],
                Tbc_[4], Tbc_[5], Tbc_[6],
                Tbc_[8], Tbc_[9], Tbc_[10];
        Eigen::Quaterniond qr(R);
        R = qr.normalized().toRotationMatrix();
        Eigen::Matrix<double,3,1> t( Tbc_[3], Tbc_[7], Tbc_[11]);
        _EigTbc = Eigen::Matrix4d::Identity();
        _EigTbc.block<3,3>(0,0) = R;
        _EigTbc.block<3,1>(0,3) = t;
        _MatTbc = cv::Mat::eye(4,4,CV_32F);
        for(int i=0;i<4;i++)
            for(int j=0;j<4;j++)
                _MatTbc.at<float>(i,j) = _EigTbc(i,j);

        _EigTcb = Eigen::Matrix4d::Identity();
        _EigTcb.block<3,3>(0,0) = R.transpose();
        _EigTcb.block<3,1>(0,3) = -R.transpose()*t;
        for(int i=0;i<4;i++)
            for(int j=0;j<4;j++)
                _MatTcb.at<float>(i,j) = _EigTcb(i,j);

        // Tbc_[0], Tbc_[1], Tbc_[2], Tbc_[3], Tbc_[4], Tbc_[5], Tbc_[6], Tbc_[7], Tbc_[8], Tbc_[9], Tbc_[10], Tbc_[11], Tbc_[12], Tbc_[13], Tbc_[14], Tbc_[15];
        std::cout<<"Tbc inited:"<<std::endl<<_EigTbc<<std::endl<<_MatTbc<<std::endl;
        std::cout<<"Tcb inited:"<<std::endl<<_EigTcb<<std::endl<<_MatTcb<<std::endl;
        std::cout<<"Tbc*Tcb:"<<std::endl<<_EigTbc*_EigTcb<<std::endl<<_MatTbc*_MatTcb<<std::endl;
    }

    {
        int tmpBool = fSettings["IMU.multiplyG"];
        _bAccMultiply9p8 = (tmpBool != 0);
        std::cout<<"whether acc*9.8? 0/1: "<<_bAccMultiply9p8<<std::endl;
    }

    {
        int tmpBool = fSettings["test.RealTime"];
        _bRealTime = (tmpBool != 0);
        std::cout<<"whether run realtime? 0/1: "<<_bRealTime<<std::endl;
    }

    {
        cv::Mat K_l, K_r, P_l, P_r, R_l, R_r, D_l, D_r;
        fSettings["LEFT.K"] >> K_l;
        fSettings["RIGHT.K"] >> K_r;

        fSettings["LEFT.P"] >> P_l;
        fSettings["RIGHT.P"] >> P_r;

        fSettings["LEFT.R"] >> R_l;
        fSettings["RIGHT.R"] >> R_r;

        fSettings["LEFT.D"] >> D_l;
        fSettings["RIGHT.D"] >> D_r;

        int rows_l = fSettings["LEFT.height"];
        int cols_l = fSettings["LEFT.width"];
        int rows_r = fSettings["RIGHT.height"];
        int cols_r = fSettings["RIGHT.width"];
        
        if(K_l.empty() || K_r.empty() || P_l.empty() || P_r.empty() || R_l.empty() || R_r.empty() || D_l.empty() || D_r.empty() ||
                rows_l==0 || rows_r==0 || cols_l==0 || cols_r==0)
        {
            std::cerr << "ERROR: Calibration parameters to rectify stereo are missing!" << std::endl;
        }

        cv::initUndistortRectifyMap(K_l,D_l,R_l,P_l.rowRange(0,3).colRange(0,3),cv::Size(cols_l,rows_l),CV_32F,_map_1x, _map_1y);
        cv::initUndistortRectifyMap(K_r,D_r,R_r,P_r.rowRange(0,3).colRange(0,3),cv::Size(cols_r,rows_r),CV_32F,_map_2x, _map_2y);
    }


}

std::string ConfigParam::getTmpFilePath()
{
    return _tmpFilePath;
}

Eigen::Matrix4d ConfigParam::GetEigTbc()
{
    return _EigTbc;
}

cv::Mat ConfigParam::GetMatTbc()
{
    return _MatTbc.clone();
}

Eigen::Matrix4d ConfigParam::GetEigT_cb()
{
    return _EigTcb;
}

cv::Mat ConfigParam::GetMatT_cb()
{
    return _MatTcb.clone();
}

int ConfigParam::GetLocalWindowSize()
{
    return _LocalWindowSize;
}

double ConfigParam::GetImageDelayToIMU()
{
    return _ImageDelayToIMU;
}

bool ConfigParam::GetAccMultiply9p8()
{
    return _bAccMultiply9p8;
}

}
