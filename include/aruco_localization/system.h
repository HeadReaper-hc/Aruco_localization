//
// Created by hc on 2020/3/13.
//

#ifndef ARUCO_LOCALIZATION_SYSTEM_H
#define ARUCO_LOCALIZATION_SYSTEM_H

//stl
#include <map>
#include <vector>

//Eigen
#include <Eigen/Dense>

//opencv
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>

#include <cameraParameter.h>

//ros
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

using namespace std;
using namespace Eigen;
using namespace cv;

namespace Aruco_LOCA{

    class System{
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        System(ros::NodeHandle& nh_, CameraParameter* cam_, int markNum_, int markSize_, double markLength_);

        void addImage(const cv::Mat& img);

        cv::Mat markedImg(){return currMarkImage;}

        void publishMap();

        void optimize(map<int, Matrix4d>& currMap, map<int, Matrix4d>& currMeasure);

        void makeEigenPose(vector<Eigen::Matrix4d>& Tca, const vector<cv::Vec3d>& rvs, const vector<cv::Vec3d>& tvs);

        void publishCameraPose();
    private:

        CameraParameter* cam;

        cv::Mat currImage;
        cv::Mat currMarkImage;

        //mark parameters
        int markNum;
        int markSize;
        double markLength;

        //map
        map<int, Matrix4d> mapMarkIdPose;
        bool mbMapBuildFinish;

        cv::Ptr<cv::aruco::Dictionary> dictionary_;

        //camera Pose
        Eigen::Matrix4d Twc;

        //fix marker id,Pose
        int fixMarkerId;
        Eigen::Matrix4d fixMarkerPose;

        //advertise
        ros::NodeHandle nh;
        ros::Publisher landmarkPub;
        ros::Publisher robotPosePub;
        ros::Publisher cameraPosePub;
        image_transport::Publisher imgPub;

    };
}
#endif //ARUCO_LOCALIZATION_SYSTEM_H
