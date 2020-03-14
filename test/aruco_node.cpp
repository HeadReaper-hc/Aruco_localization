//ros
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

//stl
#include <iostream>
#include <string>
#include <vector>

//local head file
#include <cameraParameter.h>
#include <system.h>

using namespace std;
using namespace Aruco_LOCA;

void ImageCallback(const sensor_msgs::ImageConstPtr& imgPtr );
void ImuCallback(const sensor_msgs::ImuConstPtr& imuPtr);

void PrintParam(string& paramName, int& value);
void PrintParam(string& paramName, double& value);
void PrintParam(string& paramName, string& value);
void PrintParam(string& paramName, vector<double>& value);

template <typename T>
T getParam(ros::NodeHandle& n, string paramName);

//global varaible
System* arucoSystem;

int main(int argc, char** argv){

    /***** ROS initialization *****/
    ros::init(argc, argv, "aruco_localization");
    ros::NodeHandle nh;

    /***** Read camera info *****/
    double fx, fy, cx, cy, k1, k2, p1, p2, k3;
    vector<double> Tbc;
    fx = getParam<double>(nh, "camera/fx");
    fy = getParam<double>(nh, "camera/fy");
    cx = getParam<double>(nh, "camera/cx");
    cy = getParam<double>(nh, "camera/cy");
    k1 = getParam<double>(nh, "camera/k1");
    k2 = getParam<double>(nh, "camera/k2");
    p1 = getParam<double>(nh, "camera/p1");
    p2 = getParam<double>(nh, "camera/p2");
    k3 = getParam<double>(nh, "camera/k3");
    Tbc = getParam<vector<double>>(nh, "extrinsic/Tbc");

    /***** Read topic name *****/
    string imageTopic, imuTopic;
    imageTopic = getParam<string>(nh, "topic/image");
    imuTopic = getParam<string>(nh, "topic/imu");

    /***** Read mark parameter *****/
    int markNum, markSize;
    double markLength;
    markNum = getParam<int>(nh, "aruco/n_markers");
    markSize = getParam<int>(nh, "aruco/marker_size");
    markLength = getParam<double>(nh, "aruco/marker_length");

    /***** System initialization *****/
    CameraParameter* cam = new CameraParameter(fx, fy, cx, cy, vector<double>{k1,k2,p1,p1,k3}, Tbc);
    arucoSystem = new System(nh, cam, markNum, markSize, markLength);

    /***** Message subscriber *****/
    ros::Subscriber image_sub = nh.subscribe(imageTopic, 1, ImageCallback);
    ros::Subscriber encoder_sub = nh.subscribe(imuTopic, 1, ImuCallback);

//
//    ros::Rate r(30);
//
//    while(ros::ok()){
//        ros::spinOnce();
//        r.sleep();
//    }
    ros::spin();

    delete arucoSystem;
    delete cam;
}

void ImageCallback ( const sensor_msgs::ImageConstPtr& imgPtr )
{
    //ROS_INFO_STREAM("In the Image Callback....");
    cv_bridge::CvImageConstPtr cv_ptr  = cv_bridge::toCvShare ( imgPtr, sensor_msgs::image_encodings::BGR8 );

    /* add image */
    arucoSystem->addImage(cv_ptr->image);
}

void ImuCallback(const sensor_msgs::ImuConstPtr& imuPtr){

}


template <typename T>
T getParam(ros::NodeHandle& n, string paramName){
    T value;
    if(n.getParam(paramName, value)){
        PrintParam(paramName, value);
    }
    else{
        ROS_ERROR_STREAM("Failed to load " << paramName);
        n.shutdown();
    }
    return value;
}

void PrintParam(string& paramName, int& value){
    ROS_INFO_STREAM( "Loaded " << paramName << ": " << value );
}

void PrintParam(string& paramName, double& value){
    ROS_INFO_STREAM( "Loaded " << paramName << ": " << value );
}

void PrintParam(string& paramName, string& value){
    ROS_INFO_STREAM( "Loaded " << paramName << ": " << value );
}

void PrintParam(string& paramName, vector<double>& value){
    //ROS_INFO_STREAM( "Loaded " << paramName << ": " << value );
    //cout <<"Loaded "<< paramName << ": " << value[0] << value[1] << value[2] << endl;
}
