//
// Created by hc on 2020/3/13.
//

#ifndef ARUCO_LOCALIZATION_CAMERAPARAMETER_H
#define ARUCO_LOCALIZATION_CAMERAPARAMETER_H

//stl
#include <vector>

//eigen
#include <Eigen/Dense>

//opencv
#include <opencv2/opencv.hpp>

using namespace std;
using namespace Eigen;

namespace Aruco_LOCA{

    class CameraParameter{
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        CameraParameter(double fx_, double fy_, double cx_, double cy_, vector<double> distortion_, vector<double> Tbc_)
                       : fx(fx_), fy(fy_), cx(cx_), cy(cy_), distortion(distortion_)
        {
            K << fx, 0, cx,
                 0, fy, cy,
                 0,  0,  1;
            invK = K.inverse();

            cvK = (cv::Mat_<float>(3, 3) << fx, 0, cx, 0, fy, cy, 0, 0, 1);

            cvDist = (cv::Mat_<float>(5, 1) << distortion[0], distortion[1], distortion[2], distortion[3], distortion[4]);

            assert( Tbc_.size() == 16 );

            Tbc << Tbc_[0],  Tbc_[1],  Tbc_[2],  Tbc_[3],
                   Tbc_[4],  Tbc_[5],  Tbc_[6],  Tbc_[7],
                   Tbc_[8],  Tbc_[9],  Tbc_[10], Tbc_[11],
                   Tbc_[12], Tbc_[13], Tbc_[14], Tbc_[15];

            Tcb.block<3,3>(0,0) = Tbc.block<3,3>(0,0).transpose();
            Tcb.block<3,1>(0,3) = -Tbc.block<3,3>(0,0).transpose() * Tbc.block<3,1>(0,3);

        }

        inline double     getFx()   { return fx; }
        inline double     getFy()   { return fy; }
        inline double     getCx()   { return cx; }
        inline double     getCy()   { return cy; }
        inline Matrix3d   getK()    { return K;  }
        inline Matrix3d   getInvK() { return invK; }
        inline Matrix4d   getTbc()  { return Tbc; }
        inline Matrix4d   getTcb()  { return Tcb; }
        inline cv::Mat    getcvK()  { return cvK; }
        inline cv::Mat    getcvDist() { return cvDist; }

    private:
        double fx;
        double fy;
        double cx;
        double cy;
        vector<double> distortion;
        Matrix3d K;
        Matrix3d invK;
        cv::Mat cvK;
        cv::Mat cvDist;

        Matrix4d Tbc;
        Matrix4d Tcb;
    };
}
#endif //ARUCO_LOCALIZATION_CAMERAPARAMETER_H
