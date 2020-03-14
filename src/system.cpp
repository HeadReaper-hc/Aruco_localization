//
// Created by hc on 2020/3/13.
//

#include <system.h>

#include <ros/ros.h>

#include <opencv2/core/eigen.hpp>

#include <g2o_types.h>

namespace Aruco_LOCA{

    System::System(ros::NodeHandle& nh_, CameraParameter *cam_, int markNum_, int markSize_, double markLength_)
            : nh(nh_), cam(cam_), markNum(markNum_), markSize(markSize_), markLength(markLength_)
     {
        if(markSize == 6) {
            dictionary_ = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
        }
        else{
            ROS_ERROR_STREAM("Current markSize is not supported!");
        }
        mbMapBuildFinish = false;

         /***** 初始化消息发布 *****/
         landmarkPub = nh.advertise<visualization_msgs::MarkerArray>( "arucoLocalization/landmark", 1);
         robotPosePub = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("arucolocalization/pose", 1);
         cameraPosePub = nh.advertise<visualization_msgs::Marker>("aruco_localization/camPose",1);
         image_transport::ImageTransport it(nh);
         imgPub = it.advertise("arucoLocalization/image", 1);
    }

    void System::addImage(const cv::Mat &img) {

        currImage = img.clone();

        ROS_INFO_STREAM("\n Please move robot around, need to build map first.... \n");

        std::vector<std::vector<cv::Point2f>> marker_corners;
        std::vector<int> IDs;
        //from marker to camera
        std::vector<cv::Vec3d> rvs, tvs;
        cv::aruco::detectMarkers(currImage, dictionary_ , marker_corners, IDs);
        cv::aruco::estimatePoseSingleMarkers(marker_corners, markLength, cam->getcvK(), cam->getcvDist(), rvs, tvs);

        cout<<"Detect Mark Size In Current Image: "<<IDs.size()<<endl;

        if(IDs.size()>0){
            cout<<"Mark ID: ";
        }
        for(int i=0;i<IDs.size();i++) {
            cout<<IDs[i]<<" ";
        }
        if(IDs.size()>0) {
            cout << endl;
        }

        currMarkImage = img.clone();
        cv::aruco::drawDetectedMarkers(currMarkImage, marker_corners, IDs);
        for(size_t i=0; i<IDs.size(); i++)
            cv::aruco::drawAxis(currMarkImage, cam->getcvK(), cam->getcvDist(), rvs[i], tvs[i], 0.07);

        //publish mark detect image
        sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", currMarkImage).toImageMsg();
        imgPub.publish(msg);

        //make eigen Pose matrix
        std::vector<Eigen::Matrix4d> Tca;
        makeEigenPose(Tca, rvs, tvs);

        //debug
//         cv::namedWindow("Mark Image Window",WINDOW_AUTOSIZE);
//         cv::imshow("Mark Image Window",currMarkImage);
//         cv::waitKey(10);

        //debug
//         for(int i=0;i<IDs.size();i++){
//             cout<< Tca[i] <<endl;
//         }

       if(!mbMapBuildFinish){ /***** Map built not finished *****/
           //add marker to map
           bool findDetectMarkInMap = false;
           int  findDetectMarkId = -1;
           map<int, Matrix4d> currMeasure;
           for(int i=0;i<IDs.size();i++){
               //set fix marker
               if( IDs.size()>0 && mapMarkIdPose.empty() ){
                   fixMarkerId = IDs[0];
                   fixMarkerPose = Eigen::Matrix4d::Identity();  //Twa
                   mapMarkIdPose.insert(make_pair(fixMarkerId, fixMarkerPose));
                   currMeasure.insert(make_pair(IDs[0], Tca[0]));
                   Twc = fixMarkerPose * Tca[0].inverse();
                   findDetectMarkInMap = true;
                   findDetectMarkId = 0;
                   break;
               }

               if(!findDetectMarkInMap) {
                   std::map<int, Eigen::Matrix4d>::iterator iter = mapMarkIdPose.find(IDs[i]);
                   if (iter == mapMarkIdPose.end()) {
                       continue;
                   } else {
                       findDetectMarkId = iter->first;
                       findDetectMarkInMap = true;
                       Twc = mapMarkIdPose[findDetectMarkId] * Tca[i].inverse();
                       break;
                   }
               }
           }

           if(findDetectMarkInMap){
               int oldMapSize = mapMarkIdPose.size();
               for(int i=0;i<IDs.size();i++){
                   std::map<int,Eigen::Matrix4d>::iterator iter = mapMarkIdPose.find(IDs[i]);
                   if(iter==mapMarkIdPose.end()){
                       Eigen::Matrix4d Twa;
                       Twa = Twc * Tca[i];
                       mapMarkIdPose.insert(make_pair(IDs[i],Twa));
                       currMeasure.insert(make_pair(IDs[i],Tca[i]));
                   }
                   else{
                       continue;
                   }
               }
               ROS_INFO_STREAM("Old map size: "<<oldMapSize<<" New map size: "<<mapMarkIdPose.size());
           }
           else{
               ROS_WARN_STREAM("Not find any new markers in this frame!");
           }

           //optimize
           optimize(mapMarkIdPose,currMeasure);

           //publish map
           publishMap();

           //publish camera
           publishCameraPose();

           if(mapMarkIdPose.size()==markNum){
               ROS_INFO_STREAM("All the marker is detected, the map building is finished!");
               mbMapBuildFinish = true;
           }
       }
       else{  /***** map built finished ******/
           map<int, Matrix4d> currMeasure;
           for(int i=0;i<IDs.size();i++){
               currMeasure.insert(make_pair(IDs[i],Tca[i]));
           }

           optimize(mapMarkIdPose, currMeasure);

           publishMap();

           publishCameraPose();
       }

    }

    void System::publishMap() {
        if(mapMarkIdPose.size()==0) {
            return ;
        }

        visualization_msgs::MarkerArray marker_array;
        marker_array.markers.clear();
        std::map<int,Eigen::Matrix4d>::iterator iter = mapMarkIdPose.begin();
        for(int i=0;i<mapMarkIdPose.size();i++,iter++) {

            visualization_msgs::Marker marker;
            marker.header.frame_id = "/landmark";
            marker.header.stamp = ros::Time::now();
            marker.ns = "";
            marker.color.r = 0.0f;
            marker.color.g = 0.0f;
            marker.color.b = 1.0f;
            marker.color.a = 1.0;
           // marker.lifetime = ros::Duration();
            marker.frame_locked = true;
            marker.type = visualization_msgs::Marker::CUBE;
            marker.action = visualization_msgs::Marker::ADD;
            marker.id = i;

            //marker pose
            Eigen::Matrix4d Twa = iter->second;
            Eigen::Quaterniond qwa(Twa.block<3,3>(0,0));
            Eigen::Vector3d twa = Twa.block<3,1>(0,3);
           // cout<<"Twa: "<<twa[0]<<" "<<twa[1]<<" "<<twa[2]<<endl;
            marker.pose.position.x = twa[0];
            marker.pose.position.y = twa[1];
            marker.pose.position.z = twa[2];
            marker.pose.orientation.w = qwa.w();
            marker.pose.orientation.x = qwa.x();
            marker.pose.orientation.y = qwa.y();
            marker.pose.orientation.z = qwa.z();

            marker.scale.x = 0.6;
            marker.scale.y = 0.6;
            marker.scale.z = 0.1;
            marker_array.markers.push_back(marker);
        }

        landmarkPub.publish(marker_array);
    }

    void System::optimize(map<int, Matrix4d>& currMap, map<int, Matrix4d>& currMeasure){
        typedef g2o::BlockSolver<g2o::BlockSolverTraits<6,6>> BlockSolverType;
        typedef g2o::LinearSolverEigen<BlockSolverType::PoseMatrixType> LinearSolverType;
        auto solver = new g2o::OptimizationAlgorithmLevenberg(g2o::make_unique<BlockSolverType>(
                g2o::make_unique<LinearSolverType>()));
        g2o::SparseOptimizer optimizer;
        optimizer.setAlgorithm(solver);
       // optimizer.setVerbose(true);

       //add landmark vertex
       int MaxID = -1;
       map<int, Matrix4d>::iterator iter = currMap.begin();
        for(int i=0;i<currMap.size();i++, iter++){
            VertexSE3LieAlegebra* v = new VertexSE3LieAlegebra();
            v->setId(iter->first);
            SE3d temp(iter->second);
            v->setEstimate(temp);
            optimizer.addVertex(v);

            if(iter->first==fixMarkerId){
                v->setFixed(true);
            }

            if(MaxID < iter->first){
                MaxID = iter->first;
            }

//            map<int, Matrix4d>::iterator meaIter = currMeasure.begin();
//            meaIter = currMeasure.find(iter->first);
//            if(meaIter==currMeasure.end()){
//                v->setFixed(true);
//            }
        }

        //add camera vertex
        VertexSE3LieAlegebra* cameraVertex = new VertexSE3LieAlegebra();
        cameraVertex->setId(MaxID+1);
        SE3d lieTwc(Twc);
        cameraVertex->setEstimate(lieTwc);
        optimizer.addVertex(cameraVertex);


        vector<int> vertexId;
        vector<Matrix4d> vertexEst;
        iter = currMap.begin();
        for(;iter!=currMap.end();iter++){
            vertexId.push_back(iter->first);
            vertexEst.push_back(iter->second);
        }

        //add camera
        vertexId.push_back(MaxID+1);
        vertexEst.push_back(Twc);

        //add edge in currMap
        int k = 0;
        for(int i=0;i<vertexId.size();i++){
            for(int j=i+1;j<vertexId.size();j++){
                EdgeSE3LieAlgebra* edge = new EdgeSE3LieAlgebra();
                edge->setId(k);
                edge->setVertex(0, optimizer.vertex(vertexId[i]));
                edge->setVertex(1, optimizer.vertex(vertexId[j]));
                SE3d est(vertexEst[j].inverse()*vertexEst[i]);
                edge->setMeasurement(est);
                edge->setInformation(Eigen::Matrix<double,6,6>::Identity());
                optimizer.addEdge(edge);
                k++;
            }
        }

        //add edge in currMeasure
        iter = currMeasure.begin();
        for(;iter!=currMeasure.end();iter++) {
            EdgeSE3LieAlgebra* edge = new EdgeSE3LieAlgebra();
            edge->setId(k);
            edge->setVertex(0, optimizer.vertex(iter->first));
            edge->setVertex(1, optimizer.vertex(MaxID+1));
            SE3d est(iter->second);
            edge->setMeasurement(est);
            edge->setInformation(Eigen::Matrix<double,6,6>::Identity() * 5);
            optimizer.addEdge(edge);
            k++;
        }

        optimizer.initializeOptimization();
        optimizer.optimize(10);

        //recover landmark vertex
        for(int i=0;i<vertexId.size()-1;i++){
            VertexSE3LieAlegebra* v = static_cast<VertexSE3LieAlegebra*>(optimizer.vertex(vertexId[i]));
            currMap[vertexId[i]] = v->estimate().matrix();
        }

        //recover camera vertex
        VertexSE3LieAlegebra* v = static_cast<VertexSE3LieAlegebra*>(optimizer.vertex(MaxID+1));
        Twc = v->estimate().matrix();
    }

    void System::makeEigenPose(vector<Eigen::Matrix4d> &Tca, const vector<cv::Vec3d> &rvs,
                               const vector<cv::Vec3d> &tvs) {
        Tca.resize(rvs.size());
        for(int i=0;i<rvs.size();i++){
            cv::Mat R;
            Eigen::Matrix3d eigenR;
            Eigen::Vector3d eigenT;
            cv::Rodrigues(rvs[i],R);
            cv::cv2eigen(R,eigenR);
            cv::cv2eigen(tvs[i],eigenT);
            Tca[i] = Eigen::Matrix4d::Identity();
            Tca[i].block<3,3>(0,0) = eigenR;
            Tca[i].block<3,1>(0,3) = eigenT;
        }
    }

    void System::publishCameraPose() {
        visualization_msgs::Marker marker;
        marker.header.frame_id = "/landmark";
        marker.header.stamp = ros::Time::now();
        marker.ns = "";
        marker.color.r = 1.0f;
        marker.color.g = 0.0f;
        marker.color.b = 0.0f;
        marker.color.a = 1.0;
        // marker.lifetime = ros::Duration();
        marker.frame_locked = true;
        marker.type = visualization_msgs::Marker::CUBE;
        marker.action = visualization_msgs::Marker::ADD;
        marker.id = 0;

        //marker pose
        Eigen::Quaterniond qwc(Twc.block<3,3>(0,0));
        Eigen::Vector3d twc = Twc.block<3,1>(0,3);
        // cout<<"Twa: "<<twa[0]<<" "<<twa[1]<<" "<<twa[2]<<endl;
        marker.pose.position.x = twc[0];
        marker.pose.position.y = twc[1];
        marker.pose.position.z = twc[2];
        marker.pose.orientation.w = qwc.w();
        marker.pose.orientation.x = qwc.x();
        marker.pose.orientation.y = qwc.y();
        marker.pose.orientation.z = qwc.z();

        marker.scale.x = 0.4;
        marker.scale.y = 0.1;
        marker.scale.z = 0.1;

        cameraPosePub.publish(marker);
    }
}

