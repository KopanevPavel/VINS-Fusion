#include "ros/ros.h"
#include <opencv2/opencv.hpp>
//#include "dnnOpt.h"
#include <sensor_msgs/NavSatFix.h>
#include <std_msgs/String.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <iostream>
#include <stdio.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <fstream>
#include <queue>
#include <mutex>
#include <nlohmann/json.hpp>

using json = nlohmann::json;

//DnnOptimization dnnEstimator;
ros::Publisher pub_dnn_odometry, pub_dnn_path;
nav_msgs::Path *dnn_path;
double last_vio_t = -1;
//std::queue<std::pair<std_msgs::String>> dnnQueue;
std::mutex m_buf;


void dnn_callback(const std_msgs::String &dnn_msg)
{
    //printf("gps_callback! \n");
    m_buf.lock();

    std::string dnn_msg_data = dnn_msg.data;

    json j = json::parse(dnn_msg_data);
    
    std::vector<std::vector<double>> cur_keypoints_coords = j[0].get<std::vector<std::vector<double>>>();
    std::vector<std::vector<double>> prev_keypoints_coords = j[1].get<std::vector<std::vector<double>>>(); 
    std::vector<double> scores = j[2].get<std::vector<double>>(); 
    double timestamp = std::stod(j[3].get<std::string>()); 

    std::vector<cv::Point2f> cur_keypoints_coords_vec;
    std::vector<cv::Point2f> prev_keypoints_coords_vec;

    for (size_t i = 0; i < cur_keypoints_coords.size(); i++)
    {   
        cv::Point2f pt_cur(cur_keypoints_coords[i][0], cur_keypoints_coords[i][1]);
        cur_keypoints_coords_vec.push_back(pt_cur);

        cv::Point2f pt_prev(prev_keypoints_coords[i][0], prev_keypoints_coords[i][1]);
        prev_keypoints_coords_vec.push_back(pt_prev);
    } 

    // std::cout << std::fixed << timestamp << std::endl; 

    cv::Mat_<double> cameraMatrix(3, 3); // rows, cols
    cameraMatrix << 816.40221474060002, 0, 608.82658427579997,
                    0, 817.38388562809996, 266.68865652440002,
                    0, 0, 1;

    cv::Mat E, R, t, mask;
    E = cv::findEssentialMat(cur_keypoints_coords_vec, prev_keypoints_coords_vec, cameraMatrix, cv::RANSAC, 0.999, 1.0, mask);
    cv::recoverPose(E, cur_keypoints_coords_vec, prev_keypoints_coords_vec, cameraMatrix, R, t, mask);

    Eigen::Vector3d dnn_t(t.at<double>(0), t.at<double>(1), t.at<double>(2));

    /*
    std::vector<std::vector<double>> R_vec;

    for (size_t i = 0; i < R.size().height; i++)
    {   
        std::vector<double> R_vec_row;
        for (size_t j = 0; j < R.size().width; j++)
        {   
            R_vec_row.push_back(R.at<double>(i, j));
        }   
        R_vec.push_back(R_vec_row);
    } 
    */  

    Eigen::Matrix3f mat_R;

    mat_R << R.at<double>(0, 0), R.at<double>(0, 1), R.at<double>(0, 2),
             R.at<double>(1, 0), R.at<double>(1, 1), R.at<double>(1, 2),
             R.at<double>(2, 0), R.at<double>(2, 1), R.at<double>(2, 2);

    Eigen::Quaternionf dnn_q(mat_R);

    // std::cout << dnn_t[0] << dnn_t[1] << dnn_t[2] << std::endl;

    // gpsQueue.push(GPS_msg);
    m_buf.unlock();
}

void vio_callback(const nav_msgs::Odometry::ConstPtr &pose_msg)
{
    //printf("vio_callback! \n");
    double t = pose_msg->header.stamp.toSec();
    // std::cout << std::fixed << t << std::endl;
    last_vio_t = t;
    Eigen::Vector3d vio_t(pose_msg->pose.pose.position.x, pose_msg->pose.pose.position.y, pose_msg->pose.pose.position.z);
    Eigen::Quaterniond vio_q;
    vio_q.w() = pose_msg->pose.pose.orientation.w;
    vio_q.x() = pose_msg->pose.pose.orientation.x;
    vio_q.y() = pose_msg->pose.pose.orientation.y;
    vio_q.z() = pose_msg->pose.pose.orientation.z;
    /*
    dnnEstimator.inputOdom(t, vio_t, vio_q);


    m_buf.lock();
    while(!gpsQueue.empty())
    {
        sensor_msgs::NavSatFixConstPtr GPS_msg = gpsQueue.front();
        double gps_t = GPS_msg->header.stamp.toSec();
        printf("vio t: %f, gps t: %f \n", t, gps_t);
        // 10ms sync tolerance
        if(gps_t >= t - 0.1 && gps_t <= t + 0.1)
        {
            printf("receive GPS with timestamp %f\n", GPS_msg->header.stamp.toSec());
            double latitude = GPS_msg->latitude;
            double longitude = GPS_msg->longitude;
            double altitude = GPS_msg->altitude;
            //int numSats = GPS_msg->status.service;
            double pos_accuracy = GPS_msg->position_covariance[0];
            if(pos_accuracy <= 0)
                pos_accuracy = 1;
            //printf("receive covariance %lf \n", pos_accuracy);
            //if(GPS_msg->status.status > 8)
                globalEstimator.inputGPS(t, latitude, longitude, altitude, pos_accuracy);
            gpsQueue.pop();
            break;
        }
        else if(gps_t < t - 0.01)
            gpsQueue.pop();
        else if(gps_t > t + 0.01)
            break;
    }
    m_buf.unlock();

    Eigen::Vector3d dnn_t;
    Eigen:: Quaterniond dnn_q;
    dnnEstimator.getDnnOdom(dnn_t, dnn_q);

    nav_msgs::Odometry odometry;
    odometry.header = pose_msg->header;
    odometry.header.frame_id = "world";
    odometry.child_frame_id = "world";
    odometry.pose.pose.position.x = dnn_t.x();
    odometry.pose.pose.position.y = dnn_t.y();
    odometry.pose.pose.position.z = dnn_t.z();
    odometry.pose.pose.orientation.x = dnn_q.x();
    odometry.pose.pose.orientation.y = dnn_q.y();
    odometry.pose.pose.orientation.z = dnn_q.z();
    odometry.pose.pose.orientation.w = dnn_q.w();
    pub_dnn_odometry.publish(odometry);
    pub_dnn_path.publish(*dnn_path);

    */
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "dnnEstimator");
    ros::NodeHandle n("~");

    // dnn_path = &dnnEstimator.dnn_path;

    ros::Subscriber sub_dnn = n.subscribe("/superglue/matches/all_data_left", 100, dnn_callback);
    ros::Subscriber sub_vio = n.subscribe("/vins_estimator/odometry", 100, vio_callback);

    pub_dnn_path = n.advertise<nav_msgs::Path>("dnn_path", 100);
    pub_dnn_odometry = n.advertise<nav_msgs::Odometry>("dnn_odometry", 100);

    ros::spin();
    return 0;
}
