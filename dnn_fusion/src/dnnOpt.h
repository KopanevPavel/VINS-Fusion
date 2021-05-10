#pragma once
#include <vector>
#include <map>
#include <iostream>
#include <mutex>
#include <thread>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <ceres/ceres.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include "tic_toc.h"

using namespace std;

class DnnOptimization
{
public:
	DnnOptimization();
	~DnnOptimization();
	void inputDnn(double t, Eigen::Vector3d DnnP, Eigen::Quaterniond DnnQ);
	void inputOdom(double t, Eigen::Vector3d OdomP, Eigen::Quaterniond OdomQ);
	void getDnnOdom(Eigen::Vector3d &odomP, Eigen::Quaterniond &odomQ);
	nav_msgs::Path dnn_path;

private:
	void optimize();
	void updateDnnPath();

	// format t, tx,ty,tz,qw,qx,qy,qz
	map<double, vector<double>> localPoseMap;
	map<double, vector<double>> dnnPoseMap;
	map<double, vector<double>> dnnPositionMap;
	bool initDnn;
	bool newDnn;
	std::mutex mPoseMap;
	Eigen::Matrix4d WDNN_T_WVIO;
	Eigen::Vector3d lastP;
	Eigen::Quaterniond lastQ;
	std::thread threadOpt;
	bool first_pair = true;
	double first_t;
};