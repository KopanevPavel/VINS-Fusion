#include "dnnOpt.h"
#include "Factors.h"

DnnOptimization::DnnOptimization()
{
	initDnn = false;
    newDnn = false;
	WDNN_T_WVIO = Eigen::Matrix4d::Identity();
    threadOpt = std::thread(&DnnOptimization::optimize, this);
}

DnnOptimization::~DnnOptimization()
{
    threadOpt.detach();
}

void DnnOptimization::inputOdom(double t, Eigen::Vector3d OdomP, Eigen::Quaterniond OdomQ)
{
	mPoseMap.lock();
    vector<double> localPose{OdomP.x(), OdomP.y(), OdomP.z(), 
    					     OdomQ.w(), OdomQ.x(), OdomQ.y(), OdomQ.z()};
    localPoseMap[t] = localPose;


    Eigen::Quaterniond dnnQ;
    dnnQ = WDNN_T_WVIO.block<3, 3>(0, 0) * OdomQ;
    Eigen::Vector3d dnnP = WDNN_T_WVIO.block<3, 3>(0, 0) * OdomP + WDNN_T_WVIO.block<3, 1>(0, 3);
    vector<double> dnnPose{dnnP.x(), dnnP.y(), dnnP.z(),
                              dnnQ.w(), dnnQ.x(), dnnQ.y(), dnnQ.z()};
    dnnPoseMap[t] = dnnPose;
    lastP = dnnP;
    lastQ = dnnQ;

    geometry_msgs::PoseStamped pose_stamped;
    pose_stamped.header.stamp = ros::Time(t);
    pose_stamped.header.frame_id = "world";
    pose_stamped.pose.position.x = lastP.x();
    pose_stamped.pose.position.y = lastP.y();
    pose_stamped.pose.position.z = lastP.z();
    pose_stamped.pose.orientation.x = lastQ.x();
    pose_stamped.pose.orientation.y = lastQ.y();
    pose_stamped.pose.orientation.z = lastQ.z();
    pose_stamped.pose.orientation.w = lastQ.w();
    dnn_path.header = pose_stamped.header;
    dnn_path.poses.push_back(pose_stamped);

    mPoseMap.unlock();
}

void DnnOptimization::getDnnOdom(Eigen::Vector3d &odomP, Eigen::Quaterniond &odomQ)
{
    odomP = lastP;
    odomQ = lastQ;
}

void DnnOptimization::inputDnn(double t, Eigen::Vector3d DnnP, Eigen::Quaterniond DnnQ)
{
	vector<double> dnnPose{DnnP.x(), DnnP.y(), DnnP.z(),
                              DnnQ.w(), DnnQ.x(), DnnQ.y(), DnnQ.z()};
	dnnPositionMap[t] = dnnPose;
    newDnn = true;
    if (first_pair) {
        first_t = t;
        first_pair = false;
    }
}

void DnnOptimization::optimize()
{
    while(true)
    {
        if(newDnn)
        {
            newDnn = false;
            printf("dnn optimization\n");
            TicToc dnnOptimizationTime;

            ceres::Problem problem;
            ceres::Solver::Options options;
            options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
            //options.minimizer_progress_to_stdout = true;
            //options.max_solver_time_in_seconds = SOLVER_TIME * 3;
            options.max_num_iterations = 5;
            ceres::Solver::Summary summary;
            ceres::LossFunction *loss_function;
            loss_function = new ceres::HuberLoss(1.0);
            ceres::LocalParameterization* local_parameterization = new ceres::QuaternionParameterization();

            //add param
            mPoseMap.lock();
            int length = localPoseMap.size();
            // w^t_i   w^q_i
            double t_array[length][3];
            double q_array[length][4];
            map<double, vector<double>>::iterator iter;
            iter = dnnPoseMap.begin();
            for (int i = 0; i < length; i++, iter++)
            {
                t_array[i][0] = iter->second[0];
                t_array[i][1] = iter->second[1];
                t_array[i][2] = iter->second[2];
                q_array[i][0] = iter->second[3];
                q_array[i][1] = iter->second[4];
                q_array[i][2] = iter->second[5];
                q_array[i][3] = iter->second[6];
                problem.AddParameterBlock(q_array[i], 4, local_parameterization);
                problem.AddParameterBlock(t_array[i], 3);
            }

            map<double, vector<double>>::iterator iterVIO, iterVIONext, iterDnn;
            int i = 0;
            int i_prev = 0;
            for (iterVIO = localPoseMap.begin(); iterVIO != localPoseMap.end(); iterVIO++, i++)
            {
                //vio factor
                iterVIONext = iterVIO;
                iterVIONext++;
                if(iterVIONext != localPoseMap.end())
                {
                    Eigen::Matrix4d wTi = Eigen::Matrix4d::Identity();
                    Eigen::Matrix4d wTj = Eigen::Matrix4d::Identity();
                    wTi.block<3, 3>(0, 0) = Eigen::Quaterniond(iterVIO->second[3], iterVIO->second[4], 
                                                               iterVIO->second[5], iterVIO->second[6]).toRotationMatrix();
                    wTi.block<3, 1>(0, 3) = Eigen::Vector3d(iterVIO->second[0], iterVIO->second[1], iterVIO->second[2]);
                    wTj.block<3, 3>(0, 0) = Eigen::Quaterniond(iterVIONext->second[3], iterVIONext->second[4], 
                                                               iterVIONext->second[5], iterVIONext->second[6]).toRotationMatrix();
                    wTj.block<3, 1>(0, 3) = Eigen::Vector3d(iterVIONext->second[0], iterVIONext->second[1], iterVIONext->second[2]);
                    Eigen::Matrix4d iTj = wTi.inverse() * wTj;
                    Eigen::Quaterniond iQj;
                    iQj = iTj.block<3, 3>(0, 0);
                    Eigen::Vector3d iPj = iTj.block<3, 1>(0, 3);

                    ceres::CostFunction* vio_function = RelativeRTError::Create(iPj.x(), iPj.y(), iPj.z(),
                                                                                iQj.w(), iQj.x(), iQj.y(), iQj.z(),
                                                                                0.1, 0.01);
                    problem.AddResidualBlock(vio_function, NULL, q_array[i], t_array[i], q_array[i+1], t_array[i+1]);

                    /*
                    double **para = new double *[4];
                    para[0] = q_array[i];
                    para[1] = t_array[i];
                    para[3] = q_array[i+1];
                    para[4] = t_array[i+1];

                    double *tmp_r = new double[6];
                    double **jaco = new double *[4];
                    jaco[0] = new double[6 * 4];
                    jaco[1] = new double[6 * 3];
                    jaco[2] = new double[6 * 4];
                    jaco[3] = new double[6 * 3];
                    vio_function->Evaluate(para, tmp_r, jaco);

                    std::cout << Eigen::Map<Eigen::Matrix<double, 6, 1>>(tmp_r).transpose() << std::endl
                        << std::endl;
                    std::cout << Eigen::Map<Eigen::Matrix<double, 6, 4, Eigen::RowMajor>>(jaco[0]) << std::endl
                        << std::endl;
                    std::cout << Eigen::Map<Eigen::Matrix<double, 6, 3, Eigen::RowMajor>>(jaco[1]) << std::endl
                        << std::endl;
                    std::cout << Eigen::Map<Eigen::Matrix<double, 6, 4, Eigen::RowMajor>>(jaco[2]) << std::endl
                        << std::endl;
                    std::cout << Eigen::Map<Eigen::Matrix<double, 6, 3, Eigen::RowMajor>>(jaco[3]) << std::endl
                        << std::endl;
                    */

                }
                //dnn factor
                double t = iterVIO->first;
                iterDnn = dnnPositionMap.find(t);
                if (iterDnn != dnnPositionMap.end())
                {
                    if (t == first_t) {
                        i_prev = i;
                    }
                    else {
                        ceres::CostFunction* dnn_function = RelativeRTError::Create(iterDnn->second[0], iterDnn->second[1], iterDnn->second[2],
                                                                                    iterDnn->second[3], iterDnn->second[4], iterDnn->second[5], iterDnn->second[6],
                                                                                    0.1, 0.01);
                        //printf("inverse weight %f \n", iterGPS->second[3]);
                        problem.AddResidualBlock(dnn_function, NULL, q_array[i_prev], t_array[i_prev], q_array[i], t_array[i]);
                        i_prev = i;
                    }

                    /*
                    double **para = new double *[1];
                    para[0] = t_array[i];

                    double *tmp_r = new double[3];
                    double **jaco = new double *[1];
                    jaco[0] = new double[3 * 3];
                    gps_function->Evaluate(para, tmp_r, jaco);

                    std::cout << Eigen::Map<Eigen::Matrix<double, 3, 1>>(tmp_r).transpose() << std::endl
                        << std::endl;
                    std::cout << Eigen::Map<Eigen::Matrix<double, 3, 3, Eigen::RowMajor>>(jaco[0]) << std::endl
                        << std::endl;
                    */
                }

            }
            //mPoseMap.unlock();
            ceres::Solve(options, &problem, &summary);
            //std::cout << summary.BriefReport() << "\n";

            // update dnn pose
            //mPoseMap.lock();
            iter = dnnPoseMap.begin();
            for (int i = 0; i < length; i++, iter++)
            {
            	vector<double> dnnPose{t_array[i][0], t_array[i][1], t_array[i][2],
            							  q_array[i][0], q_array[i][1], q_array[i][2], q_array[i][3]};
            	iter->second = dnnPose;
            	if(i == length - 1)
            	{
            	    Eigen::Matrix4d WVIO_T_body = Eigen::Matrix4d::Identity(); 
            	    Eigen::Matrix4d WDNN_T_body = Eigen::Matrix4d::Identity();
            	    double t = iter->first;
            	    WVIO_T_body.block<3, 3>(0, 0) = Eigen::Quaterniond(localPoseMap[t][3], localPoseMap[t][4], 
            	                                                       localPoseMap[t][5], localPoseMap[t][6]).toRotationMatrix();
            	    WVIO_T_body.block<3, 1>(0, 3) = Eigen::Vector3d(localPoseMap[t][0], localPoseMap[t][1], localPoseMap[t][2]);
            	    WDNN_T_body.block<3, 3>(0, 0) = Eigen::Quaterniond(dnnPose[3], dnnPose[4], 
            	                                                        dnnPose[5], dnnPose[6]).toRotationMatrix();
            	    WDNN_T_body.block<3, 1>(0, 3) = Eigen::Vector3d(dnnPose[0], dnnPose[1], dnnPose[2]);
            	    WDNN_T_WVIO = WDNN_T_body * WVIO_T_body.inverse();
            	}
            }
            updateDnnPath();
            printf("dnn time %f \n", dnnOptimizationTime.toc());
            mPoseMap.unlock();
        }
        std::chrono::milliseconds dura(2000);
        std::this_thread::sleep_for(dura);
    }
	return;
}


void DnnOptimization::updateDnnPath()
{
    dnn_path.poses.clear();
    map<double, vector<double>>::iterator iter;
    for (iter = dnnPoseMap.begin(); iter != dnnPoseMap.end(); iter++)
    {
        geometry_msgs::PoseStamped pose_stamped;
        pose_stamped.header.stamp = ros::Time(iter->first);
        pose_stamped.header.frame_id = "world";
        pose_stamped.pose.position.x = iter->second[0];
        pose_stamped.pose.position.y = iter->second[1];
        pose_stamped.pose.position.z = iter->second[2];
        pose_stamped.pose.orientation.w = iter->second[3];
        pose_stamped.pose.orientation.x = iter->second[4];
        pose_stamped.pose.orientation.y = iter->second[5];
        pose_stamped.pose.orientation.z = iter->second[6];
        dnn_path.poses.push_back(pose_stamped);
    }
}