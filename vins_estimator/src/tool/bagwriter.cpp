//
// Created by td on 2021/3/8.
//

#include <stdio.h>
#include <queue>
#include <map>
#include <thread>
#include <mutex>
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/TwistWithCovarianceStamped.h>
#include <iostream>
#include <fstream>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <ros/package.h>

bool WriteImu(std::string& imu_file, std::string topic, rosbag::Bag& bag, double time_offset);
void WriteFeature(std::string& base_file, std::string topic, rosbag::Bag& bag, double time_offset);
bool ReadFile(std::ifstream &ifs, std::string file_path);
int main(int argc, char **argv) {
    ros::init(argc, argv, "bag_writer");
    ros::NodeHandle n("~");
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);
    rosbag::Bag bag;
    double time_offset = 1.65e9;
    std::string base_path = ros::package::getPath("vins") + "/..";
    bag.open(base_path + "/sim_data/sim.bag", rosbag::bagmode::Write);
    std::string sImu_data_file = base_path + "/sim_data/imu_pose.txt";
    WriteImu(sImu_data_file,"/sim/imu/data",bag,time_offset);

    std::string sImu_noise_data_file = base_path + "/sim_data/imu_pose.txt";
    WriteImu(sImu_noise_data_file, "/sim/imu/data_noise", bag, time_offset);

    std::string sFeature_data_file = base_path + "/sim_data/keyframe/pixel_all_points";
    WriteFeature(sFeature_data_file, "/sim/camera/feature", bag, time_offset);

    std::string sFeature_noise_data_file = base_path + "/sim_data/keyframe/pixel_noise_all_points";
    WriteFeature(sFeature_noise_data_file, "/sim/camera/feature_noise", bag, time_offset);

    bag.close();

}
bool ReadFile(std::ifstream &ifs, std::string file_path) {
    ifs.open(file_path, std::ifstream::in);
    if(!ifs)
    {
        std::cout << "无法打开文件: " << std::endl << file_path << std::endl << std::endl;
        return false;
    }
    return false;
}
bool WriteImu(std::string& imu_file, std::string topic, rosbag::Bag& bag, double time_offset){
    std::ifstream ifs_imu;
    ifs_imu.open(imu_file.c_str());
    if (!ifs_imu.is_open())
    {
        std::cerr << "Failed to open imu file! " << imu_file << std::endl;
        return false;
    }

    std::string sImu_line;
    double dStampSec = 0.0;
    Eigen::Vector3d vAcc;
    Eigen::Vector3d vGyr;
    while (std::getline(ifs_imu, sImu_line) && !sImu_line.empty()) // read imu data
    {
        std::istringstream ssImuData(sImu_line);
        Eigen::Quaterniond q;
        Eigen::Vector3d t;
        sensor_msgs::Imu imu;
        ssImuData >> dStampSec >> q.w() >> q.x() >> q.y() >> q.z() >> t(0) >> t(1) >> t(2) >> vGyr.x() >> vGyr.y() >> vGyr.z() >> vAcc.x() >> vAcc.y() >> vAcc.z();
        imu.header.stamp = ros::Time(dStampSec + time_offset);
        imu.angular_velocity.x = vGyr[0];
        imu.angular_velocity.y = vGyr[1];
        imu.angular_velocity.z = vGyr[2];
        imu.linear_acceleration.x = vAcc[0];
        imu.linear_acceleration.y = vAcc[1];
        imu.linear_acceleration.z = vAcc[2];
        std::cout << "Imu t: " << std::fixed << imu.header.stamp.toSec() << " gyr: " << vGyr.transpose() << " acc: " << vAcc.transpose() << std::endl;

        bag.write(topic,imu.header.stamp,imu);
    }
    ifs_imu.close();
}
void WriteFeature(std::string& base_file, std::string topic, rosbag::Bag& bag, double time_offset){
    int i=0;
    while (true) {
        std::string feature_file = base_file + "_" + std::to_string(i) + ".txt";
        std::ifstream ifs_feature;
        ifs_feature.open(feature_file.c_str());
        if (!ifs_feature.is_open())
        {
            std::cerr << "Failed to open feature file! " << feature_file << std::endl;
            break;
        }
        std::string sFeature_line;
        double dStampSec = 0.0;
        Eigen::Vector3d vAcc;
        Eigen::Vector3d vGyr;
        sensor_msgs::PointCloud features;
        while (std::getline(ifs_feature, sFeature_line) && !sFeature_line.empty()) // read imu data
        {
            std::istringstream ssFeatureData(sFeature_line);
            Eigen::Quaterniond q;
            Eigen::Vector3d t;
            geometry_msgs::Point32_<std::allocator<void>> feature;
            Eigen::Vector4d point;//useless
            ssFeatureData >> point(0) >> point(1) >> point(2) >> point(3) >> feature.x >> feature.y >> dStampSec;
            features.points.push_back(feature);
        }
        features.header.stamp = ros::Time(dStampSec+time_offset);
        bag.write(topic, features.header.stamp, features);
        std::cout << "features t: " << std::fixed << features.header.stamp.toSec() << std::endl;
        ifs_feature.close();
        i++;
    }
}