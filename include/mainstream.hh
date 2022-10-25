#pragma once
#include "common.hh"

namespace SSLAM{

typedef pcl::PointXYZ  pointT;
typedef pcl::PointCloud<pointT>  cloudT;

class mainstream : public rclcpp::Node{
    public:
        mainstream();
        rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;

        void LaserCallback(const sensor_msgs::msg::LaserScan::ConstSharedPtr& msg);
        void Laser2PCL(const sensor_msgs::msg::LaserScan::ConstSharedPtr& msg);
        void ComputeAngle(const sensor_msgs::msg::LaserScan::ConstSharedPtr& msg);
        void ICP();

    private:
        cloudT::Ptr current_cloud_;
        cloudT::Ptr last_cloud_;
        bool is_first_frame_ = true;
        size_t num_nan = 0;
        size_t num_total = 0;
        std::vector<float> angle_cos_;
        std::vector<float> angle_sin_;
        Eigen::Matrix4f pose_ = Eigen::Matrix4f::Identity();

};
}