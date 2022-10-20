#include "mainstream.hh"

using namespace SSLAM;

mainstream::mainstream():Node("simple_slam"){
    laser_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>("laser",1,std::bind(&mainstream::LaserCallback,this,std::placeholders::_1));
    std::cout<<"1\n";
    cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("cloud",1);
    std::cout<<"2\n";
}

void mainstream::LaserCallback(const sensor_msgs::msg::LaserScan::ConstSharedPtr& msg){
    std::cout<<"3\n";
    if(is_first_frame_){
        is_first_frame_ = false;
        ComputeAngle(msg);
        Laser2PCL(msg);
    } else {
        last_cloud_ = current_cloud_;
        Laser2PCL(msg);
    }
    std::cout<<"out\n";
    sensor_msgs::msg::PointCloud2 out_msg;
    pcl::toROSMsg(*current_cloud_,out_msg);
    cloud_pub_->publish(out_msg);
}

void mainstream::ComputeAngle(const sensor_msgs::msg::LaserScan::ConstSharedPtr& msg){
    std::cout<<"4\n";
    num_total = msg->ranges.size();
    angle_cos.resize(num_total);
    angle_sin.resize(num_total);
    for(size_t i=0;i<num_total;++i){
        float angle = msg->angle_min + i * msg->angle_increment;
        angle_cos[i] = std::cos(angle);
        angle_sin[i] = std::sin(angle);
    }
    std::cout<<"5\n";
}

void mainstream::Laser2PCL(const sensor_msgs::msg::LaserScan::ConstSharedPtr& msg){
    std::cout<<"6\n";
    cloudT::Ptr cloud_msg = std::make_shared<cloudT>();
    cloud_msg->resize(num_total);
    std::cout<<"7\n";

    for(size_t i=0;i<num_total;++i){
        pointT& pt = cloud_msg->points[i];
        float range = msg->ranges[i];

        if(std::isnan(range)){
            num_nan++;
            continue;
        }

        if(range>msg->range_min && range<msg->range_max){
            pt.x = range * angle_cos[i];
            pt.y = range * angle_sin[i];
            pt.z = .0f;
        }
    }
    std::cout<<"8\n";
    cloud_msg->width = num_total-num_nan;
    cloud_msg->height = 1;
    cloud_msg->is_dense = true;
    
    pcl_conversions::toPCL(msg->header,cloud_msg->header);
    cloud_msg->header.frame_id = "lidar";
    std::cout<<"9\n";
    current_cloud_ = cloud_msg;
}