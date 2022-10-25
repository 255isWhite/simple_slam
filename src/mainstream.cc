#include "mainstream.hh"

using namespace SSLAM;

mainstream::mainstream():Node("simple_slam"){
    laser_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>("laser",1,std::bind(&mainstream::LaserCallback,this,std::placeholders::_1));
    cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("cloud",1);
}

void mainstream::LaserCallback(const sensor_msgs::msg::LaserScan::ConstSharedPtr& msg){
    if(is_first_frame_){
        is_first_frame_ = false;
        ComputeAngle(msg);
        Laser2PCL(msg);
    } else {
        last_cloud_ = current_cloud_;
        Laser2PCL(msg);
        ICP();
    }
    sensor_msgs::msg::PointCloud2 out_msg;
    pcl::toROSMsg(*current_cloud_,out_msg);
    cloud_pub_->publish(out_msg);
}

void mainstream::ComputeAngle(const sensor_msgs::msg::LaserScan::ConstSharedPtr& msg){
    num_total = msg->ranges.size();
    angle_cos_.resize(num_total);
    angle_sin_.resize(num_total);
    for(size_t i=0;i<num_total;++i){
        float angle = msg->angle_min + i * msg->angle_increment;
        angle_cos_[i] = std::cos(angle);
        angle_sin_[i] = std::sin(angle);
    }
}

void mainstream::Laser2PCL(const sensor_msgs::msg::LaserScan::ConstSharedPtr& msg){
    cloudT::Ptr cloud_msg = std::make_shared<cloudT>();
    cloud_msg->resize(num_total);

    for(size_t i=0;i<num_total;++i){
        pointT& pt = cloud_msg->points[i];
        float range = msg->ranges[i];

        if(std::isnan(range)){
            num_nan++;
            continue;
        }

        if(range>msg->range_min && range<msg->range_max){
            pt.x = range * angle_cos_[i];
            pt.y = range * angle_sin_[i];
            pt.z = .0f;
        }
    }
    cloud_msg->width = num_total-num_nan;
    cloud_msg->height = 1;
    cloud_msg->is_dense = true;
    
    pcl_conversions::toPCL(msg->header,cloud_msg->header);
    cloud_msg->header.frame_id = "lidar";
    current_cloud_ = cloud_msg;
}

void mainstream::ICP(){
    cloudT icp_cloud;
    pcl::IterativeClosestPoint<pointT,pointT> icp;
    icp.setInputSource(current_cloud_);
    icp.setInputTarget(last_cloud_);

    icp.setMaxCorrespondenceDistance(1.0);
    icp.setMaximumIterations(50);
    icp.setTransformationEpsilon(1e-8);
    icp.setEuclideanFitnessEpsilon(1.f);

    icp.align(icp_cloud);
    Eigen::Matrix4f T = icp.getFinalTransformation();
    pose_ = T*pose_;
    std::cout<<"[ICP] has convrged: "<<icp.hasConverged()<<std::endl;
    std::cout<<"[ICP] score: "<<icp.getFitnessScore()<<std::endl;
    std::cout<<"[ICP] Matrix T: \n"<<T<<std::endl;
    std::cout<<"[ICP] Matrix pose: \n"<<pose_<<std::endl;
}