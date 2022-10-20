#include "common.hh"
#include "mainstream.hh"

int main(int argc, char** argv){
    rclcpp::init(argc,argv);
    std::cout<<"0\n";
    auto sslam=std::make_shared<SSLAM::mainstream>();
    rclcpp::spin(sslam);

    rclcpp::shutdown();
    return EXIT_SUCCESS;
}
