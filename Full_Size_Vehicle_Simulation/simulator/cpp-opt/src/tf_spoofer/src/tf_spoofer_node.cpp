#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <iostream>
#include <rover_control_msgs/RoverDebugStateStamped.h> 


class tf_spoofer{
  ros::Subscriber sub_run_sim;
  ros::NodeHandle& nh;
  tf::TransformBroadcaster broadcaster;
  ros::Rate r;
  double x;
  double y;

public:
  tf_spoofer(ros::NodeHandle& nh_): nh(nh_), r(100){
    sub_run_sim = nh.subscribe("/init_conditions", 1, &tf_spoofer::startLoop, this);
  }

  void startLoop(rover_control_msgs::RoverDebugStateStamped data){
    x = data.x;
    y = data.y;
    std::cout<<"Initial conditions recieved, TF_spoofer started"<<std::endl;

    while(nh.ok()){
      broadcaster.sendTransform(
        tf::StampedTransform(
          tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(x, y, 0.0)),
          ros::Time::now(),"/base_link", "/map"));
      r.sleep();
    }
  }

};


int main(int argc, char** argv){
  ros::init(argc, argv, "tf_spoofer");
  ros::NodeHandle nh;

  tf_spoofer spoofer(nh);
  ros::spin();
  
}
