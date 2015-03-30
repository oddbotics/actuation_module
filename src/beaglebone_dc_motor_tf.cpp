#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <string>

int main(int argc, char** argv){
  ros::init(argc, argv, "motor_tf_broadcaster");
  ros::NodeHandle node;

  tf::TransformBroadcaster br;
  tf::Transform transform;

  double x,y,z;
  double roll,pitch,yaw;
  std::string connector = ros::this_node::getNamespace();

  //read in a private node handle to determine frame
  ros::NodeHandle private_node_handle_("~");
  private_node_handle_.param<double>("x", x, 0.0);
  private_node_handle_.param<double>("y", y, 0.0);
  private_node_handle_.param<double>("z", z, 0.0);
  private_node_handle_.param<double>("roll", roll, 0.0);
  private_node_handle_.param<double>("pitch", pitch, 0.0);
  private_node_handle_.param<double>("yaw", yaw, 0.0);

  tf::Quaternion quat;
  quat.setRPY(roll, pitch, yaw);

  ros::Rate rate(10.0);
  while (node.ok()){
    transform.setOrigin( tf::Vector3(x, y, z) );
    transform.setRotation( quat );
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), connector, "wheel"));
    rate.sleep();
  }
  return 0;
};
