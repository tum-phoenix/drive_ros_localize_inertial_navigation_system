#include "ros/ros.h"
#include "tf/tf.h"
#include "tf/transform_broadcaster.h"
#include "sensor_msgs/Imu.h"
#include "nav_msgs/Odometry.h"
#include "drive_ros_localize_inertial_navigation_system/moving_average.h"


static nav_msgs::Odometry odom;
static ros::Time last_time;
ros::Publisher pub;
tf::TransformBroadcaster* br;
bool broadcast_tf;                       // whether to broadcast tf

MovingAverage ax_avg;
MovingAverage ay_avg;
MovingAverage omega_avg;

double yaw = 0;

void imuCallback(const sensor_msgs::Imu& msg)
{
  if(ros::Time(0,0) == last_time)
  {
    tf::Quaternion q_new;
    q_new.setRPY(0, 0, 0);
    tf::quaternionTFToMsg(q_new, odom.pose.pose.orientation);

    last_time = msg.header.stamp;
    return;
  }

  double omega = omega_avg.addAndGetCrrtAvg(msg.angular_velocity.z);
  ros::Duration dt = (msg.header.stamp - last_time);

  // integrate omega
  yaw += omega * dt.toSec();

  // integrate acceleration
  double ax = ax_avg.addAndGetCrrtAvg(msg.linear_acceleration.x);
  double ay = ay_avg.addAndGetCrrtAvg(msg.linear_acceleration.y);

  // transform input
  double ax_trans = ax * std::cos(static_cast<float>(yaw)) +
                    ay * std::sin(static_cast<float>(yaw));

  double ay_trans = ax * std::sin(static_cast<float>(yaw)) +
                    ay * std::cos(static_cast<float>(yaw));

  odom.twist.twist.linear.x += ax_trans * dt.toSec();
  odom.twist.twist.linear.y += ay_trans * dt.toSec();
  odom.twist.twist.linear.z = 0;   // we have planar assumption

  odom.pose.pose.position.x += odom.twist.twist.linear.x * dt.toSec() + ax_trans * 0.5 * pow(dt.toSec(), 2);
  odom.pose.pose.position.y += odom.twist.twist.linear.y * dt.toSec() + ay_trans * 0.5 * pow(dt.toSec(), 2);
  odom.pose.pose.position.z  = 0;   // we have planar assumption

  odom.twist.twist.angular.x = 0;  // we have planar assumption
  odom.twist.twist.angular.y = 0;  // we have planar assumption
  odom.twist.twist.angular.z = omega;

  odom.header.stamp = msg.header.stamp;

  // write Euler angles to new trafo
  tf::Quaternion q_new;
  q_new.setRPY(0, 0, yaw); // we have planar assumption
  tf::quaternionTFToMsg(q_new, odom.pose.pose.orientation);

  // publish odometry message
  pub.publish(odom);

  // broadcast tf message
  if(broadcast_tf)
  {
    geometry_msgs::TransformStamped tf_msg;
    tf::Transform trafo;
    tf_msg.header.stamp            = msg.header.stamp;
    tf_msg.child_frame_id          = odom.child_frame_id;
    tf_msg.header.frame_id         = odom.header.frame_id;
    tf_msg.transform.translation.x = odom.pose.pose.position.x;
    tf_msg.transform.translation.y = odom.pose.pose.position.y;
    tf_msg.transform.translation.z = odom.pose.pose.position.z;
    tf_msg.transform.rotation.x    = odom.pose.pose.orientation.x;
    tf_msg.transform.rotation.y    = odom.pose.pose.orientation.y;
    tf_msg.transform.rotation.z    = odom.pose.pose.orientation.z;
    tf_msg.transform.rotation.w    = odom.pose.pose.orientation.w;
    br->sendTransform(tf_msg);
  }

  // save last message
  last_time = msg.header.stamp;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "node");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");


  // set parameter
  odom.header.frame_id = pnh.param<std::string>("static_frame", "/odom");
  odom.child_frame_id =  pnh.param<std::string>("moving_frame", "/ego_rear_axis_middle_ground");

  ax_avg.setFilterLength(pnh.param<int>("moving_avg_a_size", 10));
  ay_avg.setFilterLength(pnh.param<int>("moving_avg_a_size", 10));
  ROS_INFO_STREAM("Moving avg a size: " << ax_avg.getFilterLength());

  omega_avg.setFilterLength(pnh.param<int>("moving_avg_omega_size", 10));
  ROS_INFO_STREAM("Moving avg omega size: " << omega_avg.getFilterLength());

  broadcast_tf = pnh.param<bool>("broadcast_tf", true);

  // setup subscriber and publisher
  if(broadcast_tf)
  {
    br = new tf::TransformBroadcaster;
  }


  pub = pnh.advertise<nav_msgs::Odometry>("odo_out", 1000);
  ros::Subscriber sub = pnh.subscribe("imu_in", 1000, imuCallback);

  ros::spin();

  delete br;

  return 0;
}
