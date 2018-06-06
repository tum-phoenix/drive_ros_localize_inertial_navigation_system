#include "ros/ros.h"
#include "tf/tf.h"
#include "cmath"
#include "eigen3/Eigen/Dense"
#include "tf/transform_broadcaster.h"
#include "sensor_msgs/Imu.h"
#include "nav_msgs/Odometry.h"
#include "drive_ros_localize_inertial_navigation_system/moving_average.h"
#include "drive_ros_localize_inertial_navigation_system/cov_elements.h"


// noise vector indices
static constexpr size_t AX     = 0;
static constexpr size_t AY     = 1;
static constexpr size_t OMEGAZ = 2;

// state vector indices
static constexpr size_t X     = 0;
static constexpr size_t Y     = 1;
static constexpr size_t VX    = 2;
static constexpr size_t VY    = 3;
static constexpr size_t YAW   = 4;

// ros stuff
ros::Publisher pub;             // publisher
tf::TransformBroadcaster* br;   // tf broadcaster

// moving average filters
MovingAverage ax_avg;
MovingAverage ay_avg;
MovingAverage omega_avg;

// local variables
static nav_msgs::Odometry odom; // output odometry message
static ros::Time last_time;     // time of last imu message
static double yaw = 0;          // current yaw angle
static bool broadcast_tf;       // whether to broadcast tf
static Eigen::Matrix<double, 5, 5> Sigma_p;     // covariance matrix of previous step

// calculate Jacobian of noise vector
void calculateW(Eigen::Matrix<double, 5, 3>& W, const double T, const double oz,
                const double psi, const double ax, const double ay)
{
  /*
  Matlab generated code (check the docs) for symbolic expression: Ws
  */
  auto t2 = T*T;
  auto t3 = T*oz;
  auto t4 = t3+psi;
  auto t5 = std::cos(static_cast<float>(t4));
  auto t6 = std::sin(static_cast<float>(t4));
  auto t7 = t2*t6*(1.0/2.0);
  auto t8 = t2*t5*(1.0/2.0);
  auto t9 = ax*t6;
  auto t10 = t9-ay*t5;
  auto t11 = T*t6;
  auto t12 = T*t5;
  auto t13 = ax*t5;
  auto t14 = t13-ay*t6;
  W(0,0) = t8;
  W(0,1) = t7;
  W(0,2) = T*t2*t10*(-1.0/2.0);
  W(1,0) = t7;
  W(1,1) = t8;
  W(1,2) = T*t2*t14*(1.0/2.0);
  W(2,0) = t12;
  W(2,1) = t11;
  W(2,2) = -t2*t10;
  W(3,0) = t11;
  W(3,1) = t12;
  W(3,2) = t2*t14;
  W(4,2) = T;
}

// calculate Jacobian of state vector
void calculateG(Eigen::Matrix<double, 5, 5>& G, const double T, const double oz,
                const double psi, const double ax, const double ay)
{
  /*
  Matlab generated code (check the docs) for symbolic expression: Gs
  */
  auto t2 = T*oz;
  auto t3 = t2+psi;
  auto t4 = T*T;
  auto t5 = std::cos(static_cast<float>(t3));
  auto t6 = std::sin(static_cast<float>(t3));
  auto t7 = ax*t6;
  auto t8 = t7-ay*t5;
  auto t9 = ax*t5;
  auto t10 = t9-ay*t6;
  G(0,0) = 1.0;
  G(0,2) = T;
  G(0,4) = t4*t8*(-1.0/2.0);
  G(1,1) = 1.0;
  G(1,3) = T;
  G(1,4) = t4*t10*(1.0/2.0);
  G(2,2) = 1.0;
  G(2,4) = -T*t8;
  G(3,3) = 1.0;
  G(3,4) = T*t10;
  G(4,4) = 1.0;

}

// new imu message
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

  // jacobians
  Eigen::Matrix<double, 5, 5> G;
  Eigen::Matrix<double, 5, 3> W;
  G.setZero();
  W.setZero();

  calculateG(G, dt.toSec(), omega, yaw, ax, ay);
  calculateW(W, dt.toSec(), omega, yaw, ax, ay);

  // system noise
  Eigen::Matrix<double, 3, 3> R;
  R.setZero();
  R(AX,     AX)     = msg.linear_acceleration_covariance[CovElem::lin::linX_linX];
  R(AY,     AY)     = msg.linear_acceleration_covariance[CovElem::lin::linY_linY];
  R(OMEGAZ, OMEGAZ) = msg.angular_velocity_covariance[CovElem::ang::angZ_angZ];

  // error propagation law
  Sigma_p = G * Sigma_p * G.transpose()
          + W * R       * W.transpose();

  // write error to output message
  odom.pose.covariance[CovElem::lin_ang::linX_linX] = Sigma_p(X, X);
  odom.pose.covariance[CovElem::lin_ang::linX_linY] = Sigma_p(X, Y);
  odom.pose.covariance[CovElem::lin_ang::linX_angZ] = Sigma_p(X, YAW);
  odom.pose.covariance[CovElem::lin_ang::linY_linX] = Sigma_p(Y, X);
  odom.pose.covariance[CovElem::lin_ang::linY_linY] = Sigma_p(Y, Y);
  odom.pose.covariance[CovElem::lin_ang::linY_angZ] = Sigma_p(Y, YAW);
  odom.pose.covariance[CovElem::lin_ang::angZ_linX] = Sigma_p(YAW, X);
  odom.pose.covariance[CovElem::lin_ang::angZ_linY] = Sigma_p(YAW, Y);
  odom.pose.covariance[CovElem::lin_ang::angZ_angZ] = Sigma_p(YAW, YAW);

  odom.twist.covariance[CovElem::lin_ang::linX_linX] = Sigma_p(VX, VX);
  odom.twist.covariance[CovElem::lin_ang::linX_linY] = Sigma_p(VX, VY);
  odom.twist.covariance[CovElem::lin_ang::linY_angX] = Sigma_p(VY, VX);
  odom.twist.covariance[CovElem::lin_ang::linY_linY] = Sigma_p(VY, VY);

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

  // set initial covariances to 0
  for(int i=0; i<25; i++)
  {
    Sigma_p(i/5, i%5) = 0;
  }

  // set all output covariances to -1 (not valid)
  // valid values will be overwritten on arrival of first message
  for(int i=0; i<36; i++)
  {
    odom.pose.covariance[i] = -1;
    odom.twist.covariance[i] = -1;
  }


  pub = pnh.advertise<nav_msgs::Odometry>("odo_out", 1000);
  ros::Subscriber sub = pnh.subscribe("imu_in", 1000, imuCallback);

  ros::spin();

  delete br;

  return 0;
}
