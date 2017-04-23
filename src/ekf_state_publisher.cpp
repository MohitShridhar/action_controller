#include <iostream>

#include <math.h>
#include <ros/ros.h>

#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/Imu.h>

#include <tum_ardrone/filter_state.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

#include <math.h>
#define PI 3.14159265359

nav_msgs::Odometry _odometry;
tum_ardrone::filter_state _drone_state;
sensor_msgs::Imu _imu_state;
float _ml_scale;

ros::Publisher _drone_state_pub;

void ml_scale_cb(const std_msgs::Float64::ConstPtr &msg)
{
    _ml_scale = 1.0f / msg->data;
}

double conv_yaw_to_pi_range(double yaw, double pitch, double roll)
{
    bool is_in_last_two_quadrants = fabs(pitch) > 90.0f ? true : false;

    if (yaw > 0.0f && is_in_last_two_quadrants)
    {
        return 90.0f + (90.0f - yaw);
    }
    else if (yaw < 0.0f && is_in_last_two_quadrants)
    {
        return -90.0f - (90.0f + yaw);
    }

    return yaw;
}


void imu_cb(const sensor_msgs::Imu::ConstPtr &msg)
{
  _imu_state = *msg;
}


// Autopilot Coordinate System: x-right, y-forward, z-up
// Gazebo & Rviz Coordinate System: x-forward, y-left, z-up
// LSD SLAM Coordinate System: x-right, y-down, z-forward
// ORB SLAM Coordinate System: x-right, y-down, z-foward
// MSF EKF Filter CS: x-forward, y-left, z-up
// Raw IMU CS: x-back, y-right, z-down

void odom_cb(const nav_msgs::Odometry::ConstPtr &msg)
{
    _drone_state.header = msg->header;

    // Position
    _drone_state.x = -msg->pose.pose.position.y;
    _drone_state.y = msg->pose.pose.position.x;
    _drone_state.z = msg->pose.pose.position.z;

    // Orientation
    tf::Quaternion q(-msg->pose.pose.orientation.y, 
                     msg->pose.pose.orientation.x, 
                     msg->pose.pose.orientation.z, 
                     msg->pose.pose.orientation.w);

    tf::Matrix3x3 m(q);
    double yaw, pitch, roll;
    m.getRPY(roll, pitch, yaw);

    // Autopilot expects -180 to 180 deg range
    _drone_state.yaw = -yaw * 180.0f/PI;
    _drone_state.pitch = pitch * 180.0f/PI;
    _drone_state.roll = roll * 180.0f/PI;


    _drone_state.yaw = conv_yaw_to_pi_range(_drone_state.yaw, _drone_state.pitch, _drone_state.roll);


    // Linear Velocities
    _drone_state.dx = -msg->twist.twist.linear.y;
    _drone_state.dy = msg->twist.twist.linear.x;
    _drone_state.dz = msg->twist.twist.linear.z;


    // Linear Acceleration (WARNING: Unsyncornized Raw IMU and EKF data)
    double yawRad = _drone_state.yaw * 2 * 3.141592f / 360.0f;

    // In drone coordinates
    tf::Vector3 acc_vec;
    acc_vec[0] =  _imu_state.linear_acceleration.y;
    acc_vec[1] = -_imu_state.linear_acceleration.x;
    acc_vec[2] = -_imu_state.linear_acceleration.z;

    // Convert to global coordinates
    _drone_state.ax = 0.0f;//  ax * cos(yawRad) + ay * sin(yawRad);
    _drone_state.ay = 0.0f;// -ax * sin(yawRad) + ay * cos(yawRad);
    _drone_state.az = 0.0f;//  az;

    // Angular Velocities
    _drone_state.dyaw = -msg->twist.twist.angular.z * 180/PI;

    // Scale (External control)
    _drone_state.scale = 1.0f; // TODO: Fixed scale only for Gazebo
    _drone_state.ptamState = tum_ardrone::filter_state::PTAM_BEST; // TODO: Write a wrapper for LSD SLAM states
    _drone_state.scaleAccuracy = 1.0;

    _drone_state.droneState = 4;
    _drone_state.batteryPercent = 100.0f;


    _drone_state_pub.publish(_drone_state);


    // Publish TF link

    static tf::TransformBroadcaster br;
    tf::Transform transform;

    // Scaled Pose
    transform.setOrigin( tf::Vector3(-msg->pose.pose.position.y, msg->pose.pose.position.x, msg->pose.pose.position.z));

    // Orientation Estimate
    // tf::Quaternion q(_slam_pose.pose.orientation.x, _slam_pose.pose.orientation.z, -_slam_pose.pose.orientation.y, _slam_pose.pose.orientation.w);
    transform.setRotation(q);

    // Publish Transform
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "odom", "base_link"));

}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "ekf_state_publisher");
  ros::NodeHandle n;

  _ml_scale = 1.0f;

  ros::Subscriber odometry_sub = n.subscribe("/msf_core/odometry", 100, odom_cb);
  ros::Subscriber ml_scale_sub = n.subscribe("/ml_scale_estimate", 100, ml_scale_cb);
  ros::Subscriber imu_sub = n.subscribe("/bebop/imu", 100, imu_cb);

  // Output: Drone Filter State publisher (based on TUM Ardrone Autopilot)
  _drone_state_pub = n.advertise<tum_ardrone::filter_state>("/ardrone/predictedPose", 100); // TODO: Add proper topic name

  // Publishing Rate: 150Hz (should suffice for 100Hz EKF pose data)
  ros::Rate loop_rate(150);

  while (ros::ok())
  {
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
