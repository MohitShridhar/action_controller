#include <iostream>

#include <math.h>
#include <ros/ros.h>

#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Float64.h>

#include <tum_ardrone/filter_state.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

#include <math.h>
#define PI 3.14159265359

nav_msgs::Odometry _odometry;
geometry_msgs::PoseStamped _slam_pose, _last_slam_pose;
tum_ardrone::filter_state _drone_state;
float _ml_scale;

ros::Publisher _drone_state_pub;

void ml_scale_cb(const std_msgs::Float64::ConstPtr &msg)
{
    _ml_scale = 1.0f / msg->data;
}


void odom_cb(const nav_msgs::Odometry::ConstPtr &msg)
{
    _odometry = *msg;
}

void slam_pose_cb(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    _last_slam_pose = _slam_pose;
    _slam_pose = *msg;
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

void publish_autopilot_state(void)
{
    // FAKE PUBLISHER. TODO: Cleanup @Mohit

    _drone_state.header = _slam_pose.header;

    // Autopilot Coordinate System: x-right, y-forward, z-up
    // Gazebo & Rviz Coordinate System: x-forward, y-left, z-up
    // LSD SLAM Coordinate System: x-right, y-downward, z-forward
    // ORB SLAM Coordinate System: x-right, y-down, z-foward
    // MSF EKF Filter CS: x-forward, y-left, z-up

    // Position

    // Gazebo Ground Truth:
//    _drone_state.x = -_odometry.pose.pose.position.y;
//    _drone_state.y = _odometry.pose.pose.position.x;
//    _drone_state.z = _odometry.pose.pose.position.z;

    // LSD Slam (Scaled Pose):
    _drone_state.x = _slam_pose.pose.position.x * _ml_scale;   // 4.0f;
    _drone_state.y = _slam_pose.pose.position.z * _ml_scale; // 4.0f;
    _drone_state.z = -_slam_pose.pose.position.y * _ml_scale;  // 4.0f;

    // ORB Slam (Scale Pose)
//    _drone_state.x = _slam_pose.pose.position.x * _ml_scale;   // 4.0f;
//    _drone_state.y = _slam_pose.pose.position.y * _ml_scale; // 4.0f;
//    _drone_state.z = _slam_pose.pose.position.z * _ml_scale;  // 4.0f;

    // Orientation
//    tf::Quaternion q(_odometry.pose.pose.orientation.x, _odometry.pose.pose.orientation.y, _odometry.pose.pose.orientation.z, _odometry.pose.pose.orientation.w);
    tf::Quaternion q(_slam_pose.pose.orientation.x, _slam_pose.pose.orientation.y, _slam_pose.pose.orientation.z, _slam_pose.pose.orientation.w);
    tf::Matrix3x3 m(q);
    double yaw, pitch, roll;
    m.getRPY(roll, pitch, yaw);

    // Autopilot expects -180 to 180 deg range
    _drone_state.yaw = pitch * 180/PI;
    _drone_state.pitch = yaw * 180/PI;
    _drone_state.roll = roll * 180/PI;

    // Autopilot expects -180 to 180 deg range
//    _drone_state.yaw = - yaw * 180/PI;
//    _drone_state.pitch = pitch * 180/PI;
//    _drone_state.roll = roll * 180/PI;


    _drone_state.yaw = conv_yaw_to_pi_range(_drone_state.yaw, _drone_state.pitch, _drone_state.roll);

//    _drone_state.yaw = -yaw * 180/PI;
//    _drone_state.pitch = pitch * 180/PI;
//    _drone_state.roll = roll * 180/PI;

   // Linear Velocities
    // _drone_state.dx = -_odometry.twist.twist.linear.y;
    // _drone_state.dy = _odometry.twist.twist.linear.x;
    // _drone_state.dz = _odometry.twist.twist.linear.z;
    double delta = (_slam_pose.header.stamp.nsec - _last_slam_pose.header.stamp.nsec) * 10e-9;

    _drone_state.dx = ((_slam_pose.pose.position.x - _last_slam_pose.pose.position.x) / delta) * _ml_scale; 
    _drone_state.dy = ((_slam_pose.pose.position.z - _last_slam_pose.pose.position.z) / delta) * _ml_scale; 
    _drone_state.dz = ((-_slam_pose.pose.position.y + _last_slam_pose.pose.position.y)/ delta) * _ml_scale;


// Prediction

    _drone_state.x += _drone_state.dx * 0.2f;   // 4.0f;
    _drone_state.y += _drone_state.dy * 0.2f; // 4.0f;
    _drone_state.z += _drone_state.dz * 0.2f;  // 4.0f;


    // Angular Velocities
    _drone_state.dyaw = -_odometry.twist.twist.angular.z * 180/PI;

    // Scale
    _drone_state.scale = _ml_scale; // TODO: Fixed scale only for Gazebo
    _drone_state.ptamState = tum_ardrone::filter_state::PTAM_BEST; // TODO: Write a wrapper for LSD SLAM states
    _drone_state.scaleAccuracy = 1.0;

    _drone_state.droneState = 4;
    _drone_state.batteryPercent = 98.0f;


    _drone_state_pub.publish(_drone_state);
}

void publish_tf(void)
{
    static tf::TransformBroadcaster br;
    tf::Transform transform;

    // Scaled Pose
    transform.setOrigin( tf::Vector3(_slam_pose.pose.position.x *_ml_scale, _slam_pose.pose.position.z*_ml_scale,  -_slam_pose.pose.position.y*_ml_scale) );

    // Orientation Estimate
    tf::Quaternion q(_slam_pose.pose.orientation.x, _slam_pose.pose.orientation.z, -_slam_pose.pose.orientation.y, _slam_pose.pose.orientation.w);
    transform.setRotation(q);

    // Publish Transform
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "odom", "base_link"));
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "drone_state_publisher");
  ros::NodeHandle n;

  _ml_scale = 1.0f;

  ros::Subscriber odometry_sub = n.subscribe("/odom", 100, odom_cb);
  ros::Subscriber slam_pose_sub = n.subscribe("/lsd_slam/pose", 100, slam_pose_cb);
  ros::Subscriber ml_scale_sub = n.subscribe("/ml_scale_estimate", 100, ml_scale_cb);

  // Initialize SLAM Pose Quaternion:
  _slam_pose.pose.orientation.x = 0.0f;
  _slam_pose.pose.orientation.y = 0.0f;
  _slam_pose.pose.orientation.z = 0.0f;
  _slam_pose.pose.orientation.w = 1.0f;

  // Output: Drone Filter State publisher (based on TUM Ardrone Autopilot)
  _drone_state_pub = n.advertise<tum_ardrone::filter_state>("/ardrone/predictedPose", 100); // TODO: Add proper topic name

  // Publishing Rate: 30Hz
  ros::Rate loop_rate(200);

  while (ros::ok())
  {
    if (_odometry.header.seq != 0 && _slam_pose.header.seq != 0 && _last_slam_pose.header.seq != 0)
    {
        publish_autopilot_state();
    }

    publish_tf();

    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
