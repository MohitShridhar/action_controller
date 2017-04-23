#include <iostream>

#include <math.h>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/Float64.h>

#include <dynamic_reconfigure/DoubleParameter.h>
#include <dynamic_reconfigure/BoolParameter.h>
#include <dynamic_reconfigure/Reconfigure.h>
#include <dynamic_reconfigure/Config.h>

#define DEACTIVATE_SWITCH_PARAM_NAME "disactivate_primary_switch"

#define MSF_PARAMS_SRV "/msf_pose_sensor/pose_sensor/set_parameters"
#define MSF_INIT_FILTER_PARAM "core_init_filter"
#define MSF_INIT_SCALE_PARAM "pose_initial_scale"
#define MSF_VNOISE_PARAM "pose_noise_scale"

#define PS_HOME_BUTTON 16
#define PS_START_BUTTON 3

ros::Publisher mux_cmd_pub;
geometry_msgs::Twist _primary_cmd, _secondary_cmd, _output_cmd;
int _deactivate_primary_button;
bool _deactivate_primary_state;
bool _filter_init;
float _ml_scale;

void mux_cmd();

void primary_cmd_cb(const geometry_msgs::Twist::ConstPtr &msg)
{
    _primary_cmd = *msg;

    mux_cmd();
}

void secondary_cmd_cb(const geometry_msgs::Twist::ConstPtr &msg)
{
    _secondary_cmd = *msg;

    mux_cmd();
}

void joy_cb(const sensor_msgs::Joy::ConstPtr &msg)
{
  if ( msg->buttons[_deactivate_primary_button] && !_deactivate_primary_state )
    _deactivate_primary_state = true;

  _filter_init = msg->buttons[PS_START_BUTTON]; 

   ROS_DEBUG("Deactivate Primary State: %d", _deactivate_primary_state ? 1 : 0);
}

void scale_cb(const std_msgs::Float64::ConstPtr &msg)
{
  _ml_scale = msg->data;
}

void mux_cmd()
{
    // Emergency Takeover
    if (_deactivate_primary_state &&
        (fabs(_primary_cmd.linear.x) > 0.0f || fabs(_primary_cmd.linear.y) > 0.0f || fabs(_primary_cmd.linear.y) > 0.0f   ||
         fabs(_primary_cmd.angular.x) > 0.0f || fabs(_primary_cmd.angular.y) > 0.0f || fabs(_primary_cmd.angular.z) > 0.0f ) )
    {
        _deactivate_primary_state = false;
    }

    if ( !_deactivate_primary_state )
    {
        _output_cmd = _primary_cmd;
    }
    else
    {
        _output_cmd = _secondary_cmd;
    }

    mux_cmd_pub.publish(_output_cmd);
}

void filter_ctrl(ros::NodeHandle n)
{
  if (_filter_init)
  {
    dynamic_reconfigure::ReconfigureRequest srv_req;
    dynamic_reconfigure::ReconfigureResponse srv_resp;
    dynamic_reconfigure::DoubleParameter double_param;
    dynamic_reconfigure::BoolParameter bool_param;
    dynamic_reconfigure::Config conf;

    double_param.name = MSF_INIT_SCALE_PARAM;
    double_param.value = _ml_scale;
    conf.doubles.push_back(double_param);

    // double_param.name = MSF_VNOISE_PARAM;
    // double_param.value = _ml_scale;
    // conf.doubles.push_back(double_param);

    bool_param.name = MSF_INIT_FILTER_PARAM;
    bool_param.value = true;
    conf.bools.push_back(bool_param);

    srv_req.config = conf;

    ros::service::call(MSF_PARAMS_SRV, srv_req, srv_resp);
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "controller_priority_mux");
  ros::NodeHandle n;

  // Default Init Scale:
  _ml_scale = 1.0f;

  // Cmd Vel topics to be muxed for safe controlling
  ros::Subscriber primary_cmd_sub = n.subscribe("safety_cmd_vel", 1, primary_cmd_cb);
  ros::Subscriber secondary_cmd_sub = n.subscribe("auto_cmd_vel", 1, secondary_cmd_cb);
  ros::Subscriber ml_scale_sub = n.subscribe("/ml_scale_estimate", 1, scale_cb);

  // Deactivate Switch
  _deactivate_primary_button = PS_HOME_BUTTON;
  if (n.hasParam(DEACTIVATE_SWITCH_PARAM_NAME))
    n.getParam(DEACTIVATE_SWITCH_PARAM_NAME, _deactivate_primary_button);

  // Joystick state
  ros::Subscriber joy_sub = n.subscribe("/joy", 1, joy_cb);

  // Output of the muxed controllers
  mux_cmd_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1);

  // Publishing Rate
  ros::Rate loop_rate(100);

  while (ros::ok())
  {
    filter_ctrl(n);

    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
