#include <ros/ros.h>
#include <actionlib/server/action_server.h>
#include <pthread.h>

#include <iostream>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <action_controller/MultiDofFollowJointTrajectoryAction.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/String.h>
#include <tf/transform_datatypes.h>

#define PI 3.14159265359
#define MAX_LINEAR_SPEED 2.0

#define WAYPOINT_MAX_DELTA_TRANS 0.3 // meters
#define WAYPOINT_MAX_DELTA_ROT 1.0
#define WAYPOINT_MAX_TRIES 2000

class Controller{
private:
	typedef actionlib::ActionServer<action_controller::MultiDofFollowJointTrajectoryAction> ActionServer;
	typedef ActionServer::GoalHandle GoalHandle;

    nav_msgs::Odometry _odometry;

public:
	Controller(ros::NodeHandle &n) :
		node_(n),
        action_server_(node_, "multi_dof_joint_trajectory_action",
				boost::bind(&Controller::goalCB, this, _1),
				boost::bind(&Controller::cancelCB, this, _1),
                false),
				has_active_goal_(false)
    {
            creato=0;
            empty.linear.x=0;
            empty.linear.y=0;
            empty.linear.z=0;
            empty.angular.z=0;
            empty.angular.y=0;
            empty.angular.x=0;
//            pub_topic = node_.advertise<geometry_msgs::Twist>("cmd_vel", 1);
            autopilot_cmd_pub = node_.advertise<std_msgs::String>("/tum_ardrone/com", 1000);
            action_server_.start();
            ROS_INFO_STREAM("Node ready!");
    }

    void odom_cb(const nav_msgs::Odometry::ConstPtr &msg)
    {
        _odometry = *msg;
    }

private:
	ros::NodeHandle node_;
	ActionServer action_server_;
	ros::Publisher pub_topic;
      ros::Publisher autopilot_cmd_pub;
	geometry_msgs::Twist empty;
	geometry_msgs::Transform_<std::allocator<void> > lastPosition;
    ros::Duration last_time_from_start;
	geometry_msgs::Twist cmd;
    std::ostringstream cmd_str;
	pthread_t trajectoryExecutor;
	int creato;

	bool has_active_goal_;
	GoalHandle active_goal_;
	trajectory_msgs::MultiDOFJointTrajectory_<std::allocator<void> > toExecute;

	void cancelCB(GoalHandle gh){
		if (active_goal_ == gh)
		{
			// Stops the controller.
			if(creato){
				ROS_INFO_STREAM("Stop thread");
				pthread_cancel(trajectoryExecutor);
				creato=0;
			}
			pub_topic.publish(empty);

			// Marks the current goal as canceled.
			active_goal_.setCanceled();
			has_active_goal_ = false;
		}
	}

	void goalCB(GoalHandle gh){
		if (has_active_goal_)
		{
			// Stops the controller.
			if(creato){
				pthread_cancel(trajectoryExecutor);
				creato=0;
			}
			pub_topic.publish(empty);

			// Marks the current goal as canceled.
			active_goal_.setCanceled();
			has_active_goal_ = false;
		}

		gh.setAccepted();
		active_goal_ = gh;
		has_active_goal_ = true;
		toExecute = gh.getGoal()->trajectory;

		//controllore solo per il giunto virtuale Base
		if(pthread_create(&trajectoryExecutor, NULL, threadWrapper, this)==0){
			creato=1;
			ROS_INFO_STREAM("Thread for trajectory execution created");
		} else {
			ROS_INFO_STREAM("Thread creation failed!");
		}

	}

	static void* threadWrapper(void* arg) {
		Controller * mySelf=(Controller*)arg;
		mySelf->executeTrajectory();
		return NULL;
	}


    void stopAndSleep()
    {
        cmd.linear.x=cmd.linear.y=cmd.linear.z=cmd.angular.x=cmd.angular.y=cmd.angular.z=0;
        pub_topic.publish(cmd);
        ros::Duration sleepTime(1.0);
        sleepTime.sleep();
    }

    // Assuming origins are aligned
    bool checkWaypoint(geometry_msgs::Transform_<std::allocator<void> > waypoint)
    {
        ROS_INFO("Diff x:%f", fabs(waypoint.translation.x-_odometry.pose.pose.position.x));
        ROS_INFO("Diff y:%f", fabs(waypoint.translation.y-_odometry.pose.pose.position.y));
        ROS_INFO("Diff z:%f", fabs(waypoint.translation.z-_odometry.pose.pose.position.z));

        if (fabs(waypoint.translation.x-_odometry.pose.pose.position.x) > WAYPOINT_MAX_DELTA_TRANS ||
            fabs(waypoint.translation.y-_odometry.pose.pose.position.y) > WAYPOINT_MAX_DELTA_TRANS ||
            fabs(waypoint.translation.z-_odometry.pose.pose.position.z) > WAYPOINT_MAX_DELTA_TRANS)
            return false;

        //TODO: Implement rotation check

        return true;
    }

    void initialize_autopilot(void)
    {
        // Start autopilot
        cmd_str.str("");
        cmd_str << "c start";
        autopilot_cmd_pub.publish(cmd_str.str());

        // Set Waypoint Reach Delta
        cmd_str.str("");
        cmd_str << "c setStayWithinDist 0.4";
        autopilot_cmd_pub.publish(cmd_str.str());

        // Set Waypoint Set Time
        cmd_str.str("");
        cmd_str << "c setStayTime 0.2";
        autopilot_cmd_pub.publish(cmd_str.str());

        // Clear all previous commands
        cmd_str.str("");
        cmd_str << "c clearCommands";
        autopilot_cmd_pub.publish(cmd_str.str());
    }

    void goto_pose(geometry_msgs::Transform_<std::allocator<void> > waypoint)
    {
        tf::Quaternion q(waypoint.rotation.x, waypoint.rotation.y, waypoint.rotation.z, waypoint.rotation.w);
        tf::Matrix3x3 m(q);
        double yaw, pitch, roll;
        m.getRPY(roll, pitch, yaw);

        double yaw_rad = yaw * 180/PI;
        double yaw_360_deg = yaw_rad < 0 ? (PI + yaw_rad) + PI : yaw_rad;

        cmd_str.str("");
        cmd_str << "c goto " << waypoint.translation.x << " " << waypoint.translation.y << " " << waypoint.translation.z << " " << -yaw_360_deg;
//        cmd_str << "c goto " << -waypoint.translation.x << " " << waypoint.translation.z << " " << -waypoint.translation.y << " " << 0.0f;
        autopilot_cmd_pub.publish(cmd_str.str());
    }

    void holonomic()
    {
        initialize_autopilot();

        if(toExecute.joint_names[0]=="virtual_joint" && toExecute.points.size()>0){

            for(int k=0; k<toExecute.points.size(); k++){
                //ricavo cmd da effettuare
                geometry_msgs::Transform_<std::allocator<void> > waypoint=toExecute.points[k].transforms[0];
//                ros::Duration time_from_start = toExecute.points[k].time_from_start;

                goto_pose(waypoint);
            }

        }
        active_goal_.setSucceeded();
        has_active_goal_=false;
        creato=0;
    }

    // PID Control
    bool publishTranslationRotationCommand(geometry_msgs::Transform_<std::allocator<void> > waypoint, bool anyway, ros::Duration time_from_start)
    {
        ros::Duration deltaTime = time_from_start - last_time_from_start;
        float deltaTimeSec = deltaTime.toSec();

        cmd.linear.x = ( 0.8f*(waypoint.translation.x-_odometry.pose.pose.position.x) + 0.0f*_odometry.twist.twist.linear.x ) / deltaTimeSec;
        cmd.linear.y = ( 0.8f*(waypoint.translation.y-_odometry.pose.pose.position.y) + 0.0f*_odometry.twist.twist.linear.y ) / deltaTimeSec;
        cmd.linear.z = ( 0.8f*(waypoint.translation.z-_odometry.pose.pose.position.z) + 0.0f*_odometry.twist.twist.linear.z ) / deltaTimeSec;

        // TODO: Implement rotation command
        cmd.angular.x=cmd.angular.y=0;
        cmd.angular.z = 0;// (waypoint.rotation.z-lastPosition.rotation.z) / deltaTimeSec;

        if (anyway || (fabs(cmd.linear.x)<MAX_LINEAR_SPEED && fabs(cmd.linear.y)<MAX_LINEAR_SPEED && fabs(cmd.linear.z)<MAX_LINEAR_SPEED))
        {
            printPositionInfo();
            printCmdInfo();
            pub_topic.publish(cmd);

            deltaTime.sleep();
            return true;
        }

        return false;
    }

    void executeTrajectory(){
        holonomic();
	}

	void printPositionInfo(){
		ROS_INFO_STREAM("Start Position: ["<<lastPosition.translation.x<<
				", "<<lastPosition.translation.y<<
				", "<<lastPosition.translation.z<<"] "<<
				"[ "<<lastPosition.rotation.x<<
				", "<<lastPosition.rotation.y<<
				", "<<lastPosition.rotation.z<<" ]");
	}

	void printCmdInfo(){
		ROS_INFO_STREAM("cmd to execute: "<<"x:"<<cmd.linear.x
				<<" y: " << cmd.linear.y
				<<" z: " << cmd.linear.z
				<<" rX: " << cmd.angular.x
				<<" rY: " << cmd.angular.y
				<<" rZ: " << cmd.angular.z);
	}

};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "my_controller_node");
    ros::NodeHandle node;
    Controller control(node);

//    ros::Subscriber odometry_sub = node.subscribe("/odom", 1000, &Controller::odom_cb, &control);

    ros::spin();

    return 0;
}
