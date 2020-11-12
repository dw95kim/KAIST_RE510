#include <ros/ros.h>
#include <ros/publisher.h>
#include <ros/subscriber.h>

#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Transform.h>

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>


class TeleSlaveController
{
public:

	std::string robot_name_;

	ros::NodeHandlePtr node_;
	ros::NodeHandlePtr pnode_;

	ros::Publisher target_pose_pub_;

	ros::Subscriber master_command_sub_;

	geometry_msgs::TwistStamped target_xyzrpy_;
	geometry_msgs::PoseStamped target_pose_;

	tf::StampedTransform T_BaseToTip_;

    int teleoperation_mode_;

	bool is_slave_state_;
	bool is_initial_slave_ee_pose_get_;

	TeleSlaveController(){
        is_slave_state_ = false;
        is_initial_slave_ee_pose_get_ = false;
	}

	void MasterCommandCallback(const geometry_msgs::PoseStampedConstPtr &msg){

	    geometry_msgs::PoseStamped master_cmd = *msg;

        // Dt
        ros::Time t1;
        t1 = ros::Time::now();
        static ros::Time t2 = t1;
        double Dt = (t1-t2).toSec();

        // Filtering large Dt
        static double prev_Dt = Dt;
        if(Dt > 5.0*prev_Dt && prev_Dt > 0){
            Dt = 0.0;
        }
        prev_Dt = Dt;
        t2 = t1;


        // Target Pose Update after getting the initial pose of slave manipulator
        if(!is_initial_slave_ee_pose_get_) return;

        // Example of matrix calculation based on Eigen library
        Eigen::MatrixXd T(3,3); // matrix T(3x3)
        Eigen::MatrixXd T_T = T.transpose(); // transpose of T
        Eigen::MatrixXd T_inv = T.inverse(); // inverse of T
        double t01 = T(0,1); // value for the row:1; col:2 component of matrix T

        Eigen::MatrixXd A(3,3), B(3,3); // matrix A and B
        Eigen::MatrixXd AB = A*B; // possible to multiply matrices


        // Get Euler-angle from quaternion included in PoseStamped msg
        tf::Pose pose;
        tf::poseMsgToTF(master_cmd.pose, pose);
        double mst_rolling, mst_pitching, mst_yawing;
        pose.getBasis().getRPY(mst_rolling, mst_pitching, mst_yawing);


        tf::poseMsgToTF(target_pose_.pose,pose);
        double current_roll, current_pitch, current_yaw;
        pose.getBasis().getRPY(current_roll, current_pitch, current_yaw);


        // The value of 'teleoperation_mode_' varaible is defined by the 'teleoperation_mode' parameter in the 'teleoperation.launch' file
        // 1.Position to Position : publish the increments command
        if(teleoperation_mode_ == 1){

			float linear_p_gain = 5;
			float angular_p_gain = 5;
            // Implement your controller
			Eigen::AngleAxisf rollAngle(current_roll, Eigen::Vector3f::UnitX());
			Eigen::AngleAxisf pitchAngle(current_pitch, Eigen::Vector3f::UnitY());
			Eigen::AngleAxisf yawAngle(current_yaw, Eigen::Vector3f::UnitZ());

			Eigen::Quaternionf q_current = yawAngle * pitchAngle * rollAngle;
			Eigen::Matrix3f r_current = q_current.matrix();

			Eigen::MatrixXf P(3, 1);
			P << linear_p_gain * (master_cmd.pose.position.x),
				linear_p_gain * (master_cmd.pose.position.y),
				linear_p_gain * (master_cmd.pose.position.z);

			Eigen::MatrixXf Output_P(3, 1);
			Output_P = r_current * P;

			target_pose_.pose.position.x = target_pose_.pose.position.x + Output_P(0, 0);
			target_pose_.pose.position.y = target_pose_.pose.position.y + Output_P(1, 0);
			target_pose_.pose.position.z = target_pose_.pose.position.z + Output_P(2, 0);

			Eigen::AngleAxisf cmd_rollAngle(angular_p_gain * mst_rolling, r_current * Eigen::Vector3f::UnitX());
			Eigen::AngleAxisf cmd_pitchAngle(angular_p_gain * mst_pitching, r_current * Eigen::Vector3f::UnitY());
			Eigen::AngleAxisf cmd_yawAngle(angular_p_gain * mst_yawing, r_current * Eigen::Vector3f::UnitZ());

			Eigen::Quaternionf q_cmd = cmd_yawAngle * cmd_pitchAngle * cmd_rollAngle;
			Eigen::Matrix3f r_cmd = q_cmd.matrix();

			Eigen::Matrix3f Output_O;
			Output_O = r_cmd * r_current;

			Eigen::Quaternionf q(Output_O);

			// tf::Quaternion q;
            // q.setRPY(current_roll, current_pitch, current_yaw);
            target_pose_.pose.orientation.x = q.x();
            target_pose_.pose.orientation.y = q.y();
            target_pose_.pose.orientation.z = q.z();
            target_pose_.pose.orientation.w = q.w();

            // Update Desired End-effector Pose to the 'target_pose_' variable.
        }

        // 2.Position to Velocity : publish the position command
        else if(teleoperation_mode_ == 2){

            // Implement your controller
			float linear_p_gain = 0.3;
			float angular_p_gain = 0.3;

			Eigen::AngleAxisf rollAngle(current_roll, Eigen::Vector3f::UnitX());
			Eigen::AngleAxisf pitchAngle(current_pitch, Eigen::Vector3f::UnitY());
			Eigen::AngleAxisf yawAngle(current_yaw, Eigen::Vector3f::UnitZ());

			Eigen::Quaternionf q_current = yawAngle * pitchAngle * rollAngle;
			Eigen::Matrix3f r_current = q_current.matrix();

			Eigen::MatrixXf P(3, 1);
			P << linear_p_gain * (master_cmd.pose.position.x),
				linear_p_gain * (master_cmd.pose.position.y),
				linear_p_gain * (master_cmd.pose.position.z);

			Eigen::MatrixXf Output_P(3, 1);
			Output_P = r_current * P;

			target_pose_.pose.position.x = target_pose_.pose.position.x + Output_P(0, 0);
			target_pose_.pose.position.y = target_pose_.pose.position.y + Output_P(1, 0);
			target_pose_.pose.position.z = target_pose_.pose.position.z + Output_P(2, 0);
            // Update Desired End-effector Pose to the 'target_pose_' variable.

			Eigen::AngleAxisf cmd_rollAngle(angular_p_gain * mst_rolling, r_current * Eigen::Vector3f::UnitX());
			Eigen::AngleAxisf cmd_pitchAngle(angular_p_gain * mst_pitching, r_current * Eigen::Vector3f::UnitY());
			Eigen::AngleAxisf cmd_yawAngle(angular_p_gain * mst_yawing, r_current * Eigen::Vector3f::UnitZ());

			Eigen::Quaternionf q_cmd = cmd_yawAngle * cmd_pitchAngle * cmd_rollAngle;
			Eigen::Matrix3f r_cmd = q_cmd.matrix();

			Eigen::Matrix3f Output_O;
			Output_O = r_cmd * r_current;

			Eigen::Quaternionf q(Output_O);

			// tf::Quaternion q;
            // q.setRPY(current_roll, current_pitch, current_yaw);
            target_pose_.pose.orientation.x = q.x();
            target_pose_.pose.orientation.y = q.y();
            target_pose_.pose.orientation.z = q.z();
            target_pose_.pose.orientation.w = q.w();
        }

        // Publish Target End-effector pose or velocity
        target_pose_.header.stamp = msg->header.stamp;
        target_pose_.header.frame_id = robot_name_+"_link0";
        target_pose_pub_.publish(target_pose_);
	}

  // Load parameters etc
	int init()
	{
		node_ = ros::NodeHandlePtr(new ros::NodeHandle(""));
		pnode_ = ros::NodeHandlePtr(new ros::NodeHandle("~"));

		pnode_->param("robot_name", robot_name_, std::string("panda"));

        node_->param("teleoperation_mode", teleoperation_mode_, 1);

        target_pose_pub_ = node_->advertise<geometry_msgs::PoseStamped>("ee_target_pose",10);

		master_command_sub_ = node_->subscribe("master_command", 10, &TeleSlaveController::MasterCommandCallback, this);


		return 0;
	}


	// Publish data
	void publish()
	{
		tf::TransformListener listener_BaseToTip;
		// ros::Rate loop_rate(100);
		while (node_->ok()) {

			// Get Initial End-effector pose of slave manipulator
			if(!is_initial_slave_ee_pose_get_){
				try{
					listener_BaseToTip.lookupTransform(robot_name_+"_link0", robot_name_+"_link7",ros::Time(0), T_BaseToTip_);
					is_slave_state_ = true;
					// ROS_INFO("tf get success");
				}
				catch (tf::TransformException ex){
					ROS_ERROR("%s",ex.what());
					is_slave_state_ = false;
					ros::Duration(0.5).sleep();
				}
				if(is_slave_state_){
					geometry_msgs::TransformStamped transform;
					tf::transformStampedTFToMsg(T_BaseToTip_,transform);

					target_xyzrpy_.twist.linear.x = transform.transform.translation.x;
					target_xyzrpy_.twist.linear.y = transform.transform.translation.y;
					target_xyzrpy_.twist.linear.z = transform.transform.translation.z;

					tf::Pose tf_pose;
					tf::transformMsgToTF(transform.transform,tf_pose);					
					double current_roll,current_pitch, current_yaw;
			        tf_pose.getBasis().getRPY(current_roll, current_pitch, current_yaw);

			        target_xyzrpy_.twist.angular.x = current_roll;
			        target_xyzrpy_.twist.angular.y = current_pitch;
			        target_xyzrpy_.twist.angular.z = current_yaw;

					// Eigen::AngleAxisf rollAngle(current_roll, Eigen::Vector3f::UnitX());
					// Eigen::AngleAxisf pitchAngle(current_pitch, Eigen::Vector3f::UnitY());
					// Eigen::AngleAxisf yawAngle(current_yaw, Eigen::Vector3f::UnitZ());

					// Eigen::Quaternionf q_current = yawAngle * pitchAngle * rollAngle;

					// Eigen::Matrix3f r_current = q_current.matrix();
					// Eigen::Matrix3f r_cur_inv = r_current.inverse();

					double x_diff = 0.0;
					double y_diff = 0.0;
					double z_diff = 0.0;

					// Eigen::Matrix3f R;
					// double w = transform.transform.rotation.w;
					// double x = transform.transform.rotation.x;
					// double y = transform.transform.rotation.y;
					// double z = transform.transform.rotation.z;
					// R = Eigen::Quaternionf(w, x, y, z);
					
					// Eigen::Matrix3f New_T = r_cur_inv*R;
					// Eigen::Quaternionf q;
					// q = New_T;

			        target_pose_.pose.position.x = transform.transform.translation.x + x_diff;
			        target_pose_.pose.position.y = transform.transform.translation.y + y_diff;
			        target_pose_.pose.position.z = transform.transform.translation.z + z_diff;
					target_pose_.pose.orientation = transform.transform.rotation;

					// target_pose_.pose.orientation.w = q.w();
					// target_pose_.pose.orientation.x = q.x();
					// target_pose_.pose.orientation.y = q.y();
					// target_pose_.pose.orientation.z = q.z();

					is_initial_slave_ee_pose_get_ = true;
				}
			}

			ros::spinOnce();
			// loop_rate.sleep();
		}
	}
};


int main(int argc, char **argv)
{
	ros::init(argc, argv, "tele_slave_controller_node");

	TeleSlaveController slave;
	if(slave.init())
	{
		ROS_FATAL("tele_slave_controller_node initialization failed");
		return -1;
	}

	slave.publish();

	return 0;
}


