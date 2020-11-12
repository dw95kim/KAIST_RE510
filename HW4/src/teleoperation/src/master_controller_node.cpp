#include <ros/ros.h>
#include <ros/publisher.h>
#include <ros/subscriber.h>

#include <tf/tf.h>
#include <geometry_msgs/PoseStamped.h>

#include <virtual_master_device/master_dev_state.h>


class TeleMasterController
{
public:

	ros::NodeHandlePtr node_;
	ros::NodeHandlePtr pnode_;


    ros::Publisher master_cmd_pub_;


	ros::Subscriber master_device_state_sub_;

    int teleoperation_mode_;


	void MasterDevStateCallback(const virtual_master_device::master_dev_stateConstPtr &msg){

        // Dt
        ros::Time t1;
        t1 = ros::Time::now();
        static ros::Time t2 = t1;
        double Dt = (t1-t2).toSec();
        t2 = t1;
        if(Dt == 0) return;
        
        
        // Master Position
        double m_px = msg->pos.twist.linear.x;
        double m_py = msg->pos.twist.linear.y;
        double m_pz = msg->pos.twist.linear.z;

        // Master Orientation
        double m_ox = msg->pos.twist.angular.x;
        double m_oy = msg->pos.twist.angular.y;
        double m_oz = msg->pos.twist.angular.z;

        // Indexing trigger
        bool clutch = msg->button;

        static virtual_master_device::master_dev_state old_msg = *msg;

        geometry_msgs::PoseStamped master_command;


        // The value of 'teleoperation_mode_' varaible is defined by the 'teleoperation_mode' parameter in the 'teleoperation.launch' file
        // 1.Position to Position : publish the increments command
        if(teleoperation_mode_ == 1){

            if(clutch)
            {
                // Make your increments command
                double x_command = (m_px - old_msg.pos.twist.linear.x);
                double y_command = (m_py - old_msg.pos.twist.linear.y);
                double z_command = (m_pz - old_msg.pos.twist.linear.z);

                master_command.pose.position.x = x_command; // replace '0.0' to your command value
                master_command.pose.position.y = y_command; // replace '0.0' to your command value
                master_command.pose.position.z = z_command; // replace '0.0' to your command value

                double roll_command = (m_ox - old_msg.pos.twist.angular.x);
                double pitch_command = (m_oy - old_msg.pos.twist.angular.y);
                double yaw_command = (m_oz - old_msg.pos.twist.angular.z);

                double roll,pitch,yaw;

                roll = roll_command;  // replace '0.0' to your command value
                pitch = pitch_command;  // replace '0.0' to your command value
                yaw = yaw_command;  // replace '0.0' to your command value

                // ROS_INFO("roll : %.2f", roll);
                // ROS_INFO("pitch : %.2f", pitch);
                // ROS_INFO("yaw : %.2f", yaw);

                // Euler angle to Quaternion
                tf::Quaternion q;
                q.setRPY(roll,pitch,yaw);
                master_command.pose.orientation.x = q.x();
                master_command.pose.orientation.y = q.y();
                master_command.pose.orientation.z = q.z();
                master_command.pose.orientation.w = q.w();
            }
            else
            {
                master_command.pose.position.x = 0;
                master_command.pose.position.y = 0;
                master_command.pose.position.z = 0;

                tf::Quaternion q;
                q.setRPY(0,0,0);
                master_command.pose.orientation.x = q.x();
                master_command.pose.orientation.y = q.y();
                master_command.pose.orientation.z = q.z();
                master_command.pose.orientation.w = q.w();
            }
        }

        // 2.Position to Velocity : publish the position command
        else if(teleoperation_mode_ == 2){
            // Same as 'teleoperation_mode_ == 1' case.
            if(clutch)
            {
                master_command.pose.position.x = m_px;
                master_command.pose.position.y = m_py;
                master_command.pose.position.z = m_pz;

                tf::Quaternion q;
                q.setRPY(m_ox,m_oy,m_oz);
                master_command.pose.orientation.x = q.x();
                master_command.pose.orientation.y = q.y();
                master_command.pose.orientation.z = q.z();
                master_command.pose.orientation.w = q.w();
            }
            else
            {
                master_command.pose.position.x = 0;
                master_command.pose.position.y = 0;
                master_command.pose.position.z = 0;

                tf::Quaternion q;
                q.setRPY(0,0,0);
                master_command.pose.orientation.x = q.x();
                master_command.pose.orientation.y = q.y();
                master_command.pose.orientation.z = q.z();
                master_command.pose.orientation.w = q.w();
            }
        }


        // Publish Master Command
        master_command.header = msg->pos.header;
        master_cmd_pub_.publish(master_command);
    
        // Update old_msg
        old_msg = *msg;
	}

	// Smoothing the noisy signal. previous: x(t-1), current: x(t).
	double lowpass_filter(double previous, double current){
	    double k = 0.95; // can adjust the gain
        current = k*current + (1.0-k)*previous;
        return current;
	}


  // Load parameters etc
	int init()
	{
		node_ = ros::NodeHandlePtr(new ros::NodeHandle(""));
		pnode_ = ros::NodeHandlePtr(new ros::NodeHandle("~"));

        node_->param("teleoperation_mode", teleoperation_mode_, 1);

        master_device_state_sub_ = node_->subscribe("master_device/state",10,&TeleMasterController::MasterDevStateCallback,this);

        master_cmd_pub_ = node_->advertise<geometry_msgs::PoseStamped>("master_command",10);

		return 0;
	}

	// Publish data
	void publish()
	{
		// ros::Rate loop_rate(100);
		while (node_->ok()) {
		  ros::spinOnce();
		  // loop_rate.sleep();
		}
	}
};


int main(int argc, char **argv)
{
	ros::init(argc, argv, "tele_master_controller_node");

	TeleMasterController master;
	if(master.init())
	{
		ROS_FATAL("tele_master_controller_node initialization failed");
		return -1;
	}

	master.publish();

	return 0;
}


