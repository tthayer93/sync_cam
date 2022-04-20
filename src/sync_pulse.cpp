#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Bool.h>

bool pulse_toggle = false;

/*Receive a pulse toggle*/
void received_toggle_pulse(const std_msgs::Bool& in_bool){
	pulse_toggle = in_bool.data;
}

/*Execute program.*/
int main(int argc, char **argv){
	ros::init(argc, argv, "sync_cam_pulse");
	ros::NodeHandle nh;
	ros::NodeHandle node_private("~");
	//Create publisher for the pulse message.
	ros::Publisher pub_pulse = nh.advertise<std_msgs::Int32>("sync_cam/pulse", 1000);
	std_msgs::Int32 out_pulse;
	int count = 0;
	//Create listener to start/stop pulses.
	ros::Subscriber sub_toggle_pulse = nh.subscribe("sync_cam/toggle_pulse", 1000, &received_toggle_pulse);
	//Get parameters.
	float delay, rate;
	node_private.param<float>("delay", delay, 5.0);
	node_private.param<float>("rate", rate, 5.0);
	ros::Rate wait(1.0/delay);
	ros::Rate pulse(1.0/rate);
	wait.reset();
	wait.sleep();
	//Start publishing pulses.
	while(ros::ok()){
		ros::spinOnce();
		if(pulse_toggle == true){
			out_pulse.data = ++count;
			pub_pulse.publish(out_pulse);
			ROS_INFO("Pulse %d sent.", count);
		}
		else{
			ROS_INFO("Pulses are paused at count %d.", count);
		}
		pulse.sleep();
	}
}
