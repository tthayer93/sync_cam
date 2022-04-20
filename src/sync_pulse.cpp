#include <ros/ros.h>
#include <std_msgs/Int32.h>

/*Execute program.*/
int main(int argc, char **argv){
	ros::init(argc, argv, "sync_pulse");
	ros::NodeHandle nh;
	ros::NodeHandle node_private("~");
	//Create publisher for the pulse message.
	ros::Publisher pub_pulse = nh.advertise<std_msgs::Int32>("sync/pulse", 1000);
	std_msgs::Int32 out_pulse;
	int count = 0;
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
		out_pulse.data = ++count;
		pub_pulse.publish(out_pulse);
		ROS_INFO("Pulse %d sent.", count); 
		pulse.sleep();
	}
}
