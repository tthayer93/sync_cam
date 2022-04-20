#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include <boost/filesystem.hpp>
#include <string>
#include <iostream>
#include <fstream>
#include <ctime>
namespace fs = boost::filesystem;

std::string suffix;
fs::path save_path;
bool fix_filled = false;
bool image_filled = false;
sensor_msgs::NavSatFix current_fix;
cv_bridge::CvImageConstPtr current_image_ptr;
std::ofstream outfile;

/*Get current date and time.*/
std::string get_date_time_str(){
	time_t now = time(0);
	struct tm *current;
	current = localtime(&now);
	std::string str = "                   ";
	sprintf(&str[0], "%04d-%02d-%02d_%02d:%02d:%02d", current->tm_year, current->tm_mon, current->tm_mday, current->tm_hour, current->tm_min, current->tm_sec);
	return str;
}

/*Receive an image message.*/
void received_image(const sensor_msgs::ImageConstPtr& in_image){
	try{
		current_image_ptr = cv_bridge::toCvShare(in_image, sensor_msgs::image_encodings::BGR8);
	}
	catch(cv_bridge::Exception& ex){
		ROS_ERROR("cv_bridge exception: %s", ex.what());
		return;
	}
	image_filled = true;
}

/*Receive a navsat fix message.*/
void received_navsat(const sensor_msgs::NavSatFix& in_fix){
	current_fix = in_fix;
	fix_filled = true;
}

/*Save image and update fix file.*/
bool save_image_fix(cv::Mat image, sensor_msgs::NavSatFix fix, std::string date_time){
	fs::path image_path = save_path / (date_time + "_" + suffix + ".jpg");
	imwrite(image_path.string(), image);
	outfile << suffix << "\t" << date_time << "\t" << fix.header.frame_id << "\tlat:" << fix.latitude << "\tlong:" << fix.longitude << std::endl;
	ROS_INFO("%s: Wrote image and fix.", suffix.c_str());
	return true;
}

/*Get pulse to save image and fix.*/
void received_pulse(const std_msgs::Int32& in_pulse){
	if(!image_filled){
		ROS_ERROR("%s Pulse %d: No new image. Check camera.", suffix.c_str(), in_pulse.data);
		return;
	}
	if(!fix_filled){
		ROS_INFO("%s Pulse %d: No new gps fix. Using old gps fix.", suffix.c_str(), in_pulse.data);
	}
	image_filled = false;
	fix_filled = false;
	save_image_fix(current_image_ptr->image, current_fix, get_date_time_str());
}

/*Execute program.*/
int main(int argc, char **argv){
	ros::init(argc, argv, "sync_pulse");
	ros::NodeHandle nh;
	ros::NodeHandle node_private("~");
	//Create message subscribers
	ros::Subscriber sub_pulse = nh.subscribe("sync/pulse", 1000, &received_pulse);
	ros::Subscriber sub_navsat = nh.subscribe("navsat/fix", 1000, &received_navsat);
	image_transport::ImageTransport it(nh);
	image_transport::Subscriber sub_image = it.subscribe("camera/image", 1, received_image);
	//Get node parameters
	std::string tmp_save_path;
	node_private.param<std::string>("camera_suffix", suffix, "camera1");
	node_private.param<std::string>("save_path", tmp_save_path, "~/Desktop/images_" + suffix);
	save_path = tmp_save_path;
	//Check directory
	try{
		if(fs::exists(save_path) && fs::is_directory(save_path)){
			ROS_INFO("Using directory %s to save images.", save_path.string().c_str());
		}
		else if(fs::exists(save_path)){
			ROS_ERROR("Path %s already exists but is not a directory. Aborting.", save_path.string().c_str());
			return 0;
		}
		else{
			ROS_INFO("Path %s does not exist. Creating directory.", save_path.string().c_str());
			if(!fs::create_directories(save_path)){
				ROS_ERROR("Directory %s could not be created. Aborting.", save_path.string().c_str());
				return 0;
			}
		}
		fs::path fix_file_path = save_path / ("fix_" + suffix + ".txt");
		outfile.open(fix_file_path.string().c_str(), std::ios::app);
	}
	catch(const fs::filesystem_error& ex){
		ROS_ERROR("%s", ex.what());
		return 0;
	}
	//Initialize current_fix in case no gps is connected
	current_fix.header.seq = 1;
	current_fix.header.stamp = ros::Time::now();
	current_fix.header.frame_id = "fake";
	current_fix.status.status = -1;
	current_fix.status.service = 0;
	current_fix.latitude = 0.0;
	current_fix.longitude = 0.0;
	current_fix.altitude = 0.0;
	current_fix.position_covariance = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
	current_fix.position_covariance_type = 0;
	//Wait for messages
	ros::spin();
	outfile.close();
}
