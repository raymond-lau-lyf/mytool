#include <ros/ros.h>
#include <iostream>
#include <fstream>
#include <sensor_msgs/Image.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <std_msgs/Int32.h>
using namespace std;
using namespace cv;
ros::Publisher pub_image;

void callback(const sensor_msgs::ImageConstPtr& img_msg) 
{
    cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(img_msg, img_msg->encoding);

    int height = img_msg->height;
    int width = img_msg->width;
	cv::Mat raw_image = cv_ptr->image;
    cv::Mat mask(512, 640, CV_8UC1, cv::Scalar(0)); // create a black mask image with the desired size
    cv::rectangle(mask, cv::Rect(0, 0, 640, 400), cv::Scalar(255), -1); // draw a white rectangle in the top half
    cv::Mat cut_image=raw_image.clone();
    cv::bitwise_and(raw_image,mask,cut_image); //TODO


    sensor_msgs::ImagePtr output_msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", cut_image).toImageMsg();
    output_msg->header = img_msg->header;
    pub_image.publish(output_msg);
}


int main(int argc, char **argv) {

    ros::init(argc, argv, "image_cut"); 
    ros::NodeHandle nh;
    pub_image = nh.advertise<sensor_msgs::Image>("/camera/edge/cut",100);
    ros::Subscriber sub_img = nh.subscribe("/camera/edge", 100, callback);
    ros::Rate loop_rate(30);
    while (nh.ok()) {
      ros::spinOnce();
      loop_rate.sleep();
    }
    return 0;
}

