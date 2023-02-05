#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

cv::Mat left_image;
cv::Mat right_image;
bool left_received = false;
bool right_received = false;

void leftImageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  cv_bridge::CvImageConstPtr cv_ptr;
  try
  {
    cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::MONO8);
    left_image = cv_ptr->image.clone();
    left_received = true;
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }
}

void rightImageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  cv_bridge::CvImageConstPtr cv_ptr;
  try
  {
    cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::MONO8);
    right_image = cv_ptr->image.clone();
    right_received = true;
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "stereo_to_depth_image");
  ros::NodeHandle nh;

  ros::Subscriber left_image_sub = nh.subscribe("/pole0/camera/image_raw", 1, leftImageCallback);
  ros::Subscriber right_image_sub = nh.subscribe("/pole1/camera/image_raw", 1, rightImageCallback);
  ros::Publisher depth_image_pub = nh.advertise<sensor_msgs::Image>("depth_image", 1);

  ros::Rate rate(10);
  while (ros::ok())
  {
    if (left_received && right_received)
    {
      cv::Mat depth_image;
      cv::Ptr<cv::StereoBM> stereo = cv::StereoBM::create(16, 15);
      stereo->compute(left_image, right_image, depth_image);
      depth_image.convertTo(depth_image, CV_8UC1, 1.0 / 16.0);

      cv_bridge::CvImage depth_image_msg;
      depth_image_msg.header.stamp = ros::Time::now();
      depth_image_msg.header.frame_id = "depth_image";
      depth_image_msg.encoding = sensor_msgs::image_encodings::MONO8;
      depth_image_msg.image = depth_image;

      depth_image_pub.publish(depth_image_msg.toImageMsg());

      left_received = false;
      right_received = false;
    }

    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}
