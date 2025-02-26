#include "bumperbot_detection/image_compressor.h"

ImageCompressor::ImageCompressor(ros::NodeHandle& nh) : 
    nh_(nh), 
    it_(nh)
{
    // Load global parameters for services/topics
    std::string image_topic_sub_;
    std::string compressed_image_topic_pub_;
    nh_.getParam("/camera/topics/color_image_raw", image_topic_sub_);
    nh_.getParam("/camera/topics/color_image_compressed", compressed_image_topic_pub_);

    // Initialize subscriber to get raw image
    image_sub_ = it_.subscribe(image_topic_sub_, 1, &ImageCompressor::imageCallback, this);

    // Initialize publisher to advertise compressed image
    compressed_image_pub_ = it_.advertise(compressed_image_topic_pub_, 10);
}

void ImageCompressor::imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    try 
    {
        // Convert ROS image to OpenCV image
        cv::Mat image = cv_bridge::toCvCopy(msg, "bgr8")->image;

        // Publish the image as compressed JPEG
        sensor_msgs::ImagePtr compressed_msg = cv_bridge::CvImage(msg->header, "bgr8", image).toImageMsg();
        compressed_image_pub_.publish(compressed_msg);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("Failed to compress image: %s", e.what());
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "image_compressor");
    ros::NodeHandle nh;

    ImageCompressor image_compressor(nh);

    ros::spin();
    return 0;
}