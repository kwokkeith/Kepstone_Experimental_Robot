#include "bumperbot_detection/image_compressor.h"

ImageCompressor::ImageCompressor(ros::NodeHandle& nh) : 
    nh_(nh)
{
    // Load global parameters for services/topics
    std::string image_topic_sub_;
    std::string compressed_image_topic_pub_;
    nh_.getParam("/camera/topics/color_image_raw", image_topic_sub_);
    nh_.getParam("/camera/topics/color_image_front_compressed", compressed_image_topic_pub_);

    // Initialize subscriber to get raw image
    image_sub_ = nh_.subscribe(image_topic_sub_, 1, &ImageCompressor::imageCallback, this);

    // Initialize publisher to advertise compressed image
    compressed_image_pub_ = nh_.advertise<sensor_msgs::CompressedImage>(compressed_image_topic_pub_, 10);
}

void ImageCompressor::imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    try 
    {
        // Convert ROS image to OpenCV format
        cv::Mat image = cv_bridge::toCvCopy(msg, "bgr8")->image;

        // Compress the image to JPEG format
        std::vector<uchar> buffer;
        std::vector<int> compression_params = {cv::IMWRITE_JPEG_QUALITY, 80};  
        cv::imencode(".jpg", image, buffer, compression_params);

        // Create and publish a sensor_msgs::CompressedImage
        sensor_msgs::CompressedImage compressed_msg;
        compressed_msg.header = msg->header;   // Copy timestamp and frame_id
        compressed_msg.format = "jpeg";
        compressed_msg.data.assign(buffer.begin(), buffer.end());

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