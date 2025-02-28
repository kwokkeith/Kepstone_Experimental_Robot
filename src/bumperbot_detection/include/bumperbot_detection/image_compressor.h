#ifndef IMAGE_COMPRESSOR_H
#define IMAGE_COMPRESSOR_H

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CompressedImage.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>


class ImageCompressor
{

public:
    ImageCompressor(ros::NodeHandle& nh);
    ~ImageCompressor() = default;

private:
    ros::NodeHandle nh_;
    ros::Subscriber image_sub_;
    ros::Publisher compressed_image_pub_;

    void imageCallback(const sensor_msgs::ImageConstPtr& image_msg);

};

#endif