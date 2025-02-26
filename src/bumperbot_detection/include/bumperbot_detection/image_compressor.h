#ifndef IMAGE_COMPRESSOR_H
#define IMAGE_COMPRESSOR_H

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CompressedImage.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <image_transport/image_transport.h>


class ImageCompressor
{

public:
    ImageCompressor(ros::NodeHandle& nh);
    ~ImageCompressor() = default;

private:
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;
    image_transport::Publisher compressed_image_pub_;

    void imageCallback(const sensor_msgs::ImageConstPtr& image_msg);

};

#endif