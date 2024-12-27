#include "bumperbot_detection/litter_coordinate_transformer.h"


LitterCoordinateTransformer::LitterCoordinateTransformer(const ros::NodeHandle &nh, 
                                                         const std::string base_frame, 
                                                         const std::string camera_frame) :
                    nh_(nh),
                    base_frame_(base_frame),
                    camera_frame_(camera_frame)
{
    // Get Global Parameter for services/topics
    std::string detected_object_coordinates_base_topic_pub_;
    std::string detected_object_coordinates_cam_topic_pub_;
    nh_.getParam("/litter_detection/topics/detected_object_coordinates_base", detected_object_coordinates_base_topic_pub_);
    nh_.getParam("/litter_detection/topics/detected_object_coordinates_camera", detected_object_coordinates_cam_topic_pub_);

    // Initialize subscriber to get litter coordinates from the camera frame
    litter_coord_camera_frame_sub_ = nh_.subscribe(detected_object_coordinates_cam_topic_pub_, 10, &LitterCoordinateTransformer::litterCoordinatesCallback, this);

    // Initialize publisher to advertise litter coordinates in base frame
    litter_coord_base_frame_pub_ = nh_.advertise<geometry_msgs::PointStamped>(detected_object_coordinates_base_topic_pub_, 10);

    // Initialize the TF2 listener to listen for transform between camera_frame and base_footprint
    tf_listener_ = new tf2_ros::TransformListener(tf_buffer_);
}


LitterCoordinateTransformer::~LitterCoordinateTransformer()
{
    delete tf_listener_;
}


// Callback to process litter coordinates and transform them to the base_footprint frame
void LitterCoordinateTransformer::litterCoordinatesCallback(const geometry_msgs::PointStamped::ConstPtr& litter_point)
{
    try
    {
        // Lookup the transform from camera_frame to base_footprint
        geometry_msgs::TransformStamped transform_stamped = tf_buffer_.lookupTransform(base_frame_, camera_frame_, ros::Time(0), ros::Duration(1.0));

        // Drop the rotation segment by only using the translation part
        geometry_msgs::PointStamped litter_in_base_frame;
        litter_in_base_frame.header.frame_id = base_frame_;
        litter_in_base_frame.header.stamp = litter_point->header.stamp;

        // Apply translation from camera_frame to base_footprint
        litter_in_base_frame.point.x = litter_point->point.x + transform_stamped.transform.translation.x;
        litter_in_base_frame.point.y = litter_point->point.y + transform_stamped.transform.translation.y;
        litter_in_base_frame.point.z = litter_point->point.z + transform_stamped.transform.translation.z;

        // Extract x (side distance) and z (distance away) relative to base_footprint
        double x_side_distance = litter_in_base_frame.point.x;
        double z_distance_away = litter_in_base_frame.point.z;

        // Log the transformed coordinates
        ROS_INFO("Litter coordinates (base_footprint): x = %.2f, y = %.2f, z = %.2f", 
                 litter_in_base_frame.point.x, litter_in_base_frame.point.y, litter_in_base_frame.point.z);

        // Publish the litter coordinates in base frame
        litter_coord_base_frame_pub_.publish(litter_in_base_frame);
    }
    catch (tf2::TransformException& ex)
    {
        ROS_WARN("Could not transform litter coordinates: %s", ex.what());
    }
}