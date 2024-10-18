#include "bumperbot_detection/litter_coordinate_transformer.h"


LitterCoordinateTransformer::LitterCoordinateTransformer(const ros::NodeHandle &nh, 
                                                         const std::string base_frame, 
                                                         const std::string camera_frame) :
                    nh_(nh),
                    base_frame_(base_frame),
                    camera_frame_(camera_frame)
{

    // Initialize subscriber to get litter coordinates from the camera frame
    litter_coord_camera_frame_sub_ = nh_.subscribe("camera_frame/detected_object_coordinates", 10, &LitterCoordinateTransformer::litterCoordinatesCallback, this);

    // Initialize publisher to advertise litter coordinates in base frame
    litter_coord_base_frame_pub_ = nh_.advertise<geometry_msgs::PointStamped>("base_frame/detected_object_coordinates", 10);

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

        // Drop the rotation segment of the transform by setting the rotation to identity (no rotation)
        transform_stamped.transform.rotation.x = 0.0;
        transform_stamped.transform.rotation.y = 0.0;
        transform_stamped.transform.rotation.z = 0.0;
        transform_stamped.transform.rotation.w = 1.0;

        // Transform the litter point to the base_footprint frame
        geometry_msgs::PointStamped litter_in_base_frame;
        tf2::doTransform(*litter_point, litter_in_base_frame, transform_stamped);

        // Extract x (side distance) and z (distance away)
        double x_side_distance = litter_in_base_frame.point.x;
        double z_distance_away = litter_in_base_frame.point.z;

        // Log the transformed coordinates
        ROS_INFO("Litter coordinates (base_footprint): x = %.2f, z = %.2f", x_side_distance, z_distance_away);

        // Publisj the litter coordinates in base frame
        litter_coord_base_frame_pub_.publish(litter_in_base_frame);
    }
    catch (tf2::TransformException& ex)
    {
        ROS_WARN("Could not transform litter coordinates: %s", ex.what());
    }
}