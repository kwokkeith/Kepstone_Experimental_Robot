#include "bumperbot_localization/kalman_filter.h"

#define COLLISION_THRESHOLD 100  // Example threshold for detecting collisions based on acceleration

KalmanFilter::KalmanFilter(const ros::NodeHandle &nh) 
    : nh_(nh),
      angular_mean_(0.0),
      angular_variance_(1000.0),
      position_mean_(0.0),
      position_variance_(1000.0),
      imu_angular_z_(0.0),
      lidar_pos_(0.0),
      is_first_odom_(true),
      last_angular_z_(0.0),
      motion_variance_(4.0),
      measurement_variance_(0.5),
      odometry_enabled_(true)
      
{
    odom_sub_ = nh_.subscribe("bumperbot_controller/odom_noisy", 1000, &KalmanFilter::odomCallback, this);
    imu_sub_ = nh_.subscribe("imu", 1000, &KalmanFilter::imuCallback, this);
    lidar_sub_ = nh_.subscribe("/bumperbot/laser/fused_scan", 1000, &KalmanFilter::lidarCallback, this);
    odom_pub_ = nh_.advertise<nav_msgs::Odometry>("/bumperbot_controller/odom_kalman", 10);
    amcl_initial_pose_pub_ = nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>("/initialpose", 1);

    // Store the last known pose
    last_known_pose_.position.x = 0;
    last_known_pose_.position.y = 0;
    last_known_pose_.orientation.w = 1.0;  // No rotation by default
}

void KalmanFilter::imuCallback(const sensor_msgs::Imu &imu)
{
    imu_angular_z_ = imu.angular_velocity.z;

    // Detect sudden jerks, indicating a collision
    double acceleration = sqrt(pow(imu.linear_acceleration.x, 2) +
                               pow(imu.linear_acceleration.y, 2) +
                               pow(imu.linear_acceleration.z, 2));


    // Assume a threshold for acceleration indicating a collision
    if (acceleration > COLLISION_THRESHOLD) {
        triggerRelocalization();
    }
}

void KalmanFilter::odomCallback(const nav_msgs::Odometry &odom)
{
    if (is_first_odom_) {
        angular_mean_ = odom.twist.twist.angular.z;
        last_angular_z_ = odom.twist.twist.angular.z;
        is_first_odom_ = false;
        return;
    }

    if (!odometry_enabled_) {
        // Temporarily disable odometry processing during recovery
        return;
    }

    // Calculate motion from odometry
    double angular_motion = odom.twist.twist.angular.z - last_angular_z_;
    last_angular_z_ = odom.twist.twist.angular.z;

    // State prediction and measurement update for angular velocity
    angularStatePrediction(angular_motion);
    angularMeasurementUpdate();

    // State prediction and measurement update for position from LiDAR
    positionMeasurementUpdate();

    // Update the Kalman odometry with the fused angular velocity and position, then publish
    nav_msgs::Odometry kalman_odom = odom;
    kalman_odom.twist.twist.angular.z = angular_mean_;
    kalman_odom.pose.pose.position.x = position_mean_;  // Assuming position x is updated by LiDAR

    // Update last known pose for relocalization
    last_known_pose_ = odom.pose.pose;
 
    odom_pub_.publish(kalman_odom);
}

void KalmanFilter::lidarCallback(const sensor_msgs::LaserScan &scan)
{
    // Simple LiDAR position estimate by averaging valid range readings
    int valid_readings = 0;
    for (int i = 0; i < scan.ranges.size(); ++i) {
        if (scan.ranges[i] >= scan.range_min && scan.ranges[i] <= scan.range_max) {
            lidar_pos_ += scan.ranges[i];
            valid_readings++;
        }
    }
    if (valid_readings > 0) {
        lidar_pos_ /= valid_readings;  // Average range as a proxy for position
    }
}

void KalmanFilter::triggerRelocalization()
{
    ROS_INFO("Collision detected! Triggering relocalization...");

    // Temporarily disable odometry updates
    enableOdometry(false);

    // Recover AMCL by resetting the particles globally
    recoverAMCLParticles();
}

void KalmanFilter::recoverAMCLParticles()
{
    ROS_INFO("Resetting AMCL particles for relocalization...");

    // Get the last known pose 
    geometry_msgs::PoseWithCovarianceStamped new_pose;
    new_pose.header.stamp = ros::Time::now();
    new_pose.header.frame_id = "map";  // Pose estimate should be in the "map" frame

    // Set the last_known_pose to feed AMCL 
    new_pose.pose.pose.position.x = last_known_pose_.position.x;
    new_pose.pose.pose.position.y = last_known_pose_.position.y;
    new_pose.pose.pose.orientation = last_known_pose_.orientation;  // Copy the last known orientation

    // Add some uncertainty to the covariance matrix 
    // Covariance matrix: [x, y, theta] with some uncertainty
    new_pose.pose.covariance[0] = 0.5;   // Variance in x
    new_pose.pose.covariance[7] = 0.5;   // Variance in y
    new_pose.pose.covariance[35] = 0.2;  // Variance in theta (yaw)

    // Publish the new pose estimate to AMCL's /initialpose topic
    amcl_initial_pose_pub_.publish(new_pose);
    ROS_INFO("Published pose estimate to AMCL.");

    // Re-enable odometry updates after AMCL has reset
    enableOdometry(true);
}

void KalmanFilter::enableOdometry(bool enable)
{
    odometry_enabled_ = enable;
}

void KalmanFilter::angularMeasurementUpdate()
{
    angular_mean_ = (measurement_variance_ * angular_mean_ + angular_variance_ * imu_angular_z_) 
                    / (angular_variance_ + measurement_variance_);
    angular_variance_ = (angular_variance_ * measurement_variance_) / (angular_variance_ + measurement_variance_);
}

void KalmanFilter::angularStatePrediction(double angular_motion)
{
    angular_mean_ += angular_motion;
    angular_variance_ += motion_variance_;
}

void KalmanFilter::positionMeasurementUpdate()
{
    double lidar_variance = 2.0;  // Assumed variance for LiDAR
    position_mean_ = (lidar_variance * position_mean_ + position_variance_ * lidar_pos_) 
                     / (position_variance_ + lidar_variance);
    position_variance_ = (position_variance_ * lidar_variance) / (position_variance_ + lidar_variance);
}
