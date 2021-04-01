#pragma once

#include <string>
#include <math.h>
#include <algorithm>

// ROS stuff
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>
#include <cv_bridge/cv_bridge.h>
#include <tf/transform_listener.h>

// ROS messages
#include <sensor_msgs/Image.h>
#include <cdt_msgs/ObjectList.h>
#include <cdt_msgs/Object.h>
#include <sensor_msgs/PointCloud2.h>

// Eigen library
#include <Eigen/Dense>

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <iostream>

//PointCloud stuff
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>
#include <tf_conversions/tf_eigen.h>

// Colours
enum Colour { RED=0, YELLOW=1, GREEN=2,  BLUE=3 };

class ObjectDetector
{
    struct Report{
        cdt_msgs::Object obj_;
        double bb_width_;
        double bb_height_;
        double cam_x_;
        double cam_y_;
        double cam_z_;
        int nb_inliers_;
    };

    struct CompReports {
        bool operator() (Report i,Report j) { return (i.nb_inliers_<j.nb_inliers_);}
    } compareReports_;

    // Members
    // Image subscriber
    image_transport::Subscriber image_sub_;

    // Lidar tracking
    ros::Subscriber lidar_sub_;
    std::vector<sensor_msgs::PointCloud2> recent_lidar_scans_;
    int nb_scans_kept_;

    // Detected objects publisher
    ros::Publisher objects_pub_;

    // TF listener
    tf::TransformListener tf_listener_;

    // List of detected objects
    cdt_msgs::ObjectList detected_objects_;
    std::vector<Report> detected_dogs_;
    std::vector<Report> detected_barrows_;
    std::vector<Report> detected_barrels_;
    std::vector<Report> detected_computers_;


    // Time since last obs for each object
    double last_time_dog_, last_time_barrel_, last_time_barrow_, last_time_computer_;
    double timeout_limit_, max_nb_obs_;

    // Input topics
    std::string input_image_topic_;
    std::string input_lidar_topic_;
    std::string base_frame_;
    std::string fixed_frame_;
    std::string image_frame_;

    // Output topics
    std::string output_objects_topic_;

    // Internals
    double camera_extrinsic_x_; // Position from base_link to camera coordinates (in meters)
    double camera_extrinsic_y_; // Position from base_link to camera coordinates (in meters)
    double camera_extrinsic_z_; // Position from base_link to camera coordinates (in meters)

    double camera_fx_; // Intrinsic calibration: Focal length (in pixels)
    double camera_fy_; // Intrinsic calibration: Focal length (in pixels)
    double camera_cx_; // Intrinsic calibration: Camera center (in pixels)
    double camera_cy_; // Intrinsic calibration: Camera center (in pixels)

    double img_height_, img_width_;
    
    // Objects' heights
    double barrel_real_height_, barrow_real_height_, computer_real_height_, dog_real_height_;
    double hor_angle_margin_, ver_angle_margin_;
public:
    // Constructor
    ObjectDetector(ros::NodeHandle &nh);

private:
    // Read ROS parameters
    void readParameters(ros::NodeHandle &nh);

    //Determine object position
    bool getObjectPosition(const float &pixelx, const float &pixely, const std_msgs::Header &imgheader, Report &report);

    // Find object to report from multiple observations
    cdt_msgs::Object chooseReport(const std::vector<Report> &reports);

    // The callback implements all the actions
    void imageCallback(const sensor_msgs::ImageConstPtr &in_msg);

    // The callback to keep track of Lidar scans
    void lidarCallback(const sensor_msgs::PointCloud2 in_msg);

    // Finds the closest lidar scan in memory to the time query
    bool findClosestLidarScan(const ros::Time &time_query, sensor_msgs::PointCloud2 &point_cloud);

    // Converts message to opencv image
    void convertMessageToImage(const sensor_msgs::ImageConstPtr &in_msg, cv::Mat& out_image, ros::Time& out_timestamp);

    // Detects the specified colour within the input image (BGR)
    cv::Mat applyColourFilter(const cv::Mat &in_image_bgr, const Colour &colour);

    // Detects the specified colour within the input image (BGR)
    cv::Mat applyBoundingBox(const cv::Mat1b &in_mask, double &x, double &y, double &width, double &height);


    // Implements the procedures to recognize objects

    bool recognizeObject(const cv::Mat &in_image, const Colour &colour, const std_msgs::Header &in_header, 
                                  Report &report_out);
                                  
    bool wasObjectDetected(std::string object_name);
};
