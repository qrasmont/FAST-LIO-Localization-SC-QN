#ifndef FAST_LIO_LOCALIZATION_SC_QN_MAIN_H
#define FAST_LIO_LOCALIZATION_SC_QN_MAIN_H

///// common headers
#include <ctime>
#include <cmath>
#include <chrono> //time check
#include <vector>
#include <mutex>
#include <string>
#include <memory>
#include <utility> // pair, make_pair
///// ROS
#include "rclcpp/rclcpp.hpp"
#include <rosbag2_cpp/reader.hpp>
#include <rosbag2_cpp/readers/sequential_reader.hpp>
#include <tf2/LinearMath/Quaternion.h> // to Quaternion_to_euler
#include <tf2/LinearMath/Matrix3x3.h>  // to Quaternion_to_euler
#include <tf2/transform_datatypes.h>   // createQuaternionFromRPY
#include <tf2_eigen/tf2_eigen.hpp>  // tf <-> eigen
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/transform_broadcaster.h> // broadcaster
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
///// PCL
#include <pcl/point_types.h>                 //pt
#include <pcl/point_cloud.h>                 //cloud
#include <pcl/common/transforms.h>           //transformPointCloud
#include <pcl_conversions/pcl_conversions.h> //ros<->pcl
///// Nano-GICP
#include <nano_gicp/point_type_nano_gicp.hpp>
#include <nano_gicp/nano_gicp.hpp>
///// Quatro
#include <quatro/quatro_module.h>
///// Eigen
#include <Eigen/Eigen> // whole Eigen library: Sparse(Linearalgebra) + Dense(Core+Geometry+LU+Cholesky+SVD+QR+Eigenvalues)
///// coded headers
#include "utilities.hpp"
#include "pose_pcd.hpp"
#include "map_matcher.h"

using namespace std::chrono;
typedef message_filters::sync_policies::ApproximateTime<nav_msgs::msg::Odometry, sensor_msgs::msg::PointCloud2> odom_pcd_sync_pol;

////////////////////////////////////////////////////////////////////////////////////////////////////
class FastLioLocalizationScQn : public rclcpp::Node
{
private:
    using PointCloudT = sensor_msgs::msg::PointCloud2;
    using PathT = nav_msgs::msg::Path;
    using MarkerT = visualization_msgs::msg::Marker;
    using PoseStampedT = geometry_msgs::msg::PoseStamped;
    using OdomT = nav_msgs::msg::Odometry;

    ///// basic params
    std::string map_frame_;
    ///// shared data - odom and pcd
    std::mutex keyframes_mutex_, vis_mutex_;
    bool is_initialized_ = false;
    int current_keyframe_idx_ = 0;
    PosePcd last_keyframe_;
    std::vector<PosePcdReduced> saved_map_from_bag_;
    Eigen::Matrix4d last_corrected_TF_ = Eigen::Matrix4d::Identity();
    ///// map match
    double keyframe_dist_thr_;
    double voxel_res_;
    ///// visualize
    bool saved_map_vis_switch_ = true;
    std::unique_ptr<tf2_ros::TransformBroadcaster> broadcaster_;
    nav_msgs::msg::Path raw_odom_path_, corrected_odom_path_, map_path_, realtime_corrected_path_;
    std::vector<std::pair<pcl::PointXYZ, pcl::PointXYZ>> matched_pairs_xyz_; // for vis
    pcl::PointCloud<pcl::PointXYZ> raw_odoms_, corrected_odoms_;
    pcl::PointCloud<PointType> saved_map_pcd_; // for vis
    ///// ros
    rclcpp::Publisher<PointCloudT>::SharedPtr corrected_odom_pub_, odom_pub_;
    rclcpp::Publisher<PathT>::SharedPtr corrected_path_pub_, path_pub_, map_path_pub_, realtime_corrected_path_pub_;
    rclcpp::Publisher<PointCloudT>::SharedPtr corrected_current_pcd_pub_, saved_map_pub_;
    rclcpp::Publisher<PoseStampedT>::SharedPtr realtime_pose_pub_;
    rclcpp::Publisher<MarkerT>::SharedPtr map_match_pub_;
    rclcpp::Publisher<PointCloudT>::SharedPtr debug_src_pub_, debug_dst_pub_, debug_coarse_aligned_pub_, debug_fine_aligned_pub_;
    rclcpp::TimerBase::SharedPtr match_timer_, initial_publish_timer_;
    // odom, pcd sync subscriber
    std::shared_ptr<message_filters::Synchronizer<odom_pcd_sync_pol>> sub_odom_pcd_sync_ = nullptr;
    std::shared_ptr<message_filters::Subscriber<OdomT>> sub_odom_ = nullptr;
    std::shared_ptr<message_filters::Subscriber<PointCloudT>> sub_pcd_ = nullptr;
    ///// Map match
    std::shared_ptr<MapMatcher> map_matcher_;

public:
    explicit FastLioLocalizationScQn();
    ~FastLioLocalizationScQn() {};

private:
    // methods
    void init_params();
    void initialMapPublish();
    void updateOdomsAndPaths(const PosePcd &pose_pcd_in);
    bool checkIfKeyframe(const PosePcd &pose_pcd_in, const PosePcd &latest_pose_pcd);
    visualization_msgs::msg::Marker getMatchMarker(const std::vector<std::pair<pcl::PointXYZ, pcl::PointXYZ>> &match_xyz_pairs);
    void loadMap(const std::string &saved_map_path);
    // cb
    void odomPcdCallback(const nav_msgs::msg::Odometry::ConstSharedPtr &odom_msg, const sensor_msgs::msg::PointCloud2::ConstSharedPtr &pcd_msg);
    void matchingTimerFunc();
};

#endif
