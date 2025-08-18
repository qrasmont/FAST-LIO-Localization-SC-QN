#include "fast_lio_localization_sc_qn.h"

FastLioLocalizationScQn::FastLioLocalizationScQn() : Node("fast_lio_localization_sc_qn_node")
{
    init_params();
    RCLCPP_WARN(this->get_logger(), "Main class, starting node...");
}

void FastLioLocalizationScQn::init_params()
{
    ////// ROS params
    std::string saved_map_path;
    double map_match_hz;
    MapMatcherConfig mm_config;
    auto &gc = mm_config.gicp_config_;
    auto &qc = mm_config.quatro_config_;
    auto &sc = mm_config.scancontext_config_;

    /* basic */
    this->declare_parameter<std::string>("basic.map_frame", "map");
    this->declare_parameter<std::string>("basic.saved_map", "/home/mason/kitti.bag");
    this->declare_parameter<double>("basic.map_match_hz", 1.0);
    this->declare_parameter<double>("basic.visualize_voxel_size", 1.0);
    this->get_parameter("basic.map_frame", map_frame_);
    this->get_parameter("basic.saved_map", saved_map_path);
    this->get_parameter("basic.map_match_hz", map_match_hz);
    this->get_parameter("basic.visualize_voxel_size", voxel_res_);

    /* keyframe */
    this->declare_parameter<double>("keyframe.keyframe_threshold", 1.0);
    this->declare_parameter<bool>("keyframe.enable_rotation_check", false);
    this->declare_parameter<double>("keyframe.keyframe_rotation_threshold_deg", 270.0);
    this->declare_parameter<double>("keyframe.in_place_translation_threshold", 0.2);
    this->declare_parameter<int>("keyframe.num_submap_keyframes", 5);
    this->get_parameter("keyframe.keyframe_threshold", keyframe_dist_thr_);
    this->get_parameter("keyframe.enable_rotation_check", enable_rotation_check_);
    double keyframe_rotation_threshold_deg;
    this->get_parameter("keyframe.keyframe_rotation_threshold_deg", keyframe_rotation_threshold_deg);
    keyframe_rotation_threshold_rad_ = keyframe_rotation_threshold_deg * M_PI / 180.0;
    this->get_parameter("keyframe.in_place_translation_threshold", in_place_translation_threshold_);
    this->get_parameter("keyframe.num_submap_keyframes", mm_config.num_submap_keyframes_);

    /* match */
    this->declare_parameter<double>("match.scancontext_max_correspondence_distance", 15.0);
    this->declare_parameter<double>("match.quatro_nano_gicp_voxel_resolution", 0.3);
    this->get_parameter("match.scancontext_max_correspondence_distance", mm_config.scancontext_max_correspondence_distance_);
    this->get_parameter("match.quatro_nano_gicp_voxel_resolution", mm_config.voxel_res_);
    /* nano */
    this->declare_parameter<int>("nano_gicp.thread_number", 0);
    this->declare_parameter<double>("nano_gicp.icp_score_threshold", 10.0);
    this->declare_parameter<int>("nano_gicp.correspondences_number", 15);
    this->declare_parameter<double>("nano_gicp.max_correspondence_distance", 0.01);
    this->declare_parameter<int>("nano_gicp.max_iter", 32);
    this->declare_parameter<double>("nano_gicp.transformation_epsilon", 0.01);
    this->declare_parameter<double>("nano_gicp.euclidean_fitness_epsilon", 0.01);
    this->declare_parameter<int>("nano_gicp.ransac.max_iter", 5);
    this->declare_parameter<double>("nano_gicp.ransac.outlier_rejection_threshold", 1.0);
    this->get_parameter("nano_gicp.thread_number", gc.nano_thread_number_);
    this->get_parameter("nano_gicp.icp_score_threshold", gc.icp_score_thr_);
    this->get_parameter("nano_gicp.correspondences_number", gc.nano_correspondences_number_);
    this->get_parameter("nano_gicp.max_correspondence_distance", gc.max_corr_dist_);
    this->get_parameter("nano_gicp.max_iter", gc.nano_max_iter_);
    this->get_parameter("nano_gicp.transformation_epsilon", gc.transformation_epsilon_);
    this->get_parameter("nano_gicp.euclidean_fitness_epsilon", gc.euclidean_fitness_epsilon_);
    this->get_parameter("nano_gicp.ransac.max_iter", gc.nano_ransac_max_iter_);
    this->get_parameter("nano_gicp.ransac.outlier_rejection_threshold", gc.ransac_outlier_rejection_threshold_);
    /* quatro */
    this->declare_parameter<bool>("quatro.enable", false);
    this->declare_parameter<bool>("quatro.optimize_matching", true);
    this->declare_parameter<double>("quatro.distance_threshold", 30.0);
    this->declare_parameter<int>("quatro.max_correspondences", 200);
    this->declare_parameter<double>("quatro.fpfh_normal_radius", 0.02);
    this->declare_parameter<double>("quatro.fpfh_radius", 0.04);
    this->declare_parameter<bool>("quatro.estimating_scale", false);
    this->declare_parameter<double>("quatro.noise_bound", 0.25);
    this->declare_parameter<double>("quatro.rotation.gnc_factor", 0.25);
    this->declare_parameter<double>("quatro.rotation.rot_cost_diff_threshold", 0.25);
    this->declare_parameter<int>("quatro.rotation.num_max_iter", 50);
    this->declare_parameter<double>("scancontext.dist_thres", 0.2);
    this->get_parameter("quatro.enable", mm_config.enable_quatro_);
    this->get_parameter("quatro.optimize_matching", qc.use_optimized_matching_);
    this->get_parameter("quatro.distance_threshold", qc.quatro_distance_threshold_);
    this->get_parameter("quatro.max_correspondences", qc.quatro_max_num_corres_);
    this->get_parameter("quatro.fpfh_normal_radius", qc.fpfh_normal_radius_);
    this->get_parameter("quatro.fpfh_radius", qc.fpfh_radius_);
    this->get_parameter("quatro.estimating_scale", qc.estimat_scale_);
    this->get_parameter("quatro.noise_bound", qc.noise_bound_);
    this->get_parameter("quatro.rotation.gnc_factor", qc.rot_gnc_factor_);
    this->get_parameter("quatro.rotation.rot_cost_diff_threshold", qc.rot_cost_diff_thr_);
    this->get_parameter("quatro.rotation.num_max_iter", qc.quatro_max_iter_);
    this->get_parameter("scancontext.dist_thres", sc.dist_thres_);

    ////// Matching init
    map_matcher_ = std::make_shared<MapMatcher>(mm_config);

    ////// ROS things
    broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    raw_odom_path_.header.frame_id = map_frame_;
    corrected_odom_path_.header.frame_id = map_frame_;
    map_path_.header.frame_id = map_frame_;
    realtime_corrected_path_.header.frame_id = map_frame_;

    // For latched topics
    rclcpp::QoS latched_qos(1);
    latched_qos.transient_local();

    // publishers
    odom_pub_ = this->create_publisher<PointCloudT>("/ori_odom", 10);
    path_pub_ = this->create_publisher<PathT>("/ori_path", 10);
    map_path_pub_ = this->create_publisher<PathT>("/map_path", latched_qos);
    corrected_odom_pub_ = this->create_publisher<PointCloudT>("/corrected_odom", 10);
    corrected_path_pub_ = this->create_publisher<PathT>("/corrected_path", 10);
    corrected_current_pcd_pub_ = this->create_publisher<PointCloudT>("/corrected_current_pcd", 10);
    map_match_pub_ = this->create_publisher<MarkerT>("/map_match", 10);
    realtime_corrected_path_pub_ = this->create_publisher<PathT>("/realtime_corrected_path", 10);
    realtime_pose_pub_ = this->create_publisher<PoseStampedT>("/pose_stamped", 10);
    saved_map_pub_ = this->create_publisher<PointCloudT>("/saved_map", latched_qos);
    debug_src_pub_ = this->create_publisher<PointCloudT>("/src", 10);
    debug_dst_pub_ = this->create_publisher<PointCloudT>("/dst", 10);
    debug_coarse_aligned_pub_ = this->create_publisher<PointCloudT>("/coarse_aligned_quatro", 10);
    debug_fine_aligned_pub_ = this->create_publisher<PointCloudT>("/fine_aligned_nano_gicp", 10);

    // subscribers
    sub_odom_ = std::make_shared<message_filters::Subscriber<OdomT>>(this, "/Odometry");
    sub_pcd_ = std::make_shared<message_filters::Subscriber<PointCloudT>>(this, "/cloud_registered");
    sub_odom_pcd_sync_ = std::make_shared<message_filters::Synchronizer<odom_pcd_sync_pol>>(odom_pcd_sync_pol(10), *sub_odom_, *sub_pcd_);
    sub_odom_pcd_sync_->registerCallback(std::bind(&FastLioLocalizationScQn::odomPcdCallback, this, std::placeholders::_1, std::placeholders::_2));

    // Timers
    match_timer_ = this->create_wall_timer(std::chrono::duration<double>(1.0/map_match_hz), std::bind(&FastLioLocalizationScQn::matchingTimerFunc, this));
    initial_publish_timer_ = this->create_wall_timer(std::chrono::seconds(1), std::bind(&FastLioLocalizationScQn::initialMapPublish, this));

    ////// Load map
    loadMap(saved_map_path);
}

void FastLioLocalizationScQn::initialMapPublish()
{
    // One shot
    initial_publish_timer_->cancel();
    RCLCPP_INFO(this->get_logger(), "Timer triggered for initial map publication.");

    // Publish the loaded map and path
    RCLCPP_INFO(this->get_logger(), "Publishing loaded map and path...");
    map_path_.header.stamp = this->get_clock()->now();
    map_path_pub_->publish(map_path_);
    saved_map_pub_->publish(pclToPclRos(saved_map_pcd_, map_frame_, this->get_clock()->now()));
}

void FastLioLocalizationScQn::odomPcdCallback(const OdomT::ConstSharedPtr &odom_msg, const PointCloudT::ConstSharedPtr &pcd_msg)
{
    PosePcd current_frame = PosePcd(*odom_msg, *pcd_msg, current_keyframe_idx_); // to be checked if keyframe or not
    //// 1. realtime pose = last TF * odom
    current_frame.pose_corrected_eig_ = last_corrected_TF_ * current_frame.pose_eig_;
    auto current_pose_stamped_ = poseEigToPoseStamped(current_frame.pose_corrected_eig_, map_frame_);
    current_pose_stamped_.header.stamp = odom_msg->header.stamp;
    realtime_pose_pub_->publish(current_pose_stamped_);

    realtime_corrected_path_.header.stamp = odom_msg->header.stamp;
    realtime_corrected_path_.poses.push_back(current_pose_stamped_);
    realtime_corrected_path_pub_->publish(realtime_corrected_path_);

    geometry_msgs::msg::TransformStamped trans_stamped_msg;
    trans_stamped_msg.transform = tf2::toMsg(poseEigToROSTf(current_frame.pose_corrected_eig_));
    trans_stamped_msg.header.stamp = odom_msg->header.stamp;
    trans_stamped_msg.header.frame_id = map_frame_;
    trans_stamped_msg.child_frame_id = "robot";
    broadcaster_->sendTransform(trans_stamped_msg);

    // pub current scan in corrected pose frame
    corrected_current_pcd_pub_->publish(pclToPclRos(transformPcd(current_frame.pcd_, current_frame.pose_corrected_eig_), map_frame_, odom_msg->header.stamp));

    if (!is_initialized_) //// init only once
    {
        // 1. save first keyframe
        {
            std::lock_guard<std::mutex> lock(keyframes_mutex_);
            last_keyframe_ = current_frame;
        }
        current_keyframe_idx_++;
        //// 2. vis
        {
            std::lock_guard<std::mutex> lock(vis_mutex_);
            updateOdomsAndPaths(current_frame);
        }
        is_initialized_ = true;
    }
    else
    {
        //// 1. check if keyframe
        if (checkIfKeyframe(current_frame, last_keyframe_))
        {
            // 2. if so, save
            {
                std::lock_guard<std::mutex> lock(keyframes_mutex_);
                last_keyframe_ = current_frame;
            }
            current_keyframe_idx_++;
            //// 3. vis
            {
                std::lock_guard<std::mutex> lock(vis_mutex_);
                updateOdomsAndPaths(current_frame);
            }
        }
    }
    return;
}

void FastLioLocalizationScQn::matchingTimerFunc()
{
    if (!is_initialized_)
    {
        return;
    }

    //// 1. copy not processed keyframes
    high_resolution_clock::time_point t1_ = high_resolution_clock::now();
    PosePcd last_keyframe_copy;
    {
        std::lock_guard<std::mutex> lock(keyframes_mutex_);
        if(last_keyframe_.processed_)
        {
            return; // already processed
        }
        last_keyframe_copy = last_keyframe_;
        last_keyframe_.processed_ = true;
    }
    if (last_keyframe_copy.idx_ == 0)
    {
        return; // initial keyframe
    }

    //// 2. detect match and calculate TF
    // from last_keyframe_copy keyframe to map (saved keyframes) in threshold radius, get the closest keyframe
    int closest_keyframe_idx = map_matcher_->fetchClosestKeyframeIdx(last_keyframe_copy, saved_map_from_bag_);
    if (closest_keyframe_idx < 0)
    {
        return; // if no matched candidate
    }
    // Quatro + NANO-GICP to check match (from current_keyframe to closest keyframe in saved map)
    const RegistrationOutput &reg_output = map_matcher_->performMapMatcher(last_keyframe_copy,
                                                                           saved_map_from_bag_,
                                                                           closest_keyframe_idx);

    //// 3. handle corrected results
    if (reg_output.is_valid_) // TF the pose with the result of match
    {
        RCLCPP_INFO(this->get_logger(), "\033[1;32mMap matching accepted. Score: %.3f\033[0m", reg_output.score_);
        last_corrected_TF_ = reg_output.pose_between_eig_ * last_corrected_TF_; // update TF
        Eigen::Matrix4d TFed_pose = reg_output.pose_between_eig_ * last_keyframe_copy.pose_corrected_eig_;
        // correct poses in vis data
        {
            std::lock_guard<std::mutex> lock(vis_mutex_);
            corrected_odoms_.points[last_keyframe_copy.idx_] = pcl::PointXYZ(TFed_pose(0, 3), TFed_pose(1, 3), TFed_pose(2, 3));
            corrected_odom_path_.poses[last_keyframe_copy.idx_] = poseEigToPoseStamped(TFed_pose, map_frame_);
        }
        // map matches
        matched_pairs_xyz_.push_back({corrected_odoms_.points[last_keyframe_copy.idx_], raw_odoms_.points[last_keyframe_copy.idx_]}); // for vis
        map_match_pub_->publish(getMatchMarker(matched_pairs_xyz_));
    }
    high_resolution_clock::time_point t2_ = high_resolution_clock::now();

    debug_src_pub_->publish(pclToPclRos(map_matcher_->getSourceCloud(), map_frame_, this->get_clock()->now()));
    debug_dst_pub_->publish(pclToPclRos(map_matcher_->getTargetCloud(), map_frame_, this->get_clock()->now()));
    debug_coarse_aligned_pub_->publish(pclToPclRos(map_matcher_->getCoarseAlignedCloud(), map_frame_, this->get_clock()->now()));
    debug_fine_aligned_pub_->publish(pclToPclRos(map_matcher_->getFinalAlignedCloud(), map_frame_, this->get_clock()->now()));

    // publish odoms and paths
    {
        std::lock_guard<std::mutex> lock(vis_mutex_);
        corrected_odom_pub_->publish(pclToPclRos(corrected_odoms_, map_frame_, this->get_clock()->now()));
        corrected_path_pub_->publish(corrected_odom_path_);
    }
    odom_pub_->publish(pclToPclRos(raw_odoms_, map_frame_, this->get_clock()->now()));
    path_pub_->publish(raw_odom_path_);

    high_resolution_clock::time_point t3_ = high_resolution_clock::now();
    RCLCPP_INFO(this->get_logger(), "Matching: %.1fms, vis: %.1fms",
             duration_cast<microseconds>(t2_ - t1_).count() / 1e3,
             duration_cast<microseconds>(t3_ - t2_).count() / 1e3);
    return;
}

void FastLioLocalizationScQn::updateOdomsAndPaths(const PosePcd &pose_pcd_in)
{
    raw_odoms_.points.emplace_back(pose_pcd_in.pose_eig_(0, 3),
                                   pose_pcd_in.pose_eig_(1, 3),
                                   pose_pcd_in.pose_eig_(2, 3));
    corrected_odoms_.points.emplace_back(pose_pcd_in.pose_corrected_eig_(0, 3),
                                         pose_pcd_in.pose_corrected_eig_(1, 3),
                                         pose_pcd_in.pose_corrected_eig_(2, 3));
    raw_odom_path_.poses.emplace_back(poseEigToPoseStamped(pose_pcd_in.pose_eig_, map_frame_));
    corrected_odom_path_.poses.emplace_back(poseEigToPoseStamped(pose_pcd_in.pose_corrected_eig_, map_frame_));
    return;
}

visualization_msgs::msg::Marker FastLioLocalizationScQn::getMatchMarker(const std::vector<std::pair<pcl::PointXYZ, pcl::PointXYZ>> &match_xyz_pairs)
{
    visualization_msgs::msg::Marker edges_;
    edges_.type = 5u;
    edges_.scale.x = 0.2f;
    edges_.header.frame_id = map_frame_;
    edges_.pose.orientation.w = 1.0f;
    edges_.color.r = 1.0f;
    edges_.color.g = 1.0f;
    edges_.color.b = 1.0f;
    edges_.color.a = 1.0f;
    for (size_t i = 0; i < match_xyz_pairs.size(); ++i)
    {
        geometry_msgs::msg::Point p_, p2_;
        p_.x = match_xyz_pairs[i].first.x;
        p_.y = match_xyz_pairs[i].first.y;
        p_.z = match_xyz_pairs[i].first.z;
        p2_.x = match_xyz_pairs[i].second.x;
        p2_.y = match_xyz_pairs[i].second.y;
        p2_.z = match_xyz_pairs[i].second.z;
        edges_.points.push_back(p_);
        edges_.points.push_back(p2_);
    }
    return edges_;
}

bool FastLioLocalizationScQn::checkIfKeyframe(const PosePcd &pose_pcd_in, const PosePcd &latest_pose_pcd)
{
    double trans_dist = (latest_pose_pcd.pose_corrected_eig_.block<3, 1>(0, 3) - pose_pcd_in.pose_corrected_eig_.block<3, 1>(0, 3)).norm();

    if (trans_dist > keyframe_dist_thr_)
    {
        return true;
    }

    if (enable_rotation_check_ && trans_dist < in_place_translation_threshold_)
    {
        Eigen::Matrix3d last_rot = latest_pose_pcd.pose_corrected_eig_.block<3, 3>(0, 0);
        Eigen::Matrix3d current_rot = pose_pcd_in.pose_corrected_eig_.block<3, 3>(0, 0);

        Eigen::Matrix3d relative_rot = last_rot.transpose() * current_rot;

        Eigen::AngleAxisd angle_axis(relative_rot);

        if (std::abs(angle_axis.angle()) > keyframe_rotation_threshold_rad_)
        {
            return true;
        }
    }

    return false;
}

void FastLioLocalizationScQn::loadMap(const std::string &saved_map_path)
{
    rosbag2_cpp::Reader bag_reader;
    bag_reader.open(saved_map_path);

    std::vector<sensor_msgs::msg::PointCloud2> load_pcd_vec;
    std::vector<geometry_msgs::msg::PoseStamped> load_pose_vec;

    while (bag_reader.has_next()) {
        auto bag_message = bag_reader.read_next();
        if (bag_message->topic_name == "/keyframe_pcd") {
            sensor_msgs::msg::PointCloud2 msg;
            rclcpp::SerializedMessage serialized_msg(*bag_message->serialized_data);
            rclcpp::Serialization<sensor_msgs::msg::PointCloud2> serializer;
            serializer.deserialize_message(&serialized_msg, &msg);
            load_pcd_vec.push_back(msg);
        } else if (bag_message->topic_name == "/keyframe_pose") {
            geometry_msgs::msg::PoseStamped msg;
            rclcpp::SerializedMessage serialized_msg(*bag_message->serialized_data);
            rclcpp::Serialization<geometry_msgs::msg::PoseStamped> serializer;
            serializer.deserialize_message(&serialized_msg, &msg);
            load_pose_vec.push_back(msg);
        }
    }


    if (load_pcd_vec.size() != load_pose_vec.size())
    {
        RCLCPP_ERROR(this->get_logger(), "WRONG BAG FILE!!!!!");
    }

    for (size_t i = 0; i < load_pose_vec.size(); ++i)
    {
        saved_map_from_bag_.push_back(PosePcdReduced(load_pose_vec[i], load_pcd_vec[i], i));
        saved_map_pcd_ += transformPcd(saved_map_from_bag_[i].pcd_, saved_map_from_bag_[i].pose_eig_);
        map_matcher_->updateScancontext(saved_map_from_bag_[i].pcd_); // note: update scan context for loop candidate detection

	// Populate map path
        auto pose_stamped = poseEigToPoseStamped(saved_map_from_bag_[i].pose_eig_, map_frame_);
        map_path_.poses.push_back(pose_stamped);
    }
    saved_map_pcd_ = *voxelizePcd(saved_map_pcd_, voxel_res_);

    return;
}
