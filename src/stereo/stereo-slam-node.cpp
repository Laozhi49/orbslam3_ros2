#include "stereo-slam-node.hpp"

#include<opencv2/core/core.hpp>

using std::placeholders::_1;
using std::placeholders::_2;

StereoSlamNode::StereoSlamNode(ORB_SLAM3::System* pSLAM, const string &strSettingsFile, const string &strDoRectify)
:   Node("ORB_SLAM3_ROS2"),
    m_SLAM(pSLAM)
{
    stringstream ss(strDoRectify);
    ss >> boolalpha >> doRectify;

    if (doRectify){

        cv::FileStorage fsSettings(strSettingsFile, cv::FileStorage::READ);
        if(!fsSettings.isOpened()){
            cerr << "ERROR: Wrong path to settings" << endl;
            assert(0);
        }

        cv::Mat K_l, K_r, P_l, P_r, R_l, R_r, D_l, D_r;
        fsSettings["LEFT.K"] >> K_l;
        fsSettings["RIGHT.K"] >> K_r;

        fsSettings["LEFT.P"] >> P_l;
        fsSettings["RIGHT.P"] >> P_r;

        fsSettings["LEFT.R"] >> R_l;
        fsSettings["RIGHT.R"] >> R_r;

        fsSettings["LEFT.D"] >> D_l;
        fsSettings["RIGHT.D"] >> D_r;

        int rows_l = fsSettings["LEFT.height"];
        int cols_l = fsSettings["LEFT.width"];
        int rows_r = fsSettings["RIGHT.height"];
        int cols_r = fsSettings["RIGHT.width"];

        if(K_l.empty() || K_r.empty() || P_l.empty() || P_r.empty() || R_l.empty() || R_r.empty() || D_l.empty() || D_r.empty() ||
                rows_l==0 || rows_r==0 || cols_l==0 || cols_r==0){
            cerr << "ERROR: Calibration parameters to rectify stereo are missing!" << endl;
            assert(0);
        }

        cv::initUndistortRectifyMap(K_l,D_l,R_l,P_l.rowRange(0,3).colRange(0,3),cv::Size(cols_l,rows_l),CV_32F,M1l,M2l);
        cv::initUndistortRectifyMap(K_r,D_r,R_r,P_r.rowRange(0,3).colRange(0,3),cv::Size(cols_r,rows_r),CV_32F,M1r,M2r);
    }

    left_sub = std::make_shared<message_filters::Subscriber<ImageMsg> >(shared_ptr<rclcpp::Node>(this), "camera/left");
    right_sub = std::make_shared<message_filters::Subscriber<ImageMsg> >(shared_ptr<rclcpp::Node>(this), "camera/right");

    pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("orbslam3/camera_pose", 10);
    map_points_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("orbslam3/map_points", 1);

    // 创建定时器，每秒发布一次点云
    map_pub_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(1000),
        std::bind(&StereoSlamNode::publishMapPoints, this)
    );

    syncApproximate = std::make_shared<message_filters::Synchronizer<approximate_sync_policy> >(approximate_sync_policy(10), *left_sub, *right_sub);
    syncApproximate->registerCallback(&StereoSlamNode::GrabStereo, this);
}

StereoSlamNode::~StereoSlamNode()
{
    // Stop all threads
    m_SLAM->Shutdown();

    // Save camera trajectory
    m_SLAM->SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");
}

void StereoSlamNode::GrabStereo(const ImageMsg::SharedPtr msgLeft, const ImageMsg::SharedPtr msgRight)
{
    // Copy the ros rgb image message to cv::Mat.
    try
    {
        cv_ptrLeft = cv_bridge::toCvShare(msgLeft);
    }
    catch (cv_bridge::Exception& e)
    {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        return;
    }

    // Copy the ros depth image message to cv::Mat.
    try
    {
        cv_ptrRight = cv_bridge::toCvShare(msgRight);
    }
    catch (cv_bridge::Exception& e)
    {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        return;
    }

    cv::Mat imLeft, imRight;
    if (doRectify){
        cv::remap(cv_ptrLeft->image,imLeft,M1l,M2l,cv::INTER_LINEAR);
        cv::remap(cv_ptrRight->image,imRight,M1r,M2r,cv::INTER_LINEAR);
        
    }
    else
    {
        imLeft = cv_ptrLeft->image;
        imRight = cv_ptrRight->image;
    }

    // 1. 跟踪，得到相机位姿
    // Sophus::SE3f Tcw_sophus  = m_SLAM->TrackStereo(cv_ptrLeft->image, cv_ptrRight->image, Utility::StampToSec(msgLeft->header.stamp));
    Sophus::SE3f Tcw_sophus  = m_SLAM->TrackStereo(imLeft, imRight, Utility::StampToSec(msgLeft->header.stamp));

    Eigen::Matrix4f Tcw = Tcw_sophus.matrix();  // 4x4 Eigen 矩阵
    
    Eigen::Matrix3f Rcw = Tcw.block<3,3>(0,0);
    Eigen::Vector3f tcw = Tcw.block<3,1>(0,3);
    Eigen::Matrix3f Rwc = Rcw.transpose();
    Eigen::Vector3f twc = -Rwc * tcw;

    // RCLCPP_INFO(this->get_logger(), "Camera Twc: x=%f y=%f z=%f", twc[0], twc[1], twc[2]);

    // ORB-SLAM3: X-right, Y-down, Z-forward
    // ROS:       X-forward, Y-left, Z-up
    Eigen::Matrix3f cv_to_ros;
    cv_to_ros << 0,  0, 1,
                -1, 0, 0,
                0, -1, 0;

    Eigen::Matrix3f R_ros = cv_to_ros * Rwc * cv_to_ros.transpose();
    Eigen::Vector3f t_ros = cv_to_ros * twc;

    // 发布位姿
    geometry_msgs::msg::PoseStamped pose_msg;
    pose_msg.header.stamp = msgLeft->header.stamp;
    pose_msg.header.frame_id = "map";

    pose_msg.pose.position.x = t_ros[0];
    pose_msg.pose.position.y = t_ros[1];
    pose_msg.pose.position.z = t_ros[2];

    Eigen::Quaternionf q(R_ros);
    pose_msg.pose.orientation.x = q.x();
    pose_msg.pose.orientation.y = q.y();
    pose_msg.pose.orientation.z = q.z();
    pose_msg.pose.orientation.w = q.w();
    pose_pub_->publish(pose_msg);

}

void StereoSlamNode::publishMapPoints()
{
    std::vector<ORB_SLAM3::MapPoint*> mps;
    RCLCPP_INFO(this->get_logger(), "GetTrackedMapPoints...");
    try {
        mps = m_SLAM->GetTrackedMapPoints();
    } catch (const std::exception &e) {
        RCLCPP_WARN(this->get_logger(), "GetTrackedMapPoints exception: %s", e.what());
        return;
    } catch (...) {
        RCLCPP_WARN(this->get_logger(), "GetTrackedMapPoints unknown exception");
        return;
    }
    
    if (mps.empty()) return;

    // ORB-SLAM3: X-right, Y-down, Z-forward
    // ROS:       X-forward, Y-left, Z-up
    Eigen::Matrix3f cv_to_ros;
    cv_to_ros << 0,  0, 1,
                -1, 0, 0,
                0, -1, 0;

    // copy position
    std::vector<Eigen::Vector3f> pts;
    pts.reserve(mps.size());
    for (auto p : mps)
    {
        if (!p || p->isBad()) continue;

        Eigen::Vector3f pos;
        try {
            pos = p->GetWorldPos(); // 内部线程安全
            RCLCPP_INFO(this->get_logger(), "MapPoint: x=%f y=%f z=%f", pos[0], pos[1], pos[2]);
        } catch (...) {
            continue;
        }

        if (!std::isfinite(pos[0]) || !std::isfinite(pos[1]) || !std::isfinite(pos[2]))
            continue;

        // 转换到 ROS 坐标系
        Eigen::Vector3f pos_ros = cv_to_ros * pos;
        pts.push_back(pos_ros);
    }

    if (pts.empty()) return;

    RCLCPP_INFO(this->get_logger(), "pts_size = %d", pts.size());

    // 累加当前帧点到全局集合
    for(const auto &p : pts) {
        if(std::isfinite(p[0]) && std::isfinite(p[1]) && std::isfinite(p[2])) {
            global_pts_.insert(p);  // 安全去重
        }
    }

    std::vector<Eigen::Vector3f> pts_global(global_pts_.begin(), global_pts_.end());

    sensor_msgs::msg::PointCloud2 msg;
    msg.header.stamp = rclcpp::Clock().now();
    msg.header.frame_id = "map";

    msg.height = 1;               // 非组织点云
    msg.width = global_pts_.size();
    msg.is_dense = false;
    msg.is_bigendian = false;

    // 每个点有 x, y, z，每个 float 4 字节
    msg.point_step = 3 * sizeof(float);
    msg.row_step = msg.point_step * msg.width;
    msg.data.resize(msg.row_step);

    // 定义字段
    msg.fields.resize(3);
    msg.fields[0].name = "x";
    msg.fields[0].offset = 0;
    msg.fields[0].datatype = sensor_msgs::msg::PointField::FLOAT32;
    msg.fields[0].count = 1;

    msg.fields[1].name = "y";
    msg.fields[1].offset = sizeof(float);
    msg.fields[1].datatype = sensor_msgs::msg::PointField::FLOAT32;
    msg.fields[1].count = 1;

    msg.fields[2].name = "z";
    msg.fields[2].offset = 2 * sizeof(float);
    msg.fields[2].datatype = sensor_msgs::msg::PointField::FLOAT32;
    msg.fields[2].count = 1;

    // 填充点数据
    for (size_t i = 0; i < pts_global.size(); ++i)
    {
        std::memcpy(&msg.data[i * msg.point_step + 0], &pts_global[i][0], sizeof(float));
        std::memcpy(&msg.data[i * msg.point_step + 4], &pts_global[i][1], sizeof(float));
        std::memcpy(&msg.data[i * msg.point_step + 8], &pts_global[i][2], sizeof(float));
    }
    // 发布
    map_points_pub_->publish(msg);

    RCLCPP_INFO(this->get_logger(), "GetTrackedMapPoints finish");

}
