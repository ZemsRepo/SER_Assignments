#include "utils.h"

namespace Utils {

void fromROSMsg(const solution04::MyImu::ConstPtr &msg, Imu::Ptr imu) {}

void fromROSMsg(const solution04::MyPose::ConstPtr &msg, Pose::Ptr pose) {}

void fromROSMsg(const solution04::ImgPts::ConstPtr &msg, ImgPts::Ptr imgPts) {}

void publish_trajectory(const ros::Publisher &pub, const Eigen::Matrix3d &C,
                        const Eigen::Vector3d &r, const ros::Time &time) {
    Eigen::Quaterniond Q(C);

    static nav_msgs::Path gtTraj;
    gtTraj.header.frame_id = "world";

    geometry_msgs::PoseStamped gtPoseMsg;
    gtPoseMsg.header.stamp = time;
    gtPoseMsg.header.frame_id = "world";
    gtPoseMsg.pose.position.x = r.x();
    gtPoseMsg.pose.position.y = r.y();
    gtPoseMsg.pose.position.z = r.z();
    gtPoseMsg.pose.orientation.x = Q.inverse().x();
    gtPoseMsg.pose.orientation.y = Q.inverse().y();
    gtPoseMsg.pose.orientation.z = Q.inverse().z();
    gtPoseMsg.pose.orientation.w = Q.inverse().w();
    gtTraj.poses.push_back(gtPoseMsg);
    pub.publish(gtTraj);
}

void publishPointCloud(const ros::Publisher &pub, const Landmark3DPts &landmarks,
                       const ros::Time &time, const std::string &frame_id) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    cloud->width = 20;
    cloud->height = 1;
    cloud->is_dense = false;
    cloud->points.resize(cloud->width * cloud->height);
    cloud->header.frame_id = frame_id;

    for (int i = 0; i < 20; i++) {
        cloud->points[i].x = landmarks.row(i).x();
        cloud->points[i].y = landmarks.row(i).y();
        cloud->points[i].z = landmarks.row(i).z();
    }

    sensor_msgs::PointCloud2 cloudMsg;
    pcl::toROSMsg(*cloud, cloudMsg);
    cloudMsg.header.stamp = time;
    pub.publish(cloudMsg);
}

void publishPointCloud(const ros::Publisher &pubLeft, const ros::Publisher &pubRight,
                       const Landmark3DPts &landmarks, const Eigen::Matrix<double, 20, 4> &imgPts,
                       const ros::Time &time, const std::string &frame_id) {
    // publish left visible pcl
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudLeft(new pcl::PointCloud<pcl::PointXYZ>);
    cloudLeft->width = 20;
    cloudLeft->height = 1;
    cloudLeft->is_dense = false;
    cloudLeft->points.resize(cloudLeft->width * cloudLeft->height);
    cloudLeft->header.frame_id = frame_id;

    for (int i = 0; i < 20; i++) {
        if (imgPts(i, 0) == -1) continue;
        cloudLeft->points[i].x = landmarks.row(i).x();
        cloudLeft->points[i].y = landmarks.row(i).y();
        cloudLeft->points[i].z = landmarks.row(i).z();
    }

    sensor_msgs::PointCloud2 leftCloudMsg;
    pcl::toROSMsg(*cloudLeft, leftCloudMsg);
    leftCloudMsg.header.stamp = time;
    pubLeft.publish(leftCloudMsg);

    // publish right visible pcl
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudRight(new pcl::PointCloud<pcl::PointXYZ>);
    cloudRight->width = 20;
    cloudRight->height = 1;
    cloudRight->is_dense = false;
    cloudRight->points.resize(cloudRight->width * cloudRight->height);
    cloudRight->header.frame_id = frame_id;

    for (int i = 0; i < 20; i++) {
        if (imgPts(i, 2) == -1) continue;
        cloudRight->points[i].x = landmarks.row(i).x();
        cloudRight->points[i].y = landmarks.row(i).y();
        cloudRight->points[i].z = landmarks.row(i).z();
    }

    sensor_msgs::PointCloud2 rightCloudMsg;
    pcl::toROSMsg(*cloudRight, rightCloudMsg);
    rightCloudMsg.header.stamp = time;
    pubRight.publish(rightCloudMsg);
}

void publishImage(const ros::Publisher &leftPub, const ros::Publisher &rightPub,
                  const Eigen::Matrix<double, 20, 4> &imgPts, const ros::Time &time) {
    cv::Mat imgLeft = cv::Mat::zeros(cv::Size(imgWidth, imgHeight), CV_8UC3);
    cv::Mat imgRight = cv::Mat::zeros(cv::Size(imgWidth, imgHeight), CV_8UC3);
    for (int i = 0; i < 20; i++) {
        if (imgPts(i, 0) != -1) {
            cv::circle(imgLeft, cv::Point(imgPts(i, 0), imgPts(i, 1)), 5, cv::Scalar(0, 0, 164),
                       -1);
            cv::putText(imgLeft, std::to_string(i + 1), cv::Point(imgPts(i, 0), imgPts(i, 1)),
                        cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(225, 225, 225), 1, cv::LINE_AA);
        }
        if (imgPts(i, 2) != -1) {
            cv::circle(imgRight, cv::Point(imgPts(i, 2), imgPts(i, 3)), 5, cv::Scalar(135, 74, 32),
                       -1);
            cv::putText(imgRight, std::to_string(i + 1), cv::Point(imgPts(i, 2), imgPts(i, 3)),
                        cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(225, 225, 225), 1, cv::LINE_AA);
        }
    }
    auto header = std_msgs::Header();
    header.stamp = time;
    header.frame_id = "camera";
    sensor_msgs::ImagePtr imgMsgLeft = cv_bridge::CvImage(header, "bgr8", imgLeft).toImageMsg();
    sensor_msgs::ImagePtr imgMsgRight = cv_bridge::CvImage(header, "bgr8", imgRight).toImageMsg();
    leftPub.publish(imgMsgLeft);
    rightPub.publish(imgMsgRight);
}

void publishMarkerArray(const ros::Publisher &pub, const Landmark3DPts &landmarks,
                        const Eigen::Matrix<double, 20, 4> &imgPts, const ros::Time &time,
                        const std::string &frame_id) {
    static visualization_msgs::MarkerArray marker_array;
    marker_array.markers.resize(20);
    marker_array.markers.clear();

    for (int i = 0; i < 20; i++) {
        visualization_msgs::Marker marker;
        marker.header.frame_id = frame_id;
        marker.header.stamp = time;
        marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position.x = landmarks.row(i).x();
        marker.pose.position.y = landmarks.row(i).y();
        marker.pose.position.z = landmarks.row(i).z() - 0.1;
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;
        marker.scale.z = 0.05;
        marker.color.a = 1.0;
        marker.color.r = 1.0;
        marker.color.g = 1.0;
        marker.color.b = 1.0;
        marker.id = i;
        marker.lifetime = ros::Duration();
        std::stringstream ss;
        ss << i + 1;
        marker.text = imgPts(i, 0) == -1 ? " " : ss.str();
        marker_array.markers.push_back(marker);
    }
    pub.publish(marker_array);
}

void publishMarkerArray(const ros::Publisher &pub, const Landmark3DPts &landmarks,
                        const ros::Time &time, const std::string &frame_id) {
    static visualization_msgs::MarkerArray marker_array;
    marker_array.markers.resize(20);
    marker_array.markers.clear();

    for (int i = 0; i < 20; i++) {
        visualization_msgs::Marker marker;
        marker.header.frame_id = frame_id;
        marker.header.stamp = time;
        marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position.x = landmarks.row(i).x();
        marker.pose.position.y = landmarks.row(i).y();
        marker.pose.position.z = landmarks.row(i).z() - 0.1;
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;
        marker.scale.z = 0.05;
        marker.color.a = 1.0;
        marker.color.r = 1.0;
        marker.color.g = 1.0;
        marker.color.b = 1.0;
        marker.ns = frame_id;
        marker.id = i;
        marker.lifetime = ros::Duration();
        std::stringstream ss;
        ss << i + 1;
        marker.text = ss.str();
        marker_array.markers.push_back(marker);
    }
    pub.publish(marker_array);
}

void broadcastWorld2VehTF(tf2_ros::TransformBroadcaster &br, const Eigen::Matrix3d &C,
                          const Eigen::Vector3d &r, const ros::Time &time) {
    Eigen::Quaterniond Q(C);

    geometry_msgs::TransformStamped world2VehTrans;
    world2VehTrans.header.stamp = time;
    world2VehTrans.header.frame_id = "world";
    world2VehTrans.child_frame_id = "vehicle";
    world2VehTrans.transform.translation.x = r.x();
    world2VehTrans.transform.translation.y = r.y();
    world2VehTrans.transform.translation.z = r.z();
    world2VehTrans.transform.rotation.x = Q.inverse().x();
    world2VehTrans.transform.rotation.y = Q.inverse().y();
    world2VehTrans.transform.rotation.z = Q.inverse().z();
    world2VehTrans.transform.rotation.w = Q.inverse().w();
    br.sendTransform(world2VehTrans);
}

void broadcastStaticVeh2CamTF(tf2_ros::StaticTransformBroadcaster &staticBr,
                              const Eigen::Matrix3d &C_c_v, const Eigen::Vector3d &rho_v_c_v,
                              const ros::Time &time) {
    static Eigen::Quaterniond Q(C_c_v);

    static geometry_msgs::TransformStamped veh2CamTrans;
    veh2CamTrans.header.stamp = time;
    veh2CamTrans.header.frame_id = "vehicle";
    veh2CamTrans.child_frame_id = "camera";
    veh2CamTrans.transform.translation.x = rho_v_c_v.x();
    veh2CamTrans.transform.translation.y = rho_v_c_v.y();
    veh2CamTrans.transform.translation.z = rho_v_c_v.z();
    veh2CamTrans.transform.rotation.x = Q.inverse().x();
    veh2CamTrans.transform.rotation.y = Q.inverse().y();
    veh2CamTrans.transform.rotation.z = Q.inverse().z();
    veh2CamTrans.transform.rotation.w = Q.inverse().w();
    staticBr.sendTransform(veh2CamTrans);
}

Eigen::Matrix3d skewSymmetric(const Eigen::Vector3d &v) {
    Eigen::Matrix3d skewSymmetric;
    skewSymmetric << 0, -v(2), v(1), v(2), 0, -v(0), -v(1), v(0), 0;
    return skewSymmetric;
}

void vec2rotMat(const Eigen::Vector3d &v, Eigen::Matrix3d &rotMat) {
    auto &&v_norm = v.norm();
    auto &I = Eigen::Matrix3d::Identity();
    rotMat = cos(v_norm) * I + (1 - cos(v_norm)) * (v / v_norm) * (v / v_norm).transpose() -
             sin(v_norm) * skewSymmetric(v / v_norm);
}
}  // namespace Utils