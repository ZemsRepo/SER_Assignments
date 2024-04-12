#include "utils.h"

namespace Utils {

void publish_trajectory(const ros::Publisher &pub, nav_msgs::Path &traj, const Eigen::Matrix3d &C,
                        const Eigen::Vector3d &r, const ros::Time &time) {
    Eigen::Quaterniond Q(C.transpose());
    traj.header.frame_id = "world";
    geometry_msgs::PoseStamped gtPoseMsg;
    gtPoseMsg.header.stamp = time;
    gtPoseMsg.header.frame_id = "world";
    gtPoseMsg.pose.position.x = r.x();
    gtPoseMsg.pose.position.y = r.y();
    gtPoseMsg.pose.position.z = r.z();
    gtPoseMsg.pose.orientation.x = Q.x();
    gtPoseMsg.pose.orientation.y = Q.y();
    gtPoseMsg.pose.orientation.z = Q.z();
    gtPoseMsg.pose.orientation.w = Q.w();
    traj.poses.push_back(gtPoseMsg);
    pub.publish(traj);
}

void publishPointCloud(const ros::Publisher &pub, const Eigen::Matrix<double, 20, 3> &landmarks,
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

void publishPointCloud(const ros::Publisher &pub, const Eigen::Matrix<double, 20, 3> &landmarks,
                       const Eigen::Matrix<double, 20, 2> &imgPtsNomo, const ros::Time &time,
                       const std::string &frame_id) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    cloud->width = 20;
    cloud->height = 1;
    cloud->is_dense = false;
    cloud->points.resize(cloud->width * cloud->height);
    cloud->header.frame_id = frame_id;

    for (int i = 0; i < 20; i++) {
        if (imgPtsNomo(i, 0) == -1) continue;
        cloud->points[i].x = landmarks.row(i).x();
        cloud->points[i].y = landmarks.row(i).y();
        cloud->points[i].z = landmarks.row(i).z();
    }

    sensor_msgs::PointCloud2 cloudMsg;
    pcl::toROSMsg(*cloud, cloudMsg);
    cloudMsg.header.stamp = time;
    pub.publish(cloudMsg);
}

bool imgPtIsVisible(const Eigen::Vector2d &imgPt) {
    return imgPt.x() >= 0 && imgPt.x() < imgWidth && imgPt.y() >= 0 && imgPt.y() < imgHeight;
}

void publishImage(const ros::Publisher &pub, const Eigen::Matrix<double, 20, 2> &imgPtsMono,
                  const cv::Scalar &color, const ros::Time &time, const std::string &frame_id) {
    cv::Mat img = cv::Mat::zeros(cv::Size(imgWidth, imgHeight), CV_8UC3);
    for (int i = 0; i < 20; i++) {
        if (!imgPtIsVisible(imgPtsMono.row(i))) continue;
        cv::circle(img, cv::Point(imgPtsMono(i, 0), imgPtsMono(i, 1)), 5, color, -1);
        cv::putText(img, std::to_string(i + 1), cv::Point(imgPtsMono(i, 0), imgPtsMono(i, 1)),
                    cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(225, 225, 225), 1, cv::LINE_AA);
    }
    auto header = std_msgs::Header();
    header.stamp = time;
    header.frame_id = "camera";
    sensor_msgs::ImagePtr imgMsg = cv_bridge::CvImage(header, "bgr8", img).toImageMsg();
    pub.publish(imgMsg);
}

void publishImage(const ros::Publisher &pub, const Eigen::Matrix<double, 20, 2> &imgPtsMono,
                  const cv::Scalar &color, const Eigen::Matrix<double, 20, 2> &imgPtsObsModel,
                  const cv::Scalar &colorObsModel, const ros::Time &time,
                  const std::string &frame_id) {
    cv::Mat img = cv::Mat::zeros(cv::Size(imgWidth, imgHeight), CV_8UC3);
    for (int i = 0; i < 20; i++) {
        if (!imgPtIsVisible(imgPtsMono.row(i))) continue;
        cv::circle(img, cv::Point(imgPtsMono(i, 0), imgPtsMono(i, 1)), 5, color, -1);
        cv::putText(img, std::to_string(i + 1), cv::Point(imgPtsMono(i, 0), imgPtsMono(i, 1)),
                    cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(225, 225, 225), 1, cv::LINE_AA);

        if (!imgPtIsVisible(imgPtsObsModel.row(i))) continue;
        cv::circle(img, cv::Point(imgPtsObsModel(i, 0), imgPtsObsModel(i, 1)), 5, colorObsModel,
                   -1);
        cv::putText(img, std::to_string(i + 1),
                    cv::Point(imgPtsObsModel(i, 0), imgPtsObsModel(i, 1)), cv::FONT_HERSHEY_SIMPLEX,
                    0.5, cv::Scalar(225, 225, 225), 1, cv::LINE_AA);
    }

    auto header = std_msgs::Header();
    header.stamp = time;
    header.frame_id = "camera";
    sensor_msgs::ImagePtr imgMsg = cv_bridge::CvImage(header, "bgr8", img).toImageMsg();
    pub.publish(imgMsg);
}

void publishMarkerArray(const ros::Publisher &pub, const Eigen::Matrix<double, 20, 3> &landmarks,
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

void publishMarker(const ros::Publisher &pub, const ros::Time &time, const int frameIdx,
                   const std::string &frame_id) {
    visualization_msgs::Marker marker;
    marker.header.frame_id = frame_id;
    marker.header.stamp = time;
    marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = 3.5;
    marker.pose.position.y = 1.5;
    marker.pose.position.z = 0.1;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.z = 0.05;
    marker.color.a = 1.0;
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    marker.ns = frame_id;
    marker.id = 100;
    marker.lifetime = ros::Duration();
    std::stringstream ss;
    ss << "frame: " << frameIdx + 1;
    marker.text = ss.str();
    pub.publish(marker);
}

void broadcastWorld2VehTF(tf2_ros::TransformBroadcaster &br, const Eigen::Matrix3d &C,
                          const Eigen::Vector3d &r, const ros::Time &time,
                          const std::string &child_frame_id) {
    Eigen::Quaterniond Q(C.transpose());

    geometry_msgs::TransformStamped world2VehTrans;
    world2VehTrans.header.stamp = time;
    world2VehTrans.header.frame_id = "world";
    world2VehTrans.child_frame_id = child_frame_id;
    world2VehTrans.transform.translation.x = r.x();
    world2VehTrans.transform.translation.y = r.y();
    world2VehTrans.transform.translation.z = r.z();
    world2VehTrans.transform.rotation.x = Q.x();
    world2VehTrans.transform.rotation.y = Q.y();
    world2VehTrans.transform.rotation.z = Q.z();
    world2VehTrans.transform.rotation.w = Q.w();
    br.sendTransform(world2VehTrans);
}

void broadcastStaticVeh2CamTF(tf2_ros::StaticTransformBroadcaster &staticBr,
                              const Eigen::Matrix3d &C_c_v, const Eigen::Vector3d &rho_v_c_v,
                              const ros::Time &time) {
    static Eigen::Quaterniond Q(C_c_v.transpose());

    static geometry_msgs::TransformStamped veh2CamTrans;
    veh2CamTrans.header.stamp = time;
    veh2CamTrans.header.frame_id = "vehicle";
    veh2CamTrans.child_frame_id = "camera";
    veh2CamTrans.transform.translation.x = rho_v_c_v.x();
    veh2CamTrans.transform.translation.y = rho_v_c_v.y();
    veh2CamTrans.transform.translation.z = rho_v_c_v.z();
    veh2CamTrans.transform.rotation.x = Q.x();
    veh2CamTrans.transform.rotation.y = Q.y();
    veh2CamTrans.transform.rotation.z = Q.z();
    veh2CamTrans.transform.rotation.w = Q.w();
    staticBr.sendTransform(veh2CamTrans);
}
}  // namespace Utils