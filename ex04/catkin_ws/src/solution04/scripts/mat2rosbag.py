#!/usr/bin/env python3

import rospy
import rosbag
from solution04.msg import Pixel, SteroLandmarks, MyImu, MyPose
from scipy.io import loadmat
import yaml

pkg_path = "src/solution04/assects/"
mat_file = pkg_path + "dataset3.mat"
param_file = pkg_path + "params.yaml"
bag_file = pkg_path + "dataset3.bag"
mat = loadmat(mat_file)

# time
t = mat["t"].reshape(-1).tolist()  # (1, 1900) time

# ground truth
theta_vk_i = mat["theta_vk_i"].T.tolist()  # (3, 1900) gt axis angle
r_i_vk_i = mat["r_i_vk_i"].T.tolist()  # (3, 1900) gt translation

# imu
w_vk_vk_i = mat["w_vk_vk_i"].T.tolist()  # (3, 1900) imu angular velocity
w_var = (
    mat["w_var"]
    .reshape(
        3,
    )
    .tolist()
)  # (3, 1) imu angular velocity variance

v_vk_vk_i = mat["v_vk_vk_i"].T.tolist()  # (3, 1900) imu linear velocity
v_var = (
    mat["v_var"]
    .reshape(
        3,
    )
    .tolist()
)  # (3, 1) imu linear velocity variance

# landmark
rho_i_pj_i = mat["rho_i_pj_i"].T.tolist()  # (3, 20) landmark position

# stereo camera
y_k_j = (
    mat["y_k_j"].reshape(4, 20, -1).T.tolist()
)  # (4, 1900, 20) stereo camera observation
y_var = (
    mat["y_var"]
    .reshape(
        4,
    )
    .tolist()
)  # (4, 1) stereo camera observation variance

# extrinsics
C_c_v = mat["C_c_v"].tolist()  # (3, 3) rotation vehicle -> frame
rho_v_c_v = (
    mat["rho_v_c_v"]
    .reshape(
        3,
    )
    .tolist()
)  # (3, 1) translation vehicle -> frame

# camera intrinsics
fu = mat["fu"].item()  # (1, 1) horizontal focal length
fv = mat["fv"].item()  # (1, 1) vertical focal length
cu = mat["cu"].item()  # (1, 1) horizontal optical center
cv = mat["cv"].item()  # (1, 1) vertical optical center
b = mat["b"].item()  # (1, 1) stereo baseline


def write_params(file):
    params = {
        "extrinsics": {"C_c_v": C_c_v, "rho_v_c_v": rho_v_c_v},
        "intrinsics": {"fu": fu, "fv": fv, "cu": cu, "cv": cv, "b": b},
        "rho_i_pj_i": rho_i_pj_i,
    }
    with open(file, "w") as f:
        yaml.dump(params, f)


def main():

    rospy.init_node("mat2rosbag", anonymous=True)
    bag = rosbag.Bag(bag_file, "w")
    time_now = rospy.Time.now()

    # write parameters
    write_params(param_file)

    # write to rosbag
    for k in range(len(t)):
        # timestamp
        timestamp = time_now + rospy.Duration.from_sec(t[k])

        # ground truth
        gt_pose_msg = MyPose()
        ## header
        gt_pose_msg.header.frame_id = "base_link"
        gt_pose_msg.header.stamp = timestamp
        ## rotation
        gt_pose_msg.theta.x = theta_vk_i[k][0]
        gt_pose_msg.theta.y = theta_vk_i[k][1]
        gt_pose_msg.theta.z = theta_vk_i[k][2]
        ## translation
        gt_pose_msg.r.x = r_i_vk_i[k][0]
        gt_pose_msg.r.y = r_i_vk_i[k][1]
        gt_pose_msg.r.z = r_i_vk_i[k][2]

        # imu
        imu_msg = MyImu()
        ## header
        imu_msg.header.frame_id = "imu_link"
        imu_msg.header.stamp = timestamp
        ## angular velocity
        imu_msg.angular_velocity.x = w_vk_vk_i[k][0]
        imu_msg.angular_velocity.y = w_vk_vk_i[k][1]
        imu_msg.angular_velocity.z = w_vk_vk_i[k][2]
        ## linear velocity
        imu_msg.linear_velocity.x = v_vk_vk_i[k][0]
        imu_msg.linear_velocity.y = v_vk_vk_i[k][1]
        imu_msg.linear_velocity.z = v_vk_vk_i[k][2]

        # stereo camera
        stero_cam_msg = SteroLandmarks()
        ## header
        stero_cam_msg.header.frame_id = "base_link"
        stero_cam_msg.header.stamp = timestamp
        ## landmarks
        left_points = [Pixel(round(p[0]), round(p[1])) for p in y_k_j[k]]
        right_points = [Pixel(round(p[2]), round(p[3])) for p in y_k_j[k]]
        stero_cam_msg.left = left_points
        stero_cam_msg.right = right_points

        # rospy.loginfo(f"ground_truth: {gt_pose_msg}")
        # rospy.loginfo(f"imu: {imu_msg}")
        # rospy.loginfo(f"stereo_landmarks: {stero_cam_msg}")

        # save to rosbag
        bag.write("/ground_truth", gt_pose_msg, timestamp)
        bag.write("/imu", imu_msg, timestamp)
        bag.write("/stereo_landmarks", stero_cam_msg, timestamp)

    bag.close()
    rospy.on_shutdown(lambda: print(f"{mat_file} converted to {bag_file}. \n{param_file} created."))


if __name__ == "__main__":
    main()
