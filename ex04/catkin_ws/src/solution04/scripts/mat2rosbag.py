#!/usr/bin/env python

import rospy
import rosbag
from sensor_msgs.msg import Imu
from geometry_msgs.msg import PoseStamped
import scipy as sp
import yaml

pkg_path = "ex04/catkin_ws/src/solution04/assects/"
file = pkg_path + "dataset3.mat"
param_file = pkg_path + "params.yaml"
bag_file = pkg_path + "dataset3.bag"
mat = sp.io.loadmat(file)

# time
t = mat["t"]  # (1, 1900) time

# ground truth
theta_vk_i = mat["theta_vk_i"]  # (3, 1900) gt axis angle
r_i_vk_i = mat["r_i_vk_i"]  # (3, 1900) gt translation

# imu
w_vk_vk_i = mat["w_vk_vk_i"]  # (3, 1900) imu angular velocity
w_var = mat["w_var"]  # (3, 1) imu angular velocity variance

v_vk_vk_i = mat["v_vk_vk_i"]  # (3, 1900) imu linear velocity
v_var = mat["v_var"]  # (3, 1) imu linear velocity variance

# landmark
rho_i_pj_i = mat["rho_i_pj_i"]  # (3, 20) landmark position

# stereo camera
y_k_j = mat["y_k_j"]  # (4, 1900, 20) stereo camera observation
y_var = mat["y_var"]  # (4, 1) stereo camera observation variance

# extrinsics
C_c_v = mat["C_c_v"]  # (3, 3) rotation vehicle -> frame
rho_v_c_v = mat["rho_v_c_v"]  # (3, 1) translation vehicle -> frame

# camera intrinsics
fu = mat["fu"]  # (1, 1) horizontal focal length
fv = mat["fv"]  # (1, 1) vertical focal length
cu = mat["cu"]  # (1, 1) horizontal optical center
cv = mat["cv"]  # (1, 1) vertical optical center
b = mat["b"]  # (1, 1) stereo baseline

def write_params(file):
    params = {        
        "extrinsics": {
            "C_c_v": C_c_v.tolist(),
            "rho_v_c_v": rho_v_c_v.reshape(3,).tolist(),
        },
        
        "intrinsics": {
            "fu": fu.item(),
            "fv": fv.item(),
            "cu": cu.item(),
            "cv": cv.item(),
            "b": b.item(),
        },
        
        'rho_i_pj_i': rho_i_pj_i.T.tolist() 
    }
    with open(file, 'w') as f:
        yaml.dump(params, f)



# save to rosbag
bag = rosbag.Bag(bag_file, 'w')

if __name__ == '__main__':
    pass
    # rospy.init_node('mat2rosbag')
    # pub = rospy.Publisher('/mat2rosbag', Header, queue_size=10)
    # rate = rospy.Rate()
    # while not rospy.is_shutdown():
    #     header = Header()
    #     header.stamp = rospy.Time.now()
    #     pub.publish(header)
    #     rate.sleep()