# test.launch.py

import os

from ament_index_python import get_package_share_directory

from launch import LaunchDescription
from launch import LaunchContext
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.actions import OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import TextSubstitution
from launch_ros.actions import Node
from launch_ros.actions import PushRosNamespace

def launches(context):
    launch_files = [];

    robot_type_str = context.perform_substitution(LaunchConfiguration('robot_type'))
    moveit_package_name = "robot_moveit_config"
    moveit_share_dir = get_package_share_directory(moveit_package_name)
    
    # include another launch files
    launch_files.append(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    moveit_share_dir,
                    'launch/move_group.launch.py'))
        )
    )
    launch_files.append(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    moveit_share_dir,
                    'launch/rsp.launch.py'))
        )
    )
    launch_files.append(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    moveit_share_dir,
                    'launch/static_virtual_joint_tfs.launch.py'))
        )
    )
    launch_files.append(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    moveit_share_dir,
                    'launch/moveit_rviz.launch.py'))
        )
    )

    return launch_files
    
def generate_launch_description():

    args = []
    args.append(
        DeclareLaunchArgument(
            'robot_type',
            description='the string to specify the robot model (i.e. the prefix of the moveit config)\n\n\
            [Valid robot_type] / [robot name R-30iB+ / R-50iA]\n\
            \n\
            == CRX series (crx_description) =========================\n\
            crx5ia             / CRX-5iA           /  N/A \n\
            crx10ia            / CRX-10iA          /  N/A \n\
            crx10ia_l          / CRX-10iA/L        /  N/A \n\
            crx20ia_l          / CRX-20iA/L        /  N/A \n\
            crx25ia_100v       / CRX-25iA          /  N/A \n\
            crx25ia_200v       / CRX-25iA          /  N/A \n\
            \n\
            == R-2000iD series (r2000id_description) ================\n\
            r2000id_165fh      / R-2000iD/165FH    / R-2000/165F-26D Hollow \n\
            \n\
            == R-2000iC series (r2000ic_description) ================\n\
            r2000ic_165f       / R-2000iC/165F     / R-2000/165F-27C \n\
            r2000ic_210l       / R-2000iC/210L     / R-2000/210F-31C \n\
            r2000ic_240f       / R-2000iC/240F     / R-2000/240F-27C \n\
            \n\
            == R-1000iA series (r1000ia_description) ================\n\
            r1000ia_80f        / R-1000iA/80F      / R-1000/80-22A  \n\
            r1000ia_100f       / R-1000iA/100F     / R-1000/100-22A \n\
            r1000ia_120f_7b    / R-1000iA/120F-7B  / N/A            \n\
            \n\
            == M-800iA series (m800ia_description) ================\n\
            m800ia_60          / M-800iA/60        / N/A \n\
            \n\
            == M-1000iA series (m1000ia_description) ================\n\
            m1000ia            / M-1000iA          / M-1000/1000F-33A \n\
            \n\
            == LR Mate 200iD series (lrm200id_description)===========\n\
            lrm200id           / LR Mate 200iD     / LR Mate/7-7D \n\
            lrm200id_7l        / LR Mate 200iD/7L  / LR Mate/7-9D \n\
            \n\
            == M-20iB series (m20ib_description)=====================\n\
            m20ib_25           / M-20iB/25         / LR Mate/25-19A \n\
            == M-10iD series (m10id_description)=====================\n\
            m10id_8l           / M-10iD/8L         / M-10/8-20D \n\
            m10id_12           / M-10iD/12         / M-10/12-14D \n\
            m10id_16s          / M-10iD/16S        / M-10/16-11D \n\
            \n\
            == M-20iD series (m20id_description)=====================\n\
            m20id_12l          / M-20iD/12L        / M-20/12-23D \n\
            m20id_25           / M-20iD/25         / M-20/25-18D \n\
            m20id_35           / M-20iD/35         / M-20/35-18D \n\
            \n\
            == LR-10iA series (lr10ia_description)===================\n\
            lr10ia_10          / LR-10iA/10        / LR Mate/10-11A \n\
            \n\
            == M-710iC series (m710ic_description)===================\n\
            m710ic_45m         / M-710iC/45M       / N/A \n\
            \n\
            == M-710iD series (m710id_description)===================\n\
            m710id_50m         / M-710iD/50M       / M-710/50-26D \n\
            \n\
            == SR series (scara_description)===================\n\
            sr3ia_c            / SR-3iA/C          /  N/A \n\
            sr20ia             / SR-20iA           /  N/A \n\
            \n',
            choices=['crx5ia', 'crx10ia', 'crx10ia_l', 'crx20ia_l', 'crx25ia_100v', 'crx25ia_200v', 'r2000id_165fh', 'r2000ic_165f', 'r2000ic_210l', 'r2000ic_240f',\
            'r1000ia_80f', 'r1000ia_100f', 'r1000ia_120f_7b', 'm800ia_60', 'm1000ia', 'lrm200id', 'lrm200id_7l', 'm20ib_25', 'm10id_8l', 'm10id_12', 'm10id_16s',\
            'm20id_12l', 'm20id_25', 'm20id_35', 'lr10ia_10', 'm710ic_45m', 'm710id_50m', 'sr3ia_c', 'sr20ia'],
            default_value=TextSubstitution(text='crx10ia_l')
        )
    )

    return LaunchDescription(args + [OpaqueFunction(function=launches)])
