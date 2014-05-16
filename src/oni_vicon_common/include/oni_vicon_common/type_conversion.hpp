/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2014 Max-Planck-Institute for Intelligent Systems,
 *                     University of Southern California,
 *                     Karlsruhe Institute of Technology
 *    Jan Issac (jan.issac@gmail.com)
 *
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 */

/**
 * @date 05/14/2014
 * @author Jan Issac (jan.issac@gmail.com)
 * Max-Planck-Institute for Intelligent Systems, University of Southern California (USC),
 *   Karlsruhe Institute of Technology (KIT)
 */

#ifndef ONI_VICON_COMMON_TYPE_CONVERSION_HPP
#define ONI_VICON_COMMON_TYPE_CONVERSION_HPP

// openni
#include <ni/XnCppWrapper.h>

// ros
#include <ros/ros.h>
#include <tf/tf.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>

#include "oni_vicon_common/types.hpp"

namespace oni_vicon
{
    /**
     * @brief toMsgPointCloud constructs the 3D point cloud out of the depth image
     *
     * @param [in]  image                 Depth image
     * @param [in]  camera_intrinsics     Camera intrisic parameters used to capture the depth image
     * @param [out] points                Resulting point cloud
     */
    void toMsgPointCloud(const sensor_msgs::ImagePtr& image,
                         const CameraIntrinsics& camera_intrinsics,
                         sensor_msgs::PointCloud2Ptr points);

    /**
     * @brief toPoint3d Calculates the 3D point given a pixel position and its depth value
     *
     * @param u                     u-coordinate in the depth image
     * @param v                     v-coordinate in the depth image
     * @param depth                 Depth value of pixel (u,v)
     * @param camera_intrinsics     Camera intrisic parameters used to capture the depth value
     *
     * @return 3D point (u, v, depth)
     */
    Point3d toPoint3d(float u, float v, float depth, const CameraIntrinsics& camera_intrinsics);

    /**
     * @brief toMsgPose converts tf::Pose to geometry_msgs::Pose
     *
     * @param [in]  tf_pose     Given tf pose
     * @param [out] msg_pose    Requested geometry_msgs pose
     */
    void toMsgPose(const tf::Pose& tf_pose, geometry_msgs::Pose& msg_pose);

    /**
     * @brief toTfPose converts geometry_msgs::Pose to tf::Pose
     *
     * @param [in]  msg_pose    Given geometry_msgs pose
     * @param [out] tf_pose     Requested tf pose
     */
    void toTfPose(const geometry_msgs::Pose& msg_pose, tf::Pose& tf_pose);

    /**
     * @brief toTfTransform converts geometry_msgs::Pose to tf::Transform
     *
     * Converts geometry_msgs::Pose to tf::Transform. This is the same as
     * toTfPose(const geometry_msgs::Pose&, tf::Pose&) since tf::Pose is a typedef of tf::Transform
     *
     * @param [in]  msg_pose        Given geometry_msgs pose
     * @param [out] tf_transform    Requested tf transform
     */
    void toTfTransform(const geometry_msgs::Pose& msg_pose, tf::Transform& tf_transform);

    /**
     * @brief toCameraIntrinsics constructs out of sensor_msgs::CameraInfo
     *
     * @param [in]  msg_camera_info     Given camera info message
     * @param [out] camera_intrinsics   Requested camera intrinsics
     */
    void toCameraIntrinsics(sensor_msgs::CameraInfoConstPtr msg_camera_info,
                            CameraIntrinsics& camera_intrinsics);

    /**
     * @brief toCameraInfo constructs a sensor_msgs::CameraInfo out of CameraIntrinsics
     *
     * @param [in]  camera_intrinsics   Given camera intrinsics
     * @param [out] msg_camera_info     Requested sensor_msgs::CameraInfo
     */
    void toCameraInfo(const CameraIntrinsics& camera_intrinsics,
                      sensor_msgs::CameraInfoPtr msg_camera_info);


    /**
     * @brief toMsgPose converts tf::Pose to geometry_msgs::Pose
     *
     * @param [in]  tf_pose     Given tf pose
     *
     * @return Requested geometry_msgs pose
     */
    geometry_msgs::Pose toMsgPose(const tf::Pose& tf_pose);

    /**
     * @brief toTfPose converts geometry_msgs::Pose to tf::Pose
     *
     * @param [in]  msg_pose    Given geometry_msgs pose
     *
     * @return Requested tf pose
     */
    tf::Pose toTfPose(const geometry_msgs::Pose& msg_pose);

    /**
     * @brief toTfTransform converts geometry_msgs::Pose to tf::Transform
     *
     * Converts geometry_msgs::Pose to tf::Transform. This is the same as
     * toTfPose(const geometry_msgs::Pose&, tf::Pose&) since tf::Pose is a typedef of tf::Transform
     *
     * @param [in]  msg_pose        Given geometry_msgs pose
     *
     * @return Requested tf transform
     */
    tf::Transform toTfTransform(const geometry_msgs::Pose& msg_pose);

    /**
     * @brief toCameraIntrinsics constructs out of sensor_msgs::CameraInfo
     *
     * @param [in]  msg_camera_info     Given camera info message
     *
     * @return Requested camera intrinsics
     */
    CameraIntrinsics toCameraIntrinsics(sensor_msgs::CameraInfoConstPtr msg_camera_info);

    /**
     * @brief toCameraInfo constructs a sensor_msgs::CameraInfo out of CameraIntrinsics
     *
     * @param [in]  camera_intrinsics   Given camera intrinsics
     *
     * @return Requested sensor_msgs::CameraInfo
     */
    sensor_msgs::CameraInfoPtr toCameraInfo(const CameraIntrinsics& camera_intrinsics);
}

#endif
