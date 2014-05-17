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

#include <ros/ros.h>

#if ROS_VERSION_MINIMUM(1, 3, 0)
#include <sensor_msgs/distortion_models.h>
#endif

#include "oni_vicon_common/type_conversion.hpp"

namespace oni_vicon
{
    void toMsgPointCloud(const sensor_msgs::ImagePtr& image,
                         const CameraIntrinsics& camera_intrinsics,
                         sensor_msgs::PointCloud2Ptr points)
    {
        points->header.frame_id = image->header.frame_id;
        points->header.stamp = image->header.stamp;
        points->height = image->height;
        points->width = image->width;
        points->is_dense = false;
        points->is_bigendian = false;
        points->fields.resize(3);
        points->fields[0].name = "x";
        points->fields[1].name = "y";
        points->fields[2].name = "z";

        int offset = 0;
        for (size_t d = 0; d < points->fields.size(); ++d, offset += sizeof(float))
        {
            points->fields[d].offset = offset;
            points->fields[d].datatype = sensor_msgs::PointField::FLOAT32;
            points->fields[d].count  = 1;
        }

        points->point_step = offset;
        points->row_step = points->point_step * points->width;
        points->data.resize(points->width * points->height * points->point_step);

        const float* depth_data = reinterpret_cast<const float*>(&image->data[0]);
        float* point_data = reinterpret_cast<float*>(&points->data[0]);

        Point3d p;
        for(int  v = 0, k = 0; v < image->height; ++v)
        {
            for(int u = 0; u < image->width; ++u, ++k, ++point_data)
            {
                p = toPoint3d(u, v, depth_data[k], camera_intrinsics);

                *point_data++ = p.x;
                *point_data++ = p.y;
                *point_data = p.z;
            }
        }
    }

    Point3d toPoint3d(float u, float v, float depth, const CameraIntrinsics& camera_intrinsics)
    {
        Point3d point;

        static float bad_point = std::numeric_limits<float>::quiet_NaN();

        if (depth == bad_point)
        {
            point.x = point.y = point.z = bad_point;
            return point;
        }

        point.z = depth;
        point.x = (u - camera_intrinsics.cx) * depth / camera_intrinsics.f;
        point.y = (v - camera_intrinsics.cy) * depth / camera_intrinsics.f;

        return point;
    }

    void toMsgPose(const tf::Pose& tf_pose, geometry_msgs::Pose& msg_pose)
    {
        msg_pose.position.x = tf_pose.getOrigin().getX();
        msg_pose.position.y = tf_pose.getOrigin().getY();
        msg_pose.position.z = tf_pose.getOrigin().getZ();

        tf::Quaternion orientation = tf_pose.getRotation();
        msg_pose.orientation.w = orientation.getW();
        msg_pose.orientation.x = orientation.getX();
        msg_pose.orientation.y = orientation.getY();
        msg_pose.orientation.z = orientation.getZ();
    }

    void toTfPose(const geometry_msgs::Pose& msg_pose, tf::Pose& tf_pose)
    {
        tf_pose.setOrigin(tf::Vector3(msg_pose.position.x,
                                      msg_pose.position.y,
                                      msg_pose.position.z));

        tf_pose.setRotation(tf::Quaternion(msg_pose.orientation.x,
                                           msg_pose.orientation.y,
                                           msg_pose.orientation.z,
                                           msg_pose.orientation.w));
    }

    void toTfTransform(const geometry_msgs::Pose& msg_pose, tf::Transform& tf_transform)
    {
        toTfPose(msg_pose, tf_transform);
    }

    void toCameraIntrinsics(
            sensor_msgs::CameraInfoConstPtr msg_camera_info,
            CameraIntrinsics& camera_intrinsics)
    {
        camera_intrinsics.f = (msg_camera_info->K[0] + msg_camera_info->K[4])/2.;
        camera_intrinsics.cx = msg_camera_info->K[2];
        camera_intrinsics.cy = msg_camera_info->K[5];
    }

    void toCameraInfo(const CameraIntrinsics& camera_intrinsics,
                      sensor_msgs::CameraInfoPtr msg_camera_info)
    {
        /*
         * be aware to set these !
        msg_camera_info->header.stamp    = ;
        msg_camera_info->header.frame_id = ;
        msg_camera_info->width           = ;
        msg_camera_info->height          = ;
        */

        /* taken from arm_rgbd package */

    #if ROS_VERSION_MINIMUM(1, 3, 0)
        msg_camera_info->D = std::vector<double>(5, 0.0);
        msg_camera_info->distortion_model = sensor_msgs::distortion_models::PLUMB_BOB;
    #else
        msg_camera_info->D.assign (0.0);
    #endif
        msg_camera_info->K.assign (0.0);
        msg_camera_info->R.assign (0.0);
        msg_camera_info->P.assign (0.0);
        // Simple camera matrix: square pixels, principal point at center
        msg_camera_info->K[0] = msg_camera_info->K[4] = camera_intrinsics.f;
        msg_camera_info->K[2] = camera_intrinsics.cx;
        msg_camera_info->K[5] = camera_intrinsics.cy;
        msg_camera_info->K[8] = 1.0;
        // no rotation: identity
        msg_camera_info->R[0] = msg_camera_info->R[4] = msg_camera_info->R[8] = 1.0;
        // no rotation, no translation => P=K(I|0)=(K|0)
        msg_camera_info->P[0] = msg_camera_info->P[5] = msg_camera_info->K[0];
        msg_camera_info->P[2] = msg_camera_info->K[2];
        msg_camera_info->P[6] = msg_camera_info->K[5];
        msg_camera_info->P[10] = 1.0;
    }

    geometry_msgs::Pose toMsgPose(const tf::Pose& tf_pose)
    {
        geometry_msgs::Pose pose;
        toMsgPose(tf_pose, pose);
        return pose;
    }

    tf::Pose toTfPose(const geometry_msgs::Pose& msg_pose)
    {
        tf::Pose pose;
        toTfPose(msg_pose, pose);
        return pose;
    }

    tf::Transform toTfTransform(const geometry_msgs::Pose& msg_pose)
    {
        tf::Transform transform;
        toTfTransform(msg_pose, transform);
        return transform;
    }

    CameraIntrinsics toCameraIntrinsics(sensor_msgs::CameraInfoConstPtr msg_camera_info)
    {
        CameraIntrinsics camera_intrinsics;
        toCameraIntrinsics(msg_camera_info, camera_intrinsics);
        return camera_intrinsics;
    }

    sensor_msgs::CameraInfoPtr toCameraInfo(const CameraIntrinsics& camera_intrinsics)
    {
        sensor_msgs::CameraInfoPtr msg_camera_info = boost::make_shared<sensor_msgs::CameraInfo>();
        toCameraInfo(camera_intrinsics, msg_camera_info);
        return msg_camera_info;
    }

}
