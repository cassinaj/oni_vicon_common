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
 * @date 05/04/2014
 * @author Jan Issac (jan.issac@gmail.com)
 * Max-Planck-Institute for Intelligent Systems, University of Southern California (USC),
 *   Karlsruhe Institute of Technology (KIT)
 */


#ifndef ONI_VICON_PLAYER_ONI_PLAYER_HPP
#define ONI_VICON_PLAYER_ONI_PLAYER_HPP

#include <boost/thread/mutex.hpp>

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/CameraInfo.h>
#include <stereo_msgs/DisparityImage.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/PointCloud2.h>

#include <ni/XnCppWrapper.h>

#include <depth_sensor_vicon_calibration/transform.hpp>

namespace oni_vicon_player
{
    class OniPlayer
    {
    private:
        typedef depth_sensor_vicon_calibration::CalibrationTransform::CameraIntrinsics
                CameraIntrinsics;

    public:
        OniPlayer(const ros::NodeHandle &node_handle,
                  const std::string& depth_frame_id,
                  const std::string& camera_info_topic,
                  const std::string& point_cloud_topic);
        ~OniPlayer();

        bool open(const std::string &source_file, const CameraIntrinsics& camera_intrinsics);
        bool process(ros::Time time);
        bool close();

        XnUInt32 currentFrame() const;
        XnUInt32 countFrames() const;

        bool seekToFrame(XnInt32 frame);
        bool setPlaybackSpeed(double speed);

    private: /* implementation details */        
        struct Point3d // simple low cost vector (alternatives, tf::Point/tf::Vector3d)
        {
            float x;
            float y;
            float z;
        };

        void toMsgImage(const xn::DepthMetaData& depth_meta_data,
                        sensor_msgs::ImagePtr image) const;
        void toMsgPointCloud(const sensor_msgs::ImagePtr& image,
                             sensor_msgs::PointCloud2Ptr points);
        float toMeter(const XnDepthPixel& depth_pixel) const;
        Point3d toPoint3d(float depth, float x, float y) const;

    private:
        /* ros publisher */
        ros::NodeHandle node_handle_;
        image_transport::ImageTransport image_transport_;
        image_transport::Publisher pub_depth_image_;
        ros::Publisher pub_point_cloud_;
        ros::Publisher pub_depth_info_;

        /* published data */
        std::string depth_frame_id_;
        std::string camera_info_topic_;
        std::string point_cloud_topic_;
        sensor_msgs::CameraInfo depth_cam_info_;

        /* OpenNI player */
        xn::Context context_;
        xn::Player player_;
        xn::DepthGenerator depth_generator_;
        XnUInt64 no_sample_value_;
        XnUInt64 shadow_value_;

        /* frame data */
        xn::DepthMetaData depth_meta_data_;
        XnUInt32 frames_;
        XnUInt32 current_frame_;

        CameraIntrinsics camera_intrinsics_;
        boost::mutex capture_mutex_;
    };
}

#endif
