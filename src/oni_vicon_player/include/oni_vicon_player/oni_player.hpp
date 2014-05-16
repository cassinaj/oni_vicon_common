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
        typedef depth_sensor_vicon_calibration::Transformer::CameraIntrinsics
                CameraIntrinsics;

    public:
        OniPlayer();
        virtual ~OniPlayer();

        bool open(const std::string &source_file, const CameraIntrinsics& camera_intrinsics);
        bool processNextFrame();
        bool close();

        bool seekToFrame(XnInt32 frameID);
        bool setPlaybackSpeed(double speed);

        XnUInt32 currentFrameID() const;
        XnUInt32 countFrames() const;

        const xn::DepthMetaData& depthMetaData() const;
        sensor_msgs::ImagePtr depthFrameAsMsgImage();
        sensor_msgs::PointCloud2Ptr depthFrameAsMsgPointCloud(
                const CameraIntrinsics& camera_intrinsics);


        void toMsgImage(const xn::DepthMetaData& depth_meta_data,
                        sensor_msgs::ImagePtr image) const;
        void toMsgPointCloud(const sensor_msgs::ImagePtr& image,
                             const CameraIntrinsics &camera_intrinsics,
                             sensor_msgs::PointCloud2Ptr points);


        float toMeter(const XnDepthPixel& depth_pixel) const;
        float toMillimeter(const XnDepthPixel& depth_pixel) const;

    private: /* implementation details */        
        struct Point3d // simple low cost vector (alternatives, tf::Point/tf::Vector3d)
        {
            float x;
            float y;
            float z;
        };

        Point3d toPoint3d(float depth,
                          float x,
                          float y,
                          const CameraIntrinsics& camera_intrinsics) const;

    private:
        /* OpenNI player */
        xn::Context context_;
        xn::Player player_;
        xn::DepthGenerator depth_generator_;
        XnUInt64 no_sample_value_;
        XnUInt64 shadow_value_;

        /* frame data */
        xn::DepthMetaData depth_meta_data_;
        XnUInt32 frames_;

        boost::mutex capture_mutex_;

        sensor_msgs::PointCloud2Ptr msg_pointcloud_;
        sensor_msgs::ImagePtr msg_image_;

        bool msg_image_dirty_;
        bool msg_pointcloud_dirty_;
    };
}

#endif
