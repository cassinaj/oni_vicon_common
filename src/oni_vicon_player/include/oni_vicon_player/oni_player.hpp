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

#include <ni/XnCppWrapper.h>

#define KINECT_IMAGE_COLS       640
#define KINECT_IMAGE_ROWS       480

namespace oni_vicon_player
{
    class OniPlayer
    {
    public:
        struct Point3d
        {
            float x;
            float y;
            float z;
        };



    public:
        OniPlayer();
        ~OniPlayer();

        bool init();
        bool process();
        bool shutdown();

        void toMsgImage(const xn::DepthMetaData& depth_meta_data,
                        sensor_msgs::ImagePtr image) const;

        void toMsgPointCloud(const sensor_msgs::ImagePtr& image,
                             sensor_msgs::PointCloud2Ptr points);

        float toFloat(const XnDepthPixel& depth_pixel) const;
        Point3d toPoint3d(const XnDepthPixel& depth_pixel, float x, float y) const;
    private:
        ros::NodeHandle node_handle_;

        image_transport::ImageTransport* it_;

        std::string camera_name_;
        std::string frame_id_;

        sensor_msgs::Image gray_image_;

        sensor_msgs::CameraInfo depth_cam_info_;

        // Publishers Camera Info
        ros::Publisher pub_depth_info_;
        // Publishers Images
        image_transport::Publisher pub_depth_image_;
        // Publishers Point Clouds
        ros::Publisher pub_point_cloud_;

        std::string rgb_frame_id_;
        std::string depth_frame_id_;
        unsigned image_width_;
        unsigned image_height_;
        unsigned depth_width_;
        unsigned depth_height_;

        inline bool isImageStreamRequired() const;
        inline bool isDepthStreamRequired() const;
        sensor_msgs::CameraInfoPtr fillCameraInfo (ros::Time time, bool is_rgb);

        void subscriberChangedEvent ();

        // publish methods
        void publishDepthImage (ros::Time time);
        void publishXYZPointCloud (ros::Time time);

        boost::mutex capture_mutex_;

        std::string source_file_;
        xn::Context context_;
        xn::Player player_;
        xn::DepthGenerator depth_generator_;
        XnUInt64 no_sample_value_;
        XnUInt64 shadow_value_;

        xn::DepthMetaData depth_meta_data_;
        XnUInt32 frames_;
        XnUInt32 current_frame_;
        bool paused_;
    };

    bool OniPlayer::isDepthStreamRequired() const
    {
        return (pub_depth_image_.getNumSubscribers() > 0
                || pub_point_cloud_.getNumSubscribers() > 0);
    }
}

#endif
