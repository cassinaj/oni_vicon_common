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
 * Karlsruhe Institute of Technology (KIT), University of Southern California (USC)
 */

#include <sensor_msgs/PointCloud2.h>
#include <stereo_msgs/DisparityImage.h>
#include <string>
#include <yaml-cpp/yaml.h>

#include "oni_vicon_player/oni_player.hpp"

using namespace oni_vicon_player;

OniPlayer::OniPlayer()
{
    priv_nh_ = ros::NodeHandle("~");

    priv_nh_.param ("rgb_frame_id", rgb_frame_id_, std::string (""));
    if (rgb_frame_id_.empty ())
      {
        rgb_frame_id_ = "/camera_rgb_optical_frame";
        ROS_INFO ("'rgb_frame_id' not set. using default: '%s'", rgb_frame_id_.c_str());
      }
    else
      ROS_INFO ("rgb_frame_id = '%s' ", rgb_frame_id_.c_str());

    priv_nh_.param ("depth_frame_id", depth_frame_id_, std::string (""));
    if (depth_frame_id_.empty ())
      {
        depth_frame_id_ = "/camera_depth_optical_frame";
        ROS_INFO ("'depth_frame_id' not set. using default: '%s'", depth_frame_id_.c_str());
      }
    else
      ROS_INFO ("depth_frame_id = '%s' ", depth_frame_id_.c_str());

    // Init all publishers
    it_ = new image_transport::ImageTransport(priv_nh_);
    pub_depth_image_ = it_->advertise ("depth/image"    , 5);

    pub_disp_image_  = priv_nh_.advertise<stereo_msgs::DisparityImage > ("depth/disparity", 5);
    pub_depth_info_ = priv_nh_.advertise<sensor_msgs::CameraInfo > ("depth/camera_info", 5);
    pub_point_cloud_ = priv_nh_.advertise<sensor_msgs::PointCloud2> ("depth/points", 5);
}

OniPlayer::~OniPlayer()
{

}
