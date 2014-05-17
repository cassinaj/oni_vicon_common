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
 * @date 05/08/2014
 * @author Jan Issac (jan.issac@gmail.com)
 * Max-Planck-Institute for Intelligent Systems, University of Southern California (USC),
 *   Karlsruhe Institute of Technology (KIT)
 */


// c++/std
#include <string>

#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>

// ros
#include <ros/ros.h>

#include <oni_vicon_player/oni_player.hpp>
#include <oni_vicon_player/vicon_player.hpp>
#include <oni_vicon_player/oni_vicon_player.hpp>
#include <oni_vicon_player/oni_vicon_playback_server.hpp>

using namespace oni_vicon_player;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "oni_vicon_recorder");
    ros::NodeHandle nh("~");

    /* Parameters */
    // calibration parameters with defaults
    std::string depth_frame_id = "/XTION_IR";
    std::string camera_info_topic = "/XTION/depth/camera_info";
    std::string point_cloud_topic = "/XTION/depth/points";

    // load set parameters otherwise maintain defautls
    nh.param("depth_frame_id", depth_frame_id, depth_frame_id);
    nh.param("camera_info_topic", camera_info_topic, camera_info_topic);
    nh.param("point_cloud_topic", point_cloud_topic, point_cloud_topic);

    OniPlayer::Ptr oni_player = boost::make_shared<OniPlayer>();
    ViconPlayer::Ptr vicon_player = boost::make_shared<ViconPlayer>();
    OniViconPlayback::Ptr playback = boost::make_shared<OniViconPlayback>(oni_player, vicon_player);

    OniViconPlayer oni_vicon_player(nh,
                                    playback,
                                    depth_frame_id,
                                    camera_info_topic,
                                    point_cloud_topic);

    oni_vicon_player.run();

    return 0;
}
