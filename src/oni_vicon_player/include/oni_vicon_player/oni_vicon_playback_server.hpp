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

#ifndef ONI_VICON_PLAYER_ONI_VICON_PLAYBACK_SERVER_HPP
#define ONI_VICON_PLAYER_ONI_VICON_PLAYBACK_SERVER_HPP

// c++/std
#include <string>
#include <stdint.h>

// ros
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/CameraInfo.h>
#include <stereo_msgs/DisparityImage.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/PointCloud2.h>

// oni_vicon_common
#include <oni_vicon_common/types.hpp>

#include "oni_vicon_player/oni_player.hpp"
#include "oni_vicon_player/vicon_player.hpp"

namespace oni_vicon_player
{
    class OniViconPlayback
    {
    public:
        typedef boost::shared_ptr<OniViconPlayback> Ptr;

    public:
        /**
         * @brief OniViconPlayback Creates an instance of ONI-Vicon playback
         *
         * @param oni_player    Used OniPlayer to read ONI depth frames
         * @param vicon_player  Used ViconPlayer to read the vicon poses
         */
        OniViconPlayback(OniPlayer::Ptr oni_player,
                         ViconPlayer::Ptr vicon_player);

        /**
         * @brief open opens an ONI-Vicon recording
         *
         * @param recording Record directory
         */
        void open(const std::string& record_dir,
                  ViconPlayer::LoadUpdateCallback update_cb = ViconPlayer::LoadUpdateCallback());

        /**
         * @brief close
         */
        void close();

        /**
         * @brief play
         * @param starting_frame
         */
        void play(uint32_t starting_frame);

        /**
         * @brief stop
         */
        void stop();

        /**
         * @brief nextFrame
         * @return
         */
        uint32_t nextFrame();

        /**
         * @brief seekToFrame
         * @param starting_frame
         */
        void seekToFrame(uint32_t starting_frame);

        /**
         * @brief setPlaybackSpeed
         * @param speed
         */
        void setPlaybackSpeed(double speed);

        /**
         * @brief isEOF
         * @return
         */
        bool isEOF() const;

        /**
         * @brief isOpen
         * @return
         */
        bool isOpen() const;

        /**
         * @brief isPlaying
         * @return
         */
        bool isPlaying() const;

        /**
         * @brief oniPlayer
         *
         * @return
         */
        OniPlayer::Ptr oniPlayer();

        /**
         * @brief viconPlayer
         * @return
         */
        ViconPlayer::Ptr viconPlayer();

        /**
         * @brief transformer
         * @return
         */
        const oni_vicon::Transformer& transformer() const;

    private:
        OniPlayer::Ptr oni_player_;
        ViconPlayer::Ptr vicon_player_;

        oni_vicon::Transformer calibration_transform_;

        bool open_;
        bool playing_;
        uint32_t seeking_frame_;

        sensor_msgs::PointCloud2Ptr msg_pointcloud_;
        sensor_msgs::ImagePtr msg_image_;

        bool msg_image_dirty_;
        bool msg_pointcloud_dirty_;
    };

}

#endif
