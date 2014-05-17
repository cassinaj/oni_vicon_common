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


#ifndef ONI_VICON_PLAYER_ONI_VICON_PLAYER_HPP
#define ONI_VICON_PLAYER_ONI_VICON_PLAYER_HPP

#include "oni_vicon_player/oni_player.hpp"
#include "oni_vicon_player/vicon_player.hpp"

// boost
#include <boost/thread/mutex.hpp>

// ros
#include <actionlib/server/simple_action_server.h>

// depth_sensor_vicon_calibration
#include <depth_sensor_vicon_calibration/transform.hpp>

// actions
#include <oni_vicon_player/OpenAction.h>
#include <oni_vicon_player/PlayAction.h>

// services
#include <oni_vicon_player/Pause.h>
#include <oni_vicon_player/SeekFrame.h>
#include <oni_vicon_player/SetPlaybackSpeed.h>

#include <oni_vicon_common/types.hpp>

namespace oni_vicon_player
{
    class OniViconPlayer
    {
    public:
        OniViconPlayer(ros::NodeHandle& node_handle,
                       OniPlayer& oni_player,
                       ViconPlayer& vicon_player,
                       const std::string &depth_frame_id,
                       const std::string &camera_info_topic,
                       const std::string &point_cloud_topic);
        virtual ~OniViconPlayer();

        void run();

    public: /* action callbacks */
        void playCb(const PlayGoalConstPtr& goal);
        void openCb(const OpenGoalConstPtr& goal);

    public: /* service callbacks */
        bool pauseCb(Pause::Request& request,
                     Pause::Response& response);
        bool seekFrameCb(SeekFrame::Request& request,
                         SeekFrame::Response& response);
        bool setPlaybackSpeedCb(SetPlaybackSpeed::Request& request,
                                SetPlaybackSpeed::Response& response);

        void loadUpdateCb(int64_t frames_loaded);

        void publish(sensor_msgs::ImagePtr image);

        sensor_msgs::ImagePtr depthFrameAsMsgImage();
        sensor_msgs::PointCloud2Ptr depthFrameAsMsgPointCloud();


        void toMsgImage(const xn::DepthMetaData& depth_meta_data,
                        sensor_msgs::ImagePtr image,
                        oni_vicon::Unit unit = oni_vicon::Meter);

        float toMeter(const XnDepthPixel& depth_pixel) const;
        float toMillimeter(const XnDepthPixel& depth_pixel) const;

    private:
        boost::mutex player_lock_;

        image_transport::ImageTransport image_transport_;
        image_transport::Publisher pub_depth_image_;
        ros::Publisher pub_point_cloud_;
        ros::Publisher pub_depth_info_;

        /* published data */
        std::string depth_frame_id_;
        std::string camera_info_topic_;
        std::string point_cloud_topic_;
        sensor_msgs::CameraInfo depth_cam_info_;

        OniPlayer& oni_player_;
        ViconPlayer& vicon_player_;

        oni_vicon::Transformer calibration_transform_;

        actionlib::SimpleActionServer<OpenAction> open_as_;
        actionlib::SimpleActionServer<PlayAction> play_as_;

        OpenFeedback feedback_;

        ros::ServiceServer pause_srv_;
        ros::ServiceServer seek_frame_srv_;
        ros::ServiceServer set_playback_speed_srv_;

        bool open_;
        bool playing_;
        bool paused_;
        XnUInt32 seeking_frame_;


        sensor_msgs::PointCloud2Ptr msg_pointcloud_;
        sensor_msgs::ImagePtr msg_image_;

        bool msg_image_dirty_;
        bool msg_pointcloud_dirty_;
    };
}

#endif
