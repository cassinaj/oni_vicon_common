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

#include "oni_vicon_player/oni_vicon_player.hpp"

#include <boost/filesystem.hpp>
#include <oni_vicon_common/calibration_reader.hpp>
#include <oni_vicon_common/type_conversion.hpp>

using namespace oni_vicon;
using namespace oni_vicon_player;
using namespace depth_sensor_vicon_calibration;

OniViconPlayer::OniViconPlayer(ros::NodeHandle& node_handle,
                               OniPlayer& oni_player,
                               ViconPlayer& vicon_player,
                               const std::string& depth_frame_id,
                               const std::string& camera_info_topic,
                               const std::string& point_cloud_topic):
    image_transport_(node_handle),
    oni_player_(oni_player),
    vicon_player_(vicon_player),
    depth_frame_id_(depth_frame_id),
    camera_info_topic_(camera_info_topic),
    point_cloud_topic_(point_cloud_topic),
    open_as_(OpenGoal::ACTION_NAME,
             boost::bind(&OniViconPlayer::openCb, this, _1),
             false),
    play_as_(PlayGoal::ACTION_NAME,
             boost::bind(&OniViconPlayer::playCb, this, _1),
             false),
    open_(false),
    playing_(false),
    paused_(false)
{
    pause_srv_ = node_handle.advertiseService(Pause::Request::SERVICE_NAME,
                                              &OniViconPlayer::pauseCb,
                                              this);

    seek_frame_srv_ = node_handle.advertiseService(SeekFrame::Request::SERVICE_NAME,
                                                   &OniViconPlayer::seekFrameCb,
                                                   this);

    set_playback_speed_srv_ = node_handle.advertiseService(SetPlaybackSpeed::Request::SERVICE_NAME,
                                                           &OniViconPlayer::setPlaybackSpeedCb,
                                                           this);

    pub_depth_image_ = image_transport_.advertise("depth/image", 100);
    pub_depth_info_ = node_handle.advertise<sensor_msgs::CameraInfo>("depth/camera_info", 100);
    pub_point_cloud_ = node_handle.advertise<sensor_msgs::PointCloud2>("depth/points", 100);
}

OniViconPlayer::~OniViconPlayer()
{
}

void OniViconPlayer::run()
{
    open_as_.start();
    play_as_.start();

    ros::spin();
}

void OniViconPlayer::playCb(const PlayGoalConstPtr& goal)
{
    playing_ = true;

    PlayFeedback feedback;
    feedback.playing = true;
    feedback.current_time = 0;
    feedback.current_vicon_frame = 0;
    feedback.current_depth_sensor_frame = goal->starting_frame;
    oni_player_.seekToFrame(goal->starting_frame);

    play_as_.publishFeedback(feedback);


    // this is due to ros, which complains if the time stamps are small and therefore interpreted as
    // too old. ros merely throws them away. how cruel is that?!
    ros::Time startup_time = ros::Time::now();

    while (ros::ok() && !play_as_.isPreemptRequested() && playing_)
    {
        if (paused_)
        {
            oni_player_.seekToFrame(seeking_frame_);
        }

        boost::mutex::scoped_lock lock(player_lock_);
        if (!oni_player_.processNextFrame())
        {
            break;
        }

        // get depth sensor frame
        sensor_msgs::ImagePtr depth_msg = oni_player_.currentDepthImageMsg();

        // get corresponding vicon frame
        ViconPlayer::PoseRecord vicon_pose = vicon_player_.poseRecord(oni_player_.currentFrameID());

        if (vicon_pose.stamp.isZero())
        {
            break;
        }

        // set meta data and publish
        depth_msg->header.stamp.fromNSec(startup_time.toNSec() + vicon_pose.stamp.toNSec());
        depth_msg->header.frame_id = depth_frame_id_;

        // publish visualization data (depth image, point cloud, vicon pose marker)
        publish(depth_msg);
        vicon_player_.publish(vicon_pose,
                              depth_msg,
                              calibration_transform_.objectDisplay());

        // publish evaluation data
        feedback.current_time = oni_player_.currentFrameID() / 30.;
        feedback.current_vicon_frame = 0;
        feedback.current_depth_sensor_frame = oni_player_.currentFrameID();
        play_as_.publishFeedback(feedback);
    }

    paused_ = false;
    playing_ = false;
    oni_player_.seekToFrame(0);
    play_as_.setSucceeded();
}

void OniViconPlayer::openCb(const OpenGoalConstPtr& goal)
{
    OpenResult result;

    feedback_ .open = false;
    feedback_ .progress = 0;
    feedback_ .progress_max = 0;
    feedback_ .total_time = 0;
    feedback_ .total_vicon_frames = 0;
    feedback_ .total_depth_sensor_frames = 0;

    boost::filesystem::path record_path = goal->record_path;
    std::string recording_name = record_path.leaf().string();

    std::string oni_file = (record_path / (recording_name + ".oni")).string();
    std::string vicon_file = (record_path / (recording_name + ".txt")).string();
    std::string global_calib_file = (record_path / "global_calibration.yaml").string();
    std::string local_calib_file = (record_path / "local_calibration.yaml").string();

    CalibrationReader calibration_reader;
    LocalCalibration local_calibration;
    GlobalCalibration global_calibration;

    // Create calibration transform from calibration files    
    if (calibration_reader.loadGlobalCalibration(global_calib_file, global_calibration))
    {
        calibration_transform_.globalCalibration(global_calibration);
    }
    else
    {
        result.message = "Loading global calibration file <" + global_calib_file + "> failed";
        open_as_.setAborted(result);
        return;
    }       

    if (calibration_reader.loadLocalCalibration(local_calib_file, local_calibration))
    {
        calibration_transform_.localCalibration(local_calibration);
    }
    else
    {
        result.message = "Loading local calibration file <" + local_calib_file + "> failed";
        open_as_.setAborted(result);
        return;
    }

    // open oni player using the oni file and the camera intrinsics from the calibration
    if (!oni_player_.open(oni_file, calibration_transform_.cameraIntrinsics()))
    {
        result.message = "Opening ONI file <" + oni_file + "> failed";
        open_as_.setAborted(result);
        oni_player_.close();
        return;
    }

    feedback_.progress_max = oni_player_.countFrames();
    feedback_.progress = 0;

    // load vicon data
    if (!vicon_player_.load(vicon_file,
                            calibration_transform_,
                            boost::bind(&OniViconPlayer::loadUpdateCb, this, _1)))
    {
        result.message = "Loading vicon data file <" + vicon_file + "> failed";
        open_as_.setAborted(result);
        oni_player_.close();
        return;
    }

    feedback_.progress = feedback_.progress_max;
    feedback_.open = true;
    feedback_.total_time = oni_player_.countFrames() / 30;
    feedback_.total_vicon_frames = feedback_.total_time * 100;
    feedback_.total_depth_sensor_frames = oni_player_.countFrames();
    open_as_.publishFeedback(feedback_);

    ros::Rate rate(30);
    while (ros::ok() && !open_as_.isPreemptRequested())
    {
        rate.sleep();
    }

    result.message = "Record closed";

    // stop player if playing
    paused_ = false;
    playing_ = false;
    feedback_ .open = false;
    feedback_ .progress = 0;
    feedback_ .total_time = 0;
    feedback_ .total_vicon_frames = 0;
    feedback_ .total_depth_sensor_frames = 0;
    open_as_.publishFeedback(feedback_);

    // wait till oni player processing is over (if the controller is implemented correctly, this
    // should not be necessary)
    boost::mutex::scoped_lock lock(player_lock_);

    oni_player_.close();
    open_as_.setSucceeded(result);
}

bool OniViconPlayer::pauseCb(Pause::Request& request, Pause::Response& response)
{
    seeking_frame_ = oni_player_.currentFrameID();
    paused_ = request.paused;

    return true;
}

bool OniViconPlayer::seekFrameCb(SeekFrame::Request& request, SeekFrame::Response &response)
{
    //boost::mutex::scoped_lock lock(player_lock_);
    seeking_frame_ = request.frame;
    return playing_ && paused_ && oni_player_.seekToFrame(request.frame);
}

bool OniViconPlayer::setPlaybackSpeedCb(SetPlaybackSpeed::Request& request,
                                        SetPlaybackSpeed::Response& response)
{
    //return playing_ && paused_ && oni_player_.setPlaybackSpeed(request.speed);
    return oni_player_.setPlaybackSpeed(request.speed);
}

void OniViconPlayer::loadUpdateCb(int64_t frames_loaded)
{
    feedback_.progress = frames_loaded;
    open_as_.publishFeedback(feedback_);
}

void OniViconPlayer::publish(sensor_msgs::ImagePtr depth_msg)
{
    depth_msg->header.frame_id = depth_frame_id_;

    if (pub_depth_info_.getNumSubscribers() > 0)
    {
        sensor_msgs::CameraInfoPtr camera_info =
                oni_vicon::toCameraInfo(calibration_transform_.cameraIntrinsics());
        camera_info->header.frame_id = depth_msg->header.frame_id;
        camera_info->header.stamp = depth_msg->header.stamp;
        camera_info->height = depth_msg->height;
        camera_info->width = depth_msg->width;

        pub_depth_info_.publish(camera_info);
    }

    if (pub_depth_image_.getNumSubscribers () > 0)
    {
        pub_depth_image_.publish(depth_msg);
    }

    if (pub_point_cloud_.getNumSubscribers () > 0)
    {
        sensor_msgs::PointCloud2Ptr points_msg = boost::make_shared<sensor_msgs::PointCloud2>();
        oni_vicon::toMsgPointCloud(depth_msg,
                                   calibration_transform_.cameraIntrinsics(),
                                   points_msg);

        pub_point_cloud_.publish(points_msg);
    }
}
