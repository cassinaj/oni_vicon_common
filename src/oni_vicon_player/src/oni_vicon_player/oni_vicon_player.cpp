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

#include <boost/filesystem.hpp>

#include <oni_vicon_common/calibration_reader.hpp>
#include <oni_vicon_common/type_conversion.hpp>
#include <oni_vicon_common/exceptions.hpp>

#include "oni_vicon_player/exceptions.hpp"
#include "oni_vicon_player/oni_vicon_player.hpp"

using namespace oni_vicon;
using namespace oni_vicon_player;

OniViconPlayer::OniViconPlayer(ros::NodeHandle& node_handle,
                               OniViconPlayback::Ptr playback,
                               const std::string& depth_frame_id,
                               const std::string& camera_info_topic,
                               const std::string& point_cloud_topic):
    playback_(playback),
    image_transport_(node_handle),
    depth_frame_id_(depth_frame_id),
    camera_info_topic_(camera_info_topic),
    point_cloud_topic_(point_cloud_topic),
    open_as_(OpenGoal::ACTION_NAME,
             boost::bind(&OniViconPlayer::openCb, this, _1),
             false),
    play_as_(PlayGoal::ACTION_NAME,
             boost::bind(&OniViconPlayer::playCb, this, _1),
             false),
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

    vicon_object_pose_publisher_ =
            node_handle.advertise<visualization_msgs::Marker>("vicon_object_pose", 0);
}


void OniViconPlayer::run()
{
    open_as_.start();
    play_as_.start();

    ros::spin();
}

void OniViconPlayer::playCb(const PlayGoalConstPtr& goal)
{
    PlayFeedback feedback;
    feedback.playing = true;
    feedback.current_time = 0;
    feedback.current_vicon_frame = 0;
    feedback.current_depth_sensor_frame = goal->starting_frame;



    playback_->play(goal->starting_frame);



    play_as_.publishFeedback(feedback);


    // this is due to ros, which complains if the time stamps are small and therefore interpreted as
    // too old. ros merely throws them away. how cruel is that?!
    ros::Time startup_time = ros::Time::now();

    while (ros::ok() && !play_as_.isPreemptRequested() && playback_->isPlaying())
    {
        if (paused_)
        {
            playback_->seekToFrame(seeking_frame_);
        }

        boost::mutex::scoped_lock lock(player_lock_);
        uint32_t frame_id = playback_->nextFrame();

        // get depth sensor frame and corresponding vicon frame
        sensor_msgs::ImagePtr depth_msg = playback_->oniPlayer()->currentDepthImageMsg();
        ViconPlayer::PoseRecord vicon_pose_record = playback_->viconPlayer()->pose(frame_id);

        if (vicon_pose_record.stamp.isZero())
        {
            break;
        }

        // set meta data and publish
        depth_msg->header.stamp.fromNSec(startup_time.toNSec() + vicon_pose_record.stamp.toNSec());
        depth_msg->header.frame_id = depth_frame_id_;

        // publish visualization data (depth image, point cloud, vicon pose marker)
        publish(depth_msg);
        publish(vicon_pose_record.pose,
                depth_msg,
                playback_->transformer().localCalibration().objectDisplay());

        /*
        tf_broadcaster_.sendTransform(
                    tf::StampedTransform(
                        playback_->transformer().globalCalibration().viconToCameraTransform(),
                        ros::Time(depth_msg->header.stamp.toSec()-10., 0),
                        depth_frame_id_,
                        "vicon_global_frame"));
        */

        tf_broadcaster_.sendTransform(
                    tf::StampedTransform(
                        playback_->transformer().globalCalibration().viconToCameraTransform(),
                        depth_msg->header.stamp,
                        depth_frame_id_,
                        "vicon_global_frame"));

        /*
        tf_broadcaster_.sendTransform(
                    tf::StampedTransform(
                        playback_->transformer().globalCalibration().viconToCameraTransform(),
                        ros::Time(depth_msg->header.stamp.toSec()+10., 0),
                        depth_frame_id_,
                        "vicon_global_frame"));
        */

        // publish evaluation data
        feedback.current_time = frame_id / 30.;
        feedback.current_vicon_frame = 0;
        feedback.current_depth_sensor_frame = frame_id;
        play_as_.publishFeedback(feedback);
    }

    paused_ = false;
    playback_->seekToFrame(0);
    play_as_.setSucceeded();
}

void OniViconPlayer::openCb(const OpenGoalConstPtr& goal)
{
    OpenResult result;

    feedback_.open = false;
    feedback_.progress = 0;
    feedback_.progress_max = 0;
    feedback_.total_time = 0;
    feedback_.total_vicon_frames = 0;
    feedback_.total_depth_sensor_frames = 0;

    try
    {        
        playback_->open(goal->record_path,
                       boost::bind(&OniViconPlayer::loadUpdateCb, this, _1, _2));
    }
    catch(oni_vicon_player::OpenRecordException& e)
    {
        result.message = e.what();
        open_as_.setAborted(result);
        return;
    }

    feedback_.progress = feedback_.progress_max;
    feedback_.open = true;
    feedback_.total_time = playback_->oniPlayer()->countFrames() / 30;
    feedback_.total_vicon_frames = feedback_.total_time * 100;
    feedback_.total_depth_sensor_frames = playback_->oniPlayer()->countFrames();
    open_as_.publishFeedback(feedback_);

    // wait until stopped
    ros::Rate rate(30);
    while (ros::ok() && !open_as_.isPreemptRequested())
    {
        rate.sleep();
    }    

    // stop player if playing
    playback_->stop();
    paused_ = false;
    feedback_ .open = false;
    feedback_ .progress = 0;
    feedback_ .total_time = 0;
    feedback_ .total_vicon_frames = 0;
    feedback_ .total_depth_sensor_frames = 0;
    open_as_.publishFeedback(feedback_);

    // wait till oni player processing is over (if the controller is implemented correctly, this
    // should not be necessary)
    boost::mutex::scoped_lock lock(player_lock_);
    playback_->close();

    result.message = "Record closed";
    open_as_.setSucceeded(result);
}

bool OniViconPlayer::pauseCb(Pause::Request& request, Pause::Response& response)
{
    seeking_frame_ = playback_->oniPlayer()->currentFrameID();
    paused_ = request.paused;

    return true;
}

bool OniViconPlayer::seekFrameCb(SeekFrame::Request& request, SeekFrame::Response &response)
{
    //boost::mutex::scoped_lock lock(player_lock_);
    seeking_frame_ = request.frame;
    return playback_->isPlaying() && paused_ && playback_->oniPlayer()->seekToFrame(request.frame);
}

bool OniViconPlayer::setPlaybackSpeedCb(SetPlaybackSpeed::Request& request,
                                        SetPlaybackSpeed::Response& response)
{
    //return playing_ && paused_ && playback_->oniPlayer()->setPlaybackSpeed(request.speed);
    return playback_->oniPlayer()->setPlaybackSpeed(request.speed);
}

void OniViconPlayer::loadUpdateCb(uint32_t total_frames, uint32_t frames_loaded)
{
    if (total_frames > 0)
    {
        feedback_.progress_max = total_frames;
    }

    feedback_.progress = frames_loaded;
    open_as_.publishFeedback(feedback_);
}

void OniViconPlayer::publish(sensor_msgs::ImagePtr depth_msg)
{
    depth_msg->header.frame_id = depth_frame_id_;

    if (pub_depth_info_.getNumSubscribers() > 0)
    {
        sensor_msgs::CameraInfoPtr camera_info =
                oni_vicon::toCameraInfo(playback_->transformer().cameraIntrinsics());
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
                                   playback_->transformer().cameraIntrinsics(),
                                   points_msg);

        pub_point_cloud_.publish(points_msg);
    }
}

void OniViconPlayer::publish(const tf::Pose& pose,
                             sensor_msgs::ImagePtr corresponding_image,
                             const std::string& object_display)
{
    visualization_msgs::Marker marker;
    marker.header.frame_id = corresponding_image->header.frame_id;
    marker.header.stamp =  corresponding_image->header.stamp;
    marker.ns = "vicon_object_pose";
    marker.id = 0;
    marker.scale.x = 1.0;
    marker.scale.y = 1.0;
    marker.scale.z = 1.0;
    marker.color.r = 1;
    marker.color.g = 0;
    marker.color.b = 0;
    marker.color.a = 1;

    marker.type = visualization_msgs::Marker::MESH_RESOURCE;
    marker.action = visualization_msgs::Marker::ADD;

    marker.pose.position.x = pose.getOrigin().getX();
    marker.pose.position.y = pose.getOrigin().getY();
    marker.pose.position.z = pose.getOrigin().getZ();

    tf::Quaternion orientation = pose.getRotation();
    marker.pose.orientation.w = orientation.getW();
    marker.pose.orientation.x = orientation.getX();
    marker.pose.orientation.y = orientation.getY();
    marker.pose.orientation.z = orientation.getZ();

    marker.mesh_resource = object_display;

    vicon_object_pose_publisher_.publish(marker);
}
