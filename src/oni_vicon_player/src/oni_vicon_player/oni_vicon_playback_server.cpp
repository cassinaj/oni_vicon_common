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

// boost
#include <boost/filesystem.hpp>
#include <boost/function.hpp>

#include <oni_vicon_common/global_calibration.hpp>
#include <oni_vicon_common/local_calibration.hpp>
#include <oni_vicon_common/calibration_reader.hpp>

#include "oni_vicon_player/exceptions.hpp"
#include "oni_vicon_player/oni_vicon_playback_server.hpp"

using namespace oni_vicon;
using namespace oni_vicon_player;

OniViconPlayback::OniViconPlayback(OniPlayer::Ptr oni_player,
                                   ViconPlayer::Ptr vicon_player):
    oni_player_(oni_player),
    vicon_player_(vicon_player)
{
}

void OniViconPlayback::open(const std::string& record_dir,
                            ViconPlayer::LoadUpdateCallback update_cb)
{
    open_ = false;

    boost::filesystem::path record_path = record_dir;
    std::string recording_name = record_path.leaf().string();

    std::string oni_file = (record_path / (recording_name + ".oni")).string();
    std::string vicon_file = (record_path / (recording_name + ".txt")).string();
    std::string global_calib_file = (record_path / "global_calibration.yaml").string();
    std::string local_calib_file = (record_path / "local_calibration.yaml").string();

    CalibrationReader calibration_reader;
    LocalCalibration local_calibration;
    GlobalCalibration global_calibration;

    try
    {
        // load calibration
        calibration_reader.loadGlobalCalibration(global_calib_file, global_calibration);
        calibration_reader.loadLocalCalibration(local_calib_file, local_calibration);
        calibration_transform_.globalCalibration(global_calibration);
        calibration_transform_.localCalibration(local_calibration);

        // open oni file
        oni_player_->open(oni_file, calibration_transform_.cameraIntrinsics());

        update_cb(oni_player_->countFrames(), 0);

        // load vicon data
        vicon_player_->load(vicon_file, calibration_transform_, update_cb);
    }
    catch (Exception& e)
    {
        throw OpenRecordException(
                    (boost::format("Opening record <%s> failed: %s")
                     % record_dir
                     % e.what()).str());
    }

    open_ = true;
}

void OniViconPlayback::close()
{
    oni_player_->close();
    open_ = false;
    playing_ = false;
}

void OniViconPlayback::play(uint32_t starting_frame)
{
    playing_ = false;

    // start generating

    oni_player_->seekToFrame(starting_frame);

    // stop generating

    playing_ = true;
}

void OniViconPlayback::stop()
{
    playing_ = false;
}

uint32_t OniViconPlayback::nextFrame()
{
    oni_player_->processNextFrame();

    return oni_player_->currentFrameID();
}

void OniViconPlayback::seekToFrame(uint32_t starting_frame)
{
    oni_player_->seekToFrame(starting_frame);
}

void OniViconPlayback::setPlaybackSpeed(double speed)
{
}

bool OniViconPlayback::isEOF() const
{
    return oni_player_->isEOF();
}

bool OniViconPlayback::isOpen() const
{
    return open_;
}

bool OniViconPlayback::isPlaying() const
{
    return playing_;
}

OniPlayer::Ptr OniViconPlayback::oniPlayer()
{
    return oni_player_;
}

ViconPlayer::Ptr OniViconPlayback::viconPlayer()
{
    return vicon_player_;
}

const Transformer &OniViconPlayback::transformer() const
{
    return calibration_transform_;
}
