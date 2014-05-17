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

// c++/std
#include <iostream>
#include <fstream>

// boost
#include <boost/format.hpp>

#include "oni_vicon_common/exceptions.hpp"
#include "oni_vicon_common/calibration_reader.hpp"

using namespace oni_vicon;

void CalibrationReader::loadGlobalCalibration(const std::string& source,
                                              GlobalCalibration& global_calibration)
{
    try
    {
        YAML::Node doc;
        loadCalibration(source, doc);
        globalCalibrationFromYaml(doc, global_calibration);
    }
    catch(YAML::ParserException& e)
    {
        throw LoadingCalibrationException(
                    (boost::format("Loading global calibration file <%s> failed: %s")
                     % source
                     % e.what()).str());
    }
}

void CalibrationReader::loadLocalCalibration(const std::string& source,
                                             LocalCalibration& local_calibration)
{
    try
    {
        YAML::Node doc;
        loadCalibration(source, doc);
        localCalibrationFromYaml(doc, local_calibration);
    }
    catch(YAML::ParserException& e)
    {
        throw LoadingCalibrationException(
                    (boost::format("Loading local calibration file <%s> failed: %s")
                     % source
                     % e.what()).str());
    }
}

void CalibrationReader::globalCalibrationFromYaml(const YAML::Node& doc,
                                                  GlobalCalibration& global_calibration)
{
    tf::Transform global_transform;
    CameraIntrinsics camera_intrinsics;

    doc["global_calibration"]["vicon_global_frame"] >> global_transform;
    doc["global_calibration"]["camera_intrinsics"] >> camera_intrinsics;

    global_calibration.viconToCameraTransform(global_transform);
    global_calibration.cameraIntrinsics(camera_intrinsics);
}

void CalibrationReader::localCalibrationFromYaml(const YAML::Node& doc,
                                                 LocalCalibration& local_calibration)
{
    tf::Transform local_transform;
    std::string object;
    std::string object_display;

    doc["local_calibration"]["vicon_local_to_object_local_frame"] >> local_transform;
    doc["local_calibration"]["object_mesh"] >> object;
    doc["local_calibration"]["object_mesh_display"] >> object_display;

    local_calibration.viconLocalToCameraLocal(local_transform);
    local_calibration.object(object);
    local_calibration.objectDisplay(object_display);
}

bool CalibrationReader::loadCalibration(const std::string& source, YAML::Node& doc)
{
    std::ifstream calibration_file(source.c_str());
    YAML::Parser parser(calibration_file);
    parser.GetNextDocument(doc);
}
