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

#include "oni_vicon_common/calibration_writer.hpp"


using namespace oni_vicon;

void localCalibrationFrom(const YAML::Node& doc)
{
    doc["local_calibration"]["vicon_local_to_object_local_frame"] >> local_transform_;
    doc["local_calibration"]["object_mesh"] >> object_;
    doc["local_calibration"]["object_mesh_display"] >> object_display_;
}

void globalCalibrationFrom(const YAML::Node& doc)
{
    const YAML::Node& global_calibration = doc["global_calibration"];
    const YAML::Node& camera_intrinsics = global_calibration["camera_intrinsics"];
    const YAML::Node& global_transform = global_calibration["vicon_global_frame"];

    global_transform >> global_transform_;
    camera_intrinsics >> camera_intrinsics_;
}

void localCalibrationTo(YAML::Emitter& doc) const
{
    doc << YAML::Key << "local_calibration" << YAML::Value
           << YAML::BeginMap
           << YAML::Key << "object_mesh" << YAML::Value << object_
           << YAML::Key << "object_mesh_display" << YAML::Value << object_display_
           << YAML::Key << "vicon_local_to_object_local_frame" << YAML::Value << local_transform_
           << YAML::EndMap;
}

void globalCalibrationTo(YAML::Emitter& doc) const
{
    doc << YAML::Key << "global_calibration" << YAML::Value
           << YAML::BeginMap
           << YAML::Key << "camera_intrinsics" << YAML::Value << camera_intrinsics_
           << YAML::Key << "vicon_global_frame" << YAML::Value << global_transform_
           << YAML::EndMap;
}


bool saveGlobalCalibration(const std::string& destination) const
{
    YAML::Emitter doc;
    doc.SetIndent(2);
    try
    {
        doc << YAML::BeginMap;
        globalCalibrationTo(doc);
        doc << YAML::EndMap;
    }
    catch(...)
    {
        return false;
    }

    return saveCalibration(destination, doc);
}

bool saveLocalCalibration(const std::string &destination) const
{
    YAML::Emitter doc;
    doc.SetIndent(2);
    try
    {
        doc << YAML::BeginMap;
        localCalibrationTo(doc);
        doc << YAML::EndMap;
    }
    catch(...)
    {
        return false;
    }

    return saveCalibration(destination, doc);
}

bool loadGlobalCalibration(const std::string& source)
{
    try
    {
        YAML::Node doc;
        loadCalibrationDoc(source, doc);
        globalCalibrationFrom(doc);
    }
    catch(YAML::ParserException& e) {
        std::cout << e.what() << "\n";
        return false;
    }

    return true;
}

bool loadLocalCalibration(const std::string &source)
{
    try
    {
        YAML::Node doc;
        loadCalibrationDoc(source, doc);
        localCalibrationFrom(doc);
    }
    catch(YAML::ParserException& e) {
        std::cout << e.what() << "\n";
        return false;
    }

    return true;
}

bool saveCalibration(const std::string& destination, const YAML::Emitter& doc) const
{
    std::ofstream calibration_file;
    boost::filesystem::path dir(destination);

    if (!boost::filesystem::create_directories(dir.remove_filename()))
    {
        // ignore, since it might exist already
        // return false;
    }

    calibration_file.open(destination.c_str());
    if (calibration_file.is_open())
    {
        ROS_INFO("Saving calibration\n%s", doc.c_str());
        calibration_file << doc.c_str();
        calibration_file.close();
        return true;
    }

    return false;
}

