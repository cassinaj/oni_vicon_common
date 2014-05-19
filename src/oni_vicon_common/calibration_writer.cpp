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

#include <boost/filesystem.hpp>

#include <iostream>
#include <fstream>

#include "oni_vicon_common/calibration_writer.hpp"

using namespace oni_vicon;

bool CalibrationWriter::saveGlobalCalibration(const std::string& destination,
                                              const GlobalCalibration& global_calibration)
{
    YAML::Emitter doc;
    doc.SetIndent(2);
    try
    {
        doc << YAML::BeginMap;
        globalCalibrationToYaml(global_calibration, doc);
        doc << YAML::EndMap;
    }
    catch(...)
    {
        return false;
    }

    return saveCalibration(destination, doc);
}

bool CalibrationWriter::saveLocalCalibration(const std::string& destination,
                                             const LocalCalibration& local_calibration)
{
    YAML::Emitter doc;
    doc.SetIndent(2);
    try
    {
        doc << YAML::BeginMap;
        localCalibrationToYaml(local_calibration, doc);
        doc << YAML::EndMap;
    }
    catch(...)
    {
        return false;
    }

    return saveCalibration(destination, doc);
}

void CalibrationWriter::globalCalibrationToYaml(const GlobalCalibration& global_calibration,
                                                YAML::Emitter &doc)
{
    doc << YAML::Key << "global_calibration" << YAML::Value
           << YAML::BeginMap
           << YAML::Key << "camera_intrinsics"
           << YAML::Value << global_calibration.cameraIntrinsics()
           << YAML::Key << "vicon_global_frame"
           << YAML::Value << global_calibration.viconToCameraTransform()
           << YAML::EndMap;
}

void CalibrationWriter::localCalibrationToYaml(const LocalCalibration& local_calibration,
                                               YAML::Emitter &doc)
{
    doc << YAML::Key << "local_calibration" << YAML::Value
           << YAML::BeginMap
           << YAML::Key << "object_mesh"
           << YAML::Value << local_calibration.object()
           << YAML::Key << "object_mesh_display"
           << YAML::Value << local_calibration.objectDisplay()
           << YAML::Key << "vicon_local_to_object_local_frame"
           << YAML::Value << local_calibration.viconLocalToCameraLocal()
           << YAML::EndMap;
}

bool CalibrationWriter::saveCalibration(const std::string& destination, const YAML::Emitter& doc)
{
    std::ofstream calibration_file;
    boost::filesystem::path dir(destination);
    boost::filesystem::create_directories(dir.remove_filename());

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
