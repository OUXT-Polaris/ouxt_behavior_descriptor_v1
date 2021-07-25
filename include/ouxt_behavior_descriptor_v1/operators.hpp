// Copyright (c) 2021, OUXT-Polaris
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef OUXT_BEHAVIOR_DESCRIPTOR_V1__OPERATORS_HPP_
#define OUXT_BEHAVIOR_DESCRIPTOR_V1__OPERATORS_HPP_

#include <string>
#include "yaml-cpp/yaml.h"
#include "ouxt_behavior_descriptor_v1/data_structures.hpp"


void operator>>(const YAML::Node & node, Position & position)
{
  try {
    position.x = node["x"].as<double>();
    position.y = node["y"].as<double>();
    position.z = node["z"].as<double>();
  } catch (...) {
    position.x = 0.0;
    position.y = 0.0;
    position.z = 0.0;
    std::cout << "parse error : Position" << std::endl;
  }
}
void operator>>(const YAML::Node & node, Quaternion & quaternion)
{
  try {
    quaternion.x = node["x"].as<double>();
    quaternion.y = node["y"].as<double>();
    quaternion.z = node["z"].as<double>();
    quaternion.w = node["w"].as<double>();
  } catch (...) {
    quaternion.x = 0.0;
    quaternion.y = 0.0;
    quaternion.z = 0.0;
    quaternion.w = 0.0;
    std::cout << "parse error : Quaternion" << std::endl;
  }
}

void operator>>(const YAML::Node & node, Pose & pose)
{
  try {
    node["position"] >> pose.position;
    node["orientation"] >> pose.orientation;
  } catch (...) {
    std::cout << "parse error : Pose" << std::endl;
  }
}

void operator>>(const YAML::Node & node, Object & object)
{
  try {
    object.uuid = node["uuid"].as<int>();
    const YAML::Node & attributes_node = node["attributes"];
    for (auto attribute_node : attributes_node) {
      std::string attribute = attribute_node.as<std::string>();
      object.attributes.push_back(attribute);
    }
    node["pose"] >> object.pose;
  } catch (...) {
    std::cout << "parse error : Object" << std::endl;
  }
}

void operator>>(const YAML::Node & node, BlackBoard & blackboard)
{
  try {
    blackboard.input = node["input"].as<std::string>();
    blackboard.eval = node["eval"].as<std::string>();
  } catch (...) {
    blackboard.input = "";
    blackboard.eval = "";
    std::cout << "parse error : BlackBoard" << std::endl;
  }
}

void operator>>(const YAML::Node & node, Behavior & behavior)
{
  try {
    behavior.description = node["description"].as<std::string>();
    const YAML::Node & blackboards_node = node["blackboard"];
    for (auto blackboard_node : blackboards_node) {
      BlackBoard blackboard;
      blackboard_node >> blackboard;
      behavior.blackboard.push_back(blackboard);
    }
  } catch (...) {
    behavior.description = "";
    std::cout << "parse error : Behavior" << std::endl;
  }
}

void operator>>(const YAML::Node & node, Format & format)
{
  try {
    node["behavior"] >> format.behavior;
    const YAML::Node & objects_node = node["objects"];
    for (auto object_node : objects_node) {
      Object object;
      object_node >> object;
      format.objects.push_back(object);
    }
  } catch (...) {
    std::cout << "parse error : Format" << std::endl;
  }
}

#endif  // OUXT_BEHAVIOR_DESCRIPTOR_V1__OPERATORS_HPP_
