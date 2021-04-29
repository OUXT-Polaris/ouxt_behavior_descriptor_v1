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

#include "yaml-cpp/yaml.h"
#include "ouxt_behavior_descriptor_v1/data_structures.hpp"


void operator>>(const YAML::Node & node, Position & position)
{
  position.x = node["x"].as<double>();
  position.y = node["y"].as<double>();
  position.z = node["z"].as<double>();
}
void operator>>(const YAML::Node & node, Quaternion & quaternion)
{
  quaternion.x = node["x"].as<double>();
  quaternion.y = node["y"].as<double>();
  quaternion.z = node["z"].as<double>();
  quaternion.w = node["w"].as<double>();
}

void operator>>(const YAML::Node & node, Pose & pose)
{
  node["position"] >> pose.position;
  node["orientation"] >> pose.orientation;
}

void operator>>(const YAML::Node & node, Object & object)
{
  object.uuid = node["uuid"].as<int>();
  const YAML::Node & attributes_node = node["attributes"];
  for (auto attribute_node : attributes_node) {
    std::string attribute = attribute_node.as<std::string>();
    object.attributes.push_back(attribute);
  }
  node["pose"] >> object.pose;
}

void operator>>(const YAML::Node & node, BlackBoard & blackboard)
{
  blackboard.input = node["input"].as<std::string>();
  blackboard.eval = node["eval"].as<std::string>();
}

void operator>>(const YAML::Node & node, Behavior & behavior)
{
  behavior.description = node["description"].as<std::string>();
  const YAML::Node & blackboards_node = node["blackboard"];
  for (auto blackboard_node : blackboards_node) {
    BlackBoard blackboard;
    blackboard_node >> blackboard;
    behavior.blackboard.push_back(blackboard);
  }
}

void operator>>(const YAML::Node & node, Format & format) {
  node["behavior"] >> format.behavior;
  const YAML::Node & objects_node = node["objects"];
  for (auto object_node : objects_node) {
    Object object;
    object_node >> object;
    format.objects.push_back(object);
  }
}

#endif  // OUXT_BEHAVIOR_DESCRIPTOR_V1__OPERATORS_HPP_
