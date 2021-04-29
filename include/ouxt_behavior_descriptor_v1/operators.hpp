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

void operator(const YAML::Node & node, Format & format) {
  node["behavior"] >> Format.behavior;
  const YAML::Node & objects_node = node["objects"];
  for (auto object _node : objects_node) {
    Object object;
    object_node >> object;
    format.objects.push_back(object);
  }
}

void operator>>(const YAML::Node & node, Object & object)
{
  node["uuid"] >> object.uuid;
  const YAML::Node & attributes_node = node["attributes"];
  for (auto attribute_node : attributes_node) {
    Attribute attribute;
    attribute_node >> attribute;
    object.attributes.push_back(attribute);
  }
  node["pose"] >> object.pose;
}
void operator>>(const YAML::Node & node, Pose & pose)
{
  node["position"] >> pose.position;
  node["orientation"] >> pose.orientation;
}
void operator>>(const YAML::Node & node, Position & position)
{
  node["x"] >> position.x;
  node["y"] >> position.y;
  node["z"] >> position.z;
}
void operator>>(const YAML::Node & node, Quaternion & quaternion)
{
  node["x"] >> quaternion.x;
  node["y"] >> quaternion.y;
  node["z"] >> quaternion.z;
  node["w"] >> quaternion.w;
}

void operator>>(const YAML::Node & node, Behavior & behavior)
{
  node["description"] >> behavior.description;
  const YAML::Node & blackboards_node = node["blackboard"];
  for (auto blackboard_node : blackboards_node) {
    BlackBoard blackboard;
    blackboard_node >> blackboard;
    behavior.blackboard.push_back(blackboard);
  }
}

void operator>>(const YAML::Node & node, BlackBoard & blackboard)
{
  node["input"] >> blackboard.input;
  node["eval"] >> blackboard.eval;
}

#endif  // OUXT_BEHAVIOR_DESCRIPTOR_V1__OPERATORS_HPP_
