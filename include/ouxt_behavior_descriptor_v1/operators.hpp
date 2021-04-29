//
// Created by hans on 2021/04/29.
//

#ifndef OUXT_BEHAVIOR_DESCRIPTOR_V1_OPERATORS_HPP
#define OUXT_BEHAVIOR_DESCRIPTOR_V1_OPERATORS_HPP


void operator(const YAML::Node &node, Format &format) {
node["behavior"] >> Format.behavior;
const YAML::Node& objects_node = node["objects"];
for(auto object _node: objects_node){
Object object;
object_node >> object;
format.objects.push_back(object);
}
}

void operator>>(const YAML::Node &node, Object &object) {
  node["uuid"] >> object.uuid;
  const YAML::Node &attributes_node = node["attributes"];
  for (auto attribute_node : attributes_node) {
    Attribute attribute;
    attribute_node >> attribute;
    object.attributes.push_back(attribute);
  }
  node["pose"] >> object.pose;
}
void operator>>(const YAML::Node &node, Pose &pose) {
  node["position"] >> pose.position;
  node["orientation"] >> pose.orientation;
}
void operator>>(const YAML::Node &node,Position &position) {
  node["x"] >> position.x;
  node["y"] >> position.y;
  node["z"] >> position.z;
}
void operator>>(const YAML::Node &node,Quaternion &quaternion) {
  node["x"] >> quaternion.x;
  node["y"] >> quaternion.y;
  node["z"] >> quaternion.z;
  node["w"] >> quaternion.w;
}

void operator>>(const YAML::Node &node,Behavior &behavior) {
  node["description"] >>behavior.description;
  const YAML::Node &blackboards_node = node["blackboard"];
  for (auto blackboard_node : blackboards_node){
    BlackBoard blackboard;
    blackboard_node >> blackboard;
    behavior.blackboard.push_back(blackboard);
  }
}

void operator>>(const YAML::Node &node,BlackBoard &blackBoard) {
  node["input"] >> blackboard.input;
  node["eval"] >> blackboard.eval;
}

#endif // OUXT_BEHAVIOR_DESCRIPTOR_V1_OPERATORS_HPP
