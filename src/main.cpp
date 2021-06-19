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

#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
//  #include "ouxt_behavior_descriptor_v1/visibility.hpp"
#include "ouxt_behavior_descriptor_v1/data_structures.hpp"
#include "ouxt_behavior_descriptor_v1/operators.hpp"

#define SOL_ALL_SAFETIES_ON 1
#include <sol/sol.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

struct EvaluationBlock
{
  std::string evaluation;
  std::string result;
  void evaluate(sol::state &state)
  {
    // TODO(HansRobo)
  }
};
class Component : public rclcpp::Node
{
public:
  explicit Component(const rclcpp::NodeOptions & options)
  : rclcpp::Node("ouxt_behavior_descriptor_v1_node", options)
  {
    declare_parameter<std::string>("config_path", "test.xml");
    get_parameter("config_path", config_path_);
    declare_parameter<double>("update_rate", 60.0);
    get_parameter("update_rate", update_rate_);

    using namespace std::literals::chrono_literals;
    timer_ = create_wall_timer(1s, std::bind(&Component::evaluationCallback,this));
    config_path_ = ament_index_cpp::get_package_share_directory("ouxt_behavior_descriptor_v1") + "/test/example/test.yaml";
    initialize(config_path_);
  }
  ~Component() {}

  void initialize(std::string file_path)
  {
    node_ = YAML::LoadFile(file_path);
    node_ >> format_;

    lua_.open_libraries(sol::lib::base);
  }

  void evaluationCallback()
  {
    for (auto & block : evaluation_blocks_) {
      block.evaluate(lua_);
    }
  }

  std::string config_path_;
  float update_rate_;
  YAML::Node node_;
  Format format_;
  sol::state lua_;
  std::vector<EvaluationBlock> evaluation_blocks_;
  rclcpp::TimerBase::SharedPtr timer_;
};
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  auto component = std::make_shared<Component>(options);
  rclcpp::spin(component);
  rclcpp::shutdown();
  return 0;
}
