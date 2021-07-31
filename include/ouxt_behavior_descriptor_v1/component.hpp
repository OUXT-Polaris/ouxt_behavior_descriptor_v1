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


#ifndef OUXT_BEHAVIOR_DESCRIPTOR_V1__COMPONENT_HPP_
#define OUXT_BEHAVIOR_DESCRIPTOR_V1__COMPONENT_HPP_

#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
//  #include "ouxt_behavior_descriptor_v1/visibility.hpp"
#include "ouxt_behavior_descriptor_v1/data_structures.hpp"
#include "ouxt_behavior_descriptor_v1/operators.hpp"

#define SOL_ALL_SAFETIES_ON 1
#include "sol/sol.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"

struct EvaluationBlockBase
{
  std::string evaluation;
  virtual void evaluate(sol::state & state) = 0;
};
template<class T>
struct EvaluationBlock : EvaluationBlockBase
{
  T result;

  void evaluate(sol::state & state) override
  {
    std::string eval = "return " + evaluation;
    auto result = state.script(eval);
    if (result.valid()) {
      this->result = static_cast<T>(result);
      std::cout << "Evaluation : " << this->result << std::endl;
    } else {
      std::cout << "Evaluation " << evaluation << " has failed!" << std::endl;
    }
  }
};


class Component : public rclcpp::Node
{
public:
  explicit Component(const rclcpp::NodeOptions & options)
  : rclcpp::Node("ouxt_behavior_descriptor_v1_node", options)
  {
    declare_parameter<std::string>("config_package", "ouxt_behavior_descriptor_v1");
    get_parameter("config_package", config_package_);
    declare_parameter<std::string>("config_file", "test/example/test.yaml");
    get_parameter("config_file", config_file_);
    declare_parameter<double>("update_rate", 10.0);
    get_parameter("update_rate", update_rate_);

    using std::literals::chrono_literals::operator""s;
    auto interval = 1s / update_rate_;
    timer_ = create_wall_timer(interval, std::bind(&Component::evaluationCallback, this));
    std::string config_path =
      ament_index_cpp::get_package_share_directory(config_package_) + "/" + config_file_;
    initialize(config_path);
  }
  ~Component() {}

  void initialize(std::string file_path)
  {
    node_ = YAML::LoadFile(file_path);
    node_ >> format_;

    lua_.open_libraries(sol::lib::base);

    if (node_["behavior"]["blackboard"]) {
      for (auto board : node_["behavior"]["blackboard"]) {
        if (!board["eval"]) {continue;}
        // TODO(HansRobo) : make custome type evaluation
        auto evaluation = std::make_shared<EvaluationBlock<double>>();
        evaluation->evaluation = board["eval"].as<std::string>();
        this->evaluation_blocks_.emplace_back(evaluation);
      }
    }
  }

  void evaluationCallback()
  {
    for (auto & block : evaluation_blocks_) {
      block->evaluate(lua_);
    }
  }

  std::string config_file_;
  std::string config_package_;
  float update_rate_;
  YAML::Node node_;
  Format format_;
  sol::state lua_;
  std::vector<std::shared_ptr<EvaluationBlockBase>> evaluation_blocks_;
  rclcpp::TimerBase::SharedPtr timer_;
};

#endif  // OUXT_BEHAVIOR_DESCRIPTOR_V1__COMPONENT_HPP_
