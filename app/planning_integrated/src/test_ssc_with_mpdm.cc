/**
 * @file test_ssc_with_mpdm.cc
 * @author HKUST Aerial Robotics Group (lzhangbz@ust.hk)
 * @brief
 * @version 0.1
 * @date 2020-09-21
 * @copyright Copyright (c) 2020
 */
#include <ros/ros.h>
#include <stdlib.h>

#include <chrono>
#include <iostream>

#include "behavior_planner/behavior_server_ros.h"
#include "semantic_map_manager/data_renderer.h"
#include "semantic_map_manager/ros_adapter.h"
#include "semantic_map_manager/semantic_map_manager.h"
#include "semantic_map_manager/visualizer.h"
#include "ssc_planner/ssc_server_ros.h"

DECLARE_BACKWARD;
double ssc_planner_work_rate = 20.0; //20Hz,50ms
double bp_work_rate = 20.0; //20Hz, 50ms

planning::SscPlannerServer* p_ssc_server_{nullptr};
planning::BehaviorPlannerServer* p_bp_server_{nullptr};

int BehaviorUpdateCallback(const semantic_map_manager::SemanticMapManager& smm) 
{
  if (p_ssc_server_) 
    p_ssc_server_->PushSemanticMap(smm);

  return 0;
}

int SemanticMapUpdateCallback(const semantic_map_manager::SemanticMapManager& smm) 
{
  if (p_bp_server_) 
    p_bp_server_->PushSemanticMap(smm);

  return 0;
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "~");
  ros::NodeHandle nh("~");

  // 读取配置参数
  int ego_id;
  if (!nh.getParam("ego_id", ego_id)) {
    ROS_ERROR("Failed to get param %d", ego_id);
    assert(false);
  }
  std::string agent_config_path;
  if (!nh.getParam("agent_config_path", agent_config_path)) {
    ROS_ERROR("Failed to get param %s", agent_config_path.c_str());
    assert(false);
  }
  std::string ssc_config_path;
  if (!nh.getParam("ssc_config_path", ssc_config_path)) {
    ROS_ERROR("Failed to get param ssc_config_path %s",
              ssc_config_path.c_str());
    assert(false);
  }

  // 接收解析语义地图数据
  semantic_map_manager::SemanticMapManager semantic_map_manager(ego_id, agent_config_path);
  semantic_map_manager::RosAdapter smm_ros_adapter(nh, &semantic_map_manager); // 通过构造函数做依赖注入
  smm_ros_adapter.BindMapUpdateCallback(SemanticMapUpdateCallback); // 通过函数接口做依赖注入

  double desired_vel;
  nh.param("desired_vel", desired_vel, 6.0); // 读取期望速度配置参数，读取失败后默认为6
  
  // 获取和配置决策句柄
  p_bp_server_ = new planning::BehaviorPlannerServer(nh, bp_work_rate, ego_id);
  p_bp_server_->set_user_desired_velocity(desired_vel);
  p_bp_server_->BindBehaviorUpdateCallback(BehaviorUpdateCallback);
  p_bp_server_->set_autonomous_level(3);
  p_bp_server_->enable_hmi_interface();

  // 获取和配置规划句柄
  p_ssc_server_ = new planning::SscPlannerServer(nh, ssc_planner_work_rate, ego_id);
  p_ssc_server_->Init(ssc_config_path);
  p_bp_server_->Init();
  smm_ros_adapter.Init();

  // 启动决策线程和规划线程
  p_bp_server_->Start();
  p_ssc_server_->Start();

  // TicToc timer;
  ros::Rate rate(100);
  while (ros::ok()) {
    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}
