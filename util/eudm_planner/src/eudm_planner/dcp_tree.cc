/**
 * @file behavior_tree.cc
 * @author HKUST Aerial Robotics Group
 * @brief
 * @version 0.1
 * @date 2019-07-07
 *
 * @copyright Copyright (c) 2019
 *
 */

#include "eudm_planner/dcp_tree.h"

namespace planning {
DcpTree::DcpTree(const int& tree_height, const decimal_t& layer_time)
    : tree_height_(tree_height), layer_time_(layer_time) {
  last_layer_time_ = layer_time_;
  GenerateActionScript();
}

DcpTree::DcpTree(const int& tree_height, const decimal_t& layer_time,
                 const decimal_t& last_layer_time)
    : tree_height_(tree_height),
      layer_time_(layer_time),
      last_layer_time_(last_layer_time) {
  GenerateActionScript(); // 初始化时创建原始action script
}

ErrorType DcpTree::UpdateScript() { return GenerateActionScript(); }

std::vector<DcpTree::DcpAction> DcpTree::AppendActionSequence(
    const std::vector<DcpAction>& seq_in, const DcpAction& a,
    const int& n) const 
{
  std::vector<DcpAction> seq = seq_in;
  for (int i = 0; i < n; ++i) {
    seq.push_back(a);
  }
  return seq;
}

ErrorType DcpTree::GenerateActionScript() {
  action_script_.clear();
  std::vector<DcpAction> ongoing_action_seq;
  for (int lon = 0; lon < static_cast<int>(DcpLonAction::MAX_COUNT); lon++) { //外层遍历纵向动态空间
    ongoing_action_seq.clear();
    // 基于当前横向决策在每个cycle Rebuild DCP-Tree
    ongoing_action_seq.push_back(DcpAction(DcpLonAction(lon), ongoing_action_.lat, ongoing_action_.t));

    for (int h = 1; h < tree_height_; ++h) { // tree_height_即在一个planning horizon中,允许决策推演多少步
      for (int lat = 0; lat < static_cast<int>(DcpLatAction::MAX_COUNT); lat++) { // 遍历横向动作空间

        // 如果在树的第一层就发生了横向行为的改变,那么这个树枝的横向行为**不再改变**
        // DCP Tree 对动作空间的剪枝策略
        if (lat != static_cast<int>(ongoing_action_.lat)) {
          auto actions = AppendActionSequence( ongoing_action_seq/*未修改该变量*/,
                          DcpAction(DcpLonAction(lon), DcpLatAction(lat), layer_time_), tree_height_ - h);

          action_script_.push_back(actions); 
        } else {}
      }
      // 实际对应的是横向行为未发生改变的情况,直接将树的深度加深一层,作为下次横向循环子树生长的树根 
      ongoing_action_seq.push_back(DcpAction(DcpLonAction(lon), ongoing_action_.lat, layer_time_));
    }

    action_script_.push_back(ongoing_action_seq);
  }

  // override the last layer time
  for (auto& action_seq : action_script_) {
    action_seq.back().t = last_layer_time_;
  }
  return kSuccess;
}

decimal_t DcpTree::planning_horizon() const {
  if (action_script_.empty()) return 0.0;
  decimal_t planning_horizon = 0.0;
  for (const auto& a : action_script_[0]) {
    planning_horizon += a.t;
  }
  return planning_horizon;
}

}  // namespace planning
