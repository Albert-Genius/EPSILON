#ifndef _CORE_EUDM_PLANNER_INC_EUDM_PLANNER_EUDM_ITF_H_
#define _CORE_EUDM_PLANNER_INC_EUDM_PLANNER_EUDM_ITF_H_

#include <string>
#include <unordered_map>
#include <vector>

namespace planning {
namespace eudm {

struct LaneChangeInfo {
  bool forbid_lane_change_left = false; // 禁止左变道
  bool forbid_lane_change_right = false; // 禁止右变道
  bool lane_change_left_unsafe_by_occu = false;
  bool lane_change_right_unsafe_by_occu = false;
  bool left_solid_lane = false;
  bool right_solid_lane = false;
  bool recommend_lc_left = false;
  bool recommend_lc_right = false;
};

struct Task {
  bool is_under_ctrl = false;
  double user_desired_vel; /* 用户设置的期望速度 */
  int user_preferred_behavior = 0; /* 用户拨杆行为 */
  LaneChangeInfo lc_info;
};

}  // namespace eudm
}  // namespace planning

#endif
