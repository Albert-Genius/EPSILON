/**
 * @file eudm_server_ros.cc
 * @author HKUST Aerial Robotics Group
 * @brief
 * @version 0.1
 * @date 2019-07-07
 *
 * @copyright Copyright (c) 2019
 *
 */

#include "eudm_planner/eudm_server_ros.h"

namespace planning {

EudmPlannerServer::EudmPlannerServer(ros::NodeHandle nh, int ego_id)
    : nh_(nh), work_rate_(20.0), ego_id_(ego_id) {
  p_visualizer_ = new EudmPlannerVisualizer(nh, &bp_manager_, ego_id);
  p_input_smm_buff_ = new moodycamel::ReaderWriterQueue<SemanticMapManager>(
      config_.kInputBufferSize);
  task_.user_preferred_behavior = 0; // 用户指定的变道行为
}

EudmPlannerServer::EudmPlannerServer(ros::NodeHandle nh, double work_rate,
                                     int ego_id)
    : nh_(nh), work_rate_(work_rate), ego_id_(ego_id) 
{
  p_visualizer_ = new EudmPlannerVisualizer(nh, &bp_manager_, ego_id);
  p_input_smm_buff_ = new moodycamel::ReaderWriterQueue<SemanticMapManager>(
      config_.kInputBufferSize);
  task_.user_preferred_behavior = 0;
}

void EudmPlannerServer::PushSemanticMap(const SemanticMapManager &smm) {
  if (p_input_smm_buff_) p_input_smm_buff_->try_enqueue(smm);
}

void EudmPlannerServer::PublishData() 
{
  p_visualizer_->PublishDataWithStamp(ros::Time::now());
}

void EudmPlannerServer::Init(const std::string &bp_config_path) 
{
  bp_manager_.Init(bp_config_path, work_rate_);
  joy_sub_ = nh_.subscribe("/joy", 10, &EudmPlannerServer::JoyCallback, this);
  //nh_.param("use_sim_state", use_sim_state_, true);//这个变量实际没有使用
  p_visualizer_->Init();
  //p_visualizer_->set_use_sim_state(use_sim_state_); //这个变量实际没有使用
}


/* 拨杆变道 */
void EudmPlannerServer::JoyCallback(const sensor_msgs::Joy::ConstPtr &msg)
{
  int msg_id; // 这里的frame id用于标识车辆 id
  if (std::string("").compare(msg->header.frame_id) == 0) {
    msg_id = 0;
  } else {
    msg_id = std::stoi(msg->header.frame_id);
  }
  if (msg_id != ego_id_) 
    return;

  // ~ buttons[2] --> 1 -->  lcl   @对应键盘上的"A"或者向左的箭头
  // ~ buttons[1] --> 1 -->  lcr   @对应键盘上的"D"或者向右的箭头
  // ~ buttons[3] --> 1 -->  +1m/s @对应键盘上的"W"或者向上的箭头
  // ~ buttons[0] --> 1 -->  -1m/s @对应键盘上的"S"或者向下的箭头
  // ~ buttons[4] --> 1 -->  forbid left lane change @对应键盘上的"Q"
  // ~ buttons[5] --> 1 -->  forbid right lane change @对应键盘上的"E"
  // ~ buttons[6] --> 1 -->  reset @对应键盘上的"R"
  if (msg->buttons[0] == 0 && msg->buttons[1] == 0 && msg->buttons[2] == 0 &&
      msg->buttons[3] == 0 && msg->buttons[4] == 0 && msg->buttons[5] == 0 &&
      msg->buttons[6] == 0)
    return;

  if (msg->buttons[2] == 1) { //左变道
    if (task_.user_preferred_behavior != -1) {
      task_.user_preferred_behavior = -1; // -1 代表左变道
    } else {
      task_.user_preferred_behavior = 0;
    }
  } else if (msg->buttons[1] == 1) { //右变道
    if (task_.user_preferred_behavior != 1) {
      task_.user_preferred_behavior = 1; // 1 代表右变道
    } else {
      task_.user_preferred_behavior = 0;
    }
  } else if (msg->buttons[3] == 1) {
    task_.user_desired_vel = task_.user_desired_vel + 1.0; // 加速 TODO:(@yuandong.zhao) 应该加一个最大限速
  } else if (msg->buttons[0] == 1) {
    task_.user_desired_vel = std::max(task_.user_desired_vel - 1.0, 0.0); // 减速
  } else if (msg->buttons[4] == 1) {
    task_.lc_info.forbid_lane_change_left = !task_.lc_info.forbid_lane_change_left; //禁止左变道
  } else if (msg->buttons[5] == 1) {
    task_.lc_info.forbid_lane_change_right = !task_.lc_info.forbid_lane_change_right; //禁止右变道
  } else if (msg->buttons[6] == 1) {
    task_.is_under_ctrl = !task_.is_under_ctrl; //是否自动控制
  }
}

void EudmPlannerServer::Start() 
{
  // 将this作为入参的作用是,创建的线程可以访问到当前对象中的成员变量
  std::thread(&EudmPlannerServer::MainThread, this).detach(); // 创建独立线程
  task_.is_under_ctrl = true; // 将标记设置为自动控制
}

void EudmPlannerServer::MainThread() 
{
  using namespace std::chrono;
  system_clock::time_point current_start_time{system_clock::now()};
  system_clock::time_point next_start_time{current_start_time};
  const milliseconds interval{static_cast<int>(1000.0 / work_rate_)}; // 决策周期为50ms

  // main loop
  while (true) {
    current_start_time = system_clock::now();
    next_start_time = current_start_time + interval;
    PlanCycleCallback();
    std::this_thread::sleep_until(next_start_time); //TODO:(@yuandong.zhao) 这种写法,运行周期要比interval长;不符合实时性要求,需要优化
  }
}

void EudmPlannerServer::PlanCycleCallback() 
{
  if (p_input_smm_buff_ == nullptr) return;

  // 从无锁队列中取最新的数据,丢弃老数据
  bool has_updated_map = false;
  while (p_input_smm_buff_->try_dequeue(smm_)) {
    has_updated_map = true;
  }
  if (!has_updated_map) return;

  // 运行时注入地图依赖
  auto map_ptr = std::make_shared<semantic_map_manager::SemanticMapManager>(smm_); // map的处理也比较核心,细节很多;和预测&决策交互很多

  decimal_t replan_duration = 1.0 / work_rate_; // 决策周期为50ms,即1秒内进行20次决策
  double stamp = std::floor(smm_.time_stamp() / replan_duration) * replan_duration; // stamp对齐到决策周期(对duration取整)

  // 核心调用:行为决策
  if (bp_manager_.Run(stamp, map_ptr, task_) == kSuccess) {
    common::SemanticBehavior behavior;
    bp_manager_.ConstructBehavior(&behavior);
    smm_.set_ego_behavior(behavior); // 将决策结果传递给地图管理器,再传递给ssc planner
  } //TODO:(@yuandong.zhao) 如果决策失败,应该怎么处理?是否需要重试?是否需要发布一个错误消息?

  // 将结果传递给ssc planner
  if (has_callback_binded_) {
    private_callback_fn_(smm_);
  }

  PublishData();
}

void EudmPlannerServer::BindBehaviorUpdateCallback(
    std::function<int(const SemanticMapManager &)> fn) {
  private_callback_fn_ = std::bind(fn, std::placeholders::_1);
  has_callback_binded_ = true;
}

void EudmPlannerServer::set_user_desired_velocity(const decimal_t desired_vel) {
  task_.user_desired_vel = desired_vel;
}

decimal_t EudmPlannerServer::user_desired_velocity() const {
  return task_.user_desired_vel;
}

}  // namespace planning
