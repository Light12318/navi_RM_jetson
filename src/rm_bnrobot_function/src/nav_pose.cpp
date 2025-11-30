#include <memory>
#include "nav2_msgs/action/navigate_to_pose.hpp"  
#include "rclcpp/rclcpp.hpp"  
#include "rclcpp_action/rclcpp_action.hpp"  

using NavigationAction = nav2_msgs::action::NavigateToPose;  
using NavActGoalHandle =rclcpp_action::ClientGoalHandle<NavigationAction>;  

class NavToPoseClient : public rclcpp::Node {
 public: 
  rclcpp_action::Client<NavigationAction>::SharedPtr action_client_;

  NavToPoseClient() : Node("nav_to_pose_client") {
    //客户端
    action_client_ = rclcpp_action::create_client<NavigationAction>(
        this, "navigate_to_pose");
    this->declare_parameter("goal.frame_id","map");
    this->declare_parameter("goal.x",2.0f);
    this->declare_parameter("goal.y",2.0f);
    //this->declare_parameter("goal.yaw",0.0f);
  }

  void sendGoal() {
    auto send_goal_options =
        rclcpp_action::Client<NavigationAction>::SendGoalOptions();
    while (!action_client_->wait_for_action_server(std::chrono::seconds(5))) {
      RCLCPP_INFO(get_logger(), "等待Action上线");
    }


    //目标点
    auto goal_msg = NavigationAction::Goal();
    this->get_parameter("goal.frame_id",goal_msg.pose.header.frame_id);
    this->get_parameter("goal.x",goal_msg.pose.pose.position.x);
    this->get_parameter("goal.y",goal_msg.pose.pose.position.y);
    this->get_parameter("goal.z", goal_msg.pose.pose.position.z);
    // float yaw;
    // this->get_parameter("goal.yaw",yaw);

    //目标
    send_goal_options.goal_response_callback =
        [this](NavActGoalHandle::SharedPtr goal_handle) {
          if (goal_handle) {
            RCLCPP_INFO(get_logger(), "目标已发送");
          }
        };
    //移动
    send_goal_options.feedback_callback =
        [this](
            NavActGoalHandle::SharedPtr goal_handle,
            const std::shared_ptr<const NavigationAction::Feedback> feedback) {
          (void)goal_handle;  
          RCLCPP_INFO(this->get_logger(), "剩余距离:%f",
                      feedback->distance_remaining);
        };

    send_goal_options.result_callback =
        [this](const NavActGoalHandle::WrappedResult& result) {
          if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
            RCLCPP_INFO(this->get_logger(), "到达目的地");
          }
        };

    action_client_->async_send_goal(goal_msg, send_goal_options);
  }


};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<NavToPoseClient>();
  node->sendGoal();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}