#include "can_twist/usb_can_v2.hpp"
#include "can_twist/game_state.hpp"
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rm_interfaces/msg/game_para.hpp"
#include "rm_interfaces/msg/own_para.hpp"
#include "rm_interfaces/msg/ally_para.hpp"
#include <iostream>
#include <cmath>
#include <algorithm>
#include <chrono>
#include <atomic>

using namespace std;

// Forward declaration
class CanTwistNode;

// Global node pointer for static callback bridge
CanTwistNode* g_can_twist_node = nullptr;

class CanTwistNode : public rclcpp::Node {
public:
    CanTwistNode() : rclcpp::Node("can_twist_node") {
        twist_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "cmd_vel", 10, std::bind(&CanTwistNode::twist_callback, this, std::placeholders::_1));
        
        // Create publishers for referee system data
        game_para_pub_ = this->create_publisher<rm_interfaces::msg::GamePara>("game_para", 10);
        own_para_pub_ = this->create_publisher<rm_interfaces::msg::OwnPara>("own_para", 10);
        ally_para_pub_ = this->create_publisher<rm_interfaces::msg::AllyPara>("ally_para", 10);
        
        usb_can_ = new usb_can_v2();
        
        // Register CAN receivers for referee system messages
        usb_can_->listen(0x153, static_game_state_callback);  // Game arena state
        usb_can_->listen(0x155, static_robot_state_callback);  // Robot state
        
        // Initialize spin dodge tracking
        spin_dodge_active_ = false;
        spin_dodge_start_time_ = std::chrono::steady_clock::now();
        last_cmd_vel_time_ = std::chrono::steady_clock::now() - std::chrono::seconds(1);  // Init to 1s ago
        last_hp_deduction_reason_ = static_cast<uint8_t>(HPDeductionReason::ARMOR_ATTACKED);
        last_hp_ = 0;  // Initialize last HP
        
        // Timer for independent spin dodge control (100 Hz)
        // This ensures spin dodge continues even if /cmd_vel is not publishing
        spin_dodge_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(10),
            std::bind(&CanTwistNode::spin_dodge_timer_callback, this));
        
        g_can_twist_node = this;

        RCLCPP_INFO(this->get_logger(), "CAN Twist Node Started");
    }

    ~CanTwistNode() {
        delete usb_can_;
        g_can_twist_node = nullptr;
    }

private:
    // Static callback bridges for libusb C callbacks
    static void static_game_state_callback(uint16_t id, uint8_t* data, int length) {
        if (g_can_twist_node != nullptr && id == 0x153 && length >= 8) {
            g_can_twist_node->handle_game_state(data, length);
        }
    }

    static void static_robot_state_callback(uint16_t id, uint8_t* data, int length) {
        if (g_can_twist_node != nullptr && id == 0x155 && length >= 8) {
            g_can_twist_node->handle_robot_state(data, length);
        }
    }

    /**
     * @brief Process 0x153 game state message from referee system
     */
    void handle_game_state(uint8_t* data, int /* length */) {
        game_para_.parse_0x153(data);
        
        // Publish GamePara message to ROS 2
        auto msg = rm_interfaces::msg::GamePara();
        msg.game_progress = game_para_.game_progress;
        msg.stage_remain_time = game_para_.stage_remain_time;
        msg.recovery_occupy = game_para_.recovery_occupy;
        msg.mid_occupy = game_para_.mid_occupy;
        msg.recovery_rfid = game_para_.recovery_rfid;
        msg.mid_rfid = game_para_.mid_rfid;
        game_para_pub_->publish(msg);
        
        /*RCLCPP_INFO(this->get_logger(), 
            "Received 0x153 Game State: progress=%d, remain_time=%d, HP_reason=%d, power_output=%d",
            game_para_.game_progress, game_para_.stage_remain_time,
            game_para_.HP_deduction_reason, game_para_.power_management_shooter_output);*/
    }

    /**
     * @brief Process 0x155 robot state message from referee system
     */
    void handle_robot_state(uint8_t* data, int length) {
        own_para_.parse_0x155(data);
        
        if (length >= 8) {
            ally_para_.parse_0x155_extended(data);
        }
        
        // Check for HP deduction (armor attack or health drop)
        own_para_.HP_deduction_reason = game_para_.HP_deduction_reason;
        own_para_.power_management_shooter_output = game_para_.power_management_shooter_output;
        own_para_.recovery_buff = game_para_.recovery_buff;
        
        // Publish OwnPara message to ROS 2
        auto own_msg = rm_interfaces::msg::OwnPara();
        own_msg.current_hp = own_para_.current_HP;
        own_msg.maximum_hp = own_para_.maximum_HP;
        own_msg.hp_deduction_reason = own_para_.HP_deduction_reason;
        own_msg.recovery_rfid = own_para_.recovery_rfid;
        own_msg.mid_rfid = own_para_.mid_rfid;
        own_msg.power_management_shooter_output = own_para_.power_management_shooter_output;
        own_msg.recovery_buff = own_para_.recovery_buff;
        own_msg.shooter_remain = own_para_.shooter_remain;
        own_para_pub_->publish(own_msg);
        
        // Publish AllyPara message to ROS 2
        auto ally_msg = rm_interfaces::msg::AllyPara();
        ally_msg.ally_1_robot_hp = ally_para_.ally_1_robot_HP;
        ally_msg.ally_3_robot_hp = ally_para_.ally_3_robot_HP;
        ally_msg.level = ally_para_.level;
        ally_msg.offending_robot_id = ally_para_.offending_robot_id;
        ally_para_pub_->publish(ally_msg);
        
        float health_ratio = own_para_.get_health_ratio();
        
        // Trigger spin dodge if:
        // HP decreased and deduction reason is ARMOR_ATTACKED (0)
        if (!spin_dodge_active_ && 
            ((own_para_.current_HP < last_hp_ && 
             own_para_.HP_deduction_reason == static_cast<uint8_t>(HPDeductionReason::ARMOR_ATTACKED))
                /*|| health_ratio <= HEALTH_THRESHOLD*/)) {
            activate_spin_dodge();
        }
        
        // Update last HP
        last_hp_ = own_para_.current_HP;
        
        /*RCLCPP_INFO(this->get_logger(), 
            "Received 0x155 Robot State: HP=%d/%d (%.1f%%), reason=%d, shooter=%d",
            own_para_.current_HP, own_para_.maximum_HP, health_ratio * 100,
            own_para_.HP_deduction_reason, own_para_.power_management_shooter_output);*/
    }

    /**
     * @brief Activate 10-second spin dodge maneuver
     */
    void activate_spin_dodge() {
        spin_dodge_active_ = true;
        spin_dodge_start_time_ = std::chrono::steady_clock::now();
        RCLCPP_WARN(this->get_logger(), "SPIN DODGE ACTIVATED! Duration: 10s");
    }

    /**
     * @brief Check if spin dodge should remain active
     * @return true if spin dodge is still active, false if timed out or inactive
     */
    bool is_spin_dodge_active() {
        if (!spin_dodge_active_) return false;
        
        auto elapsed = std::chrono::steady_clock::now() - spin_dodge_start_time_;
        auto elapsed_sec = std::chrono::duration_cast<std::chrono::seconds>(elapsed).count();
        
        if (elapsed_sec >= 10) {
            // Spin dodge timeout - deactivate it
            spin_dodge_active_ = false;
            RCLCPP_WARN(this->get_logger(), "SPIN DODGE STOPPED - Timeout (10s)");
            return false;
        }
        return true;
    }

    /**
     * @brief Independent spin dodge control timer callback
     * 
     * Handles spin dodge when /cmd_vel is not publishing:
     * - If spin dodge is active and NO recent /cmd_vel (>= 100ms): send (0, 0, wz)
     * - If spin dodge is NOT active and NO recent /cmd_vel: send (0, 0, 0) to stop
     * - If /cmd_vel is publishing: twist_callback will handle everything, timer does nothing
     */
    void spin_dodge_timer_callback() {
        // Check if we've received /cmd_vel recently (within last 100ms)
        auto time_since_last_cmd_vel = std::chrono::steady_clock::now() - last_cmd_vel_time_;
        auto ms_since_cmd_vel = std::chrono::duration_cast<std::chrono::milliseconds>(
            time_since_last_cmd_vel).count();
        
        bool has_recent_cmd_vel = (ms_since_cmd_vel < 100);
        
        if (has_recent_cmd_vel) {
            // cmd_vel is being published, twist_callback will handle it
            RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                "Timer: cmd_vel is active, skipping timer control");
            return;
        }
        
        // No recent cmd_vel, we need to send motion command
        const float LINEAR_SCALE = 500.0f;
        const float ANGULAR_SCALE = 500.0f;
        const float SPIN_DODGE_ANGULAR_VELOCITY = 1.0f;
        
        float vx = 0.0f;
        float vy = 0.0f;
        float wz = 0.0f;
        
        // If spin dodge is active, send rotation; otherwise send stop command
        if (is_spin_dodge_active()) {
            wz = SPIN_DODGE_ANGULAR_VELOCITY;
            RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 500,
                "Timer: SPIN DODGE active, sending (0, 0, %.1f)", SPIN_DODGE_ANGULAR_VELOCITY);
        } else {
            RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 500,
                "Timer: No cmd_vel and no spin dodge, sending stop (0, 0, 0)");
        }
        
        int16_t vx_int = static_cast<int16_t>(std::round(vx * LINEAR_SCALE));
        int16_t vy_int = static_cast<int16_t>(std::round(vy * LINEAR_SCALE));
        int16_t wz_int = static_cast<int16_t>(std::round(wz * ANGULAR_SCALE));
        int16_t power_int = 100;
        
        uint8_t send_data[8] = {0};
        send_data[1] = (vx_int >> 8) & 0xFF; 
        send_data[0] = vx_int & 0xFF;        
        send_data[3] = (vy_int >> 8) & 0xFF; 
        send_data[2] = vy_int & 0xFF;        
        send_data[5] = (wz_int >> 8) & 0xFF; 
        send_data[4] = wz_int & 0xFF;        
        send_data[7] = (power_int >> 8) & 0xFF; 
        send_data[6] = power_int & 0xFF;
        
        usb_can_->transmit(0x120, send_data, 8);
        RCLCPP_DEBUG(this->get_logger(), "Sent 0x120: vx=%d, vy=%d, wz=%d", vx_int, vy_int, wz_int);
    }
    void twist_callback(const geometry_msgs::msg::Twist::SharedPtr msg) {
        // Update the timestamp of last cmd_vel reception
        last_cmd_vel_time_ = std::chrono::steady_clock::now();

        float vx = -msg->linear.y;
        float vy = msg->linear.x;
        float wz = msg->angular.z;

        // 假设线速度和角速度的映射比例不同，在此处统一定义
        const float LINEAR_SCALE = 500.0f;  // 线速度缩放系数
        const float ANGULAR_SCALE = 500.0f;   // 角速度缩放系数 (根据实际协议修改)
        const float SPIN_DODGE_ANGULAR_VELOCITY = 1.0f;  // Fixed angular velocity (rad/s)

        // Apply spin dodge if active
        // When spin dodge is active, ADD the fixed angular velocity to the cmd_vel's z
        if (is_spin_dodge_active()) {
            // ADD to the existing z velocity (not replace)
            wz += SPIN_DODGE_ANGULAR_VELOCITY;
            
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500,
                "SPIN DODGE + CMD_VEL: vx=%.2f, vy=%.2f, wz=%.1f (cmd_vel.z=%.1f + spin=%.1f)",
                vx, vy, wz, msg->angular.z, SPIN_DODGE_ANGULAR_VELOCITY);
        } else {
            RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                "Normal CMD_VEL: vx=%.2f, vy=%.2f, wz=%.1f", vx, vy, wz);
        }

        int16_t vx_int = static_cast<int16_t>(std::round(vx * LINEAR_SCALE));
        int16_t vy_int = static_cast<int16_t>(std::round(vy * LINEAR_SCALE));
        int16_t wz_int = static_cast<int16_t>(std::round(wz * ANGULAR_SCALE));
        int16_t power_int = 100; // 功率示例值

        vx_int = std::clamp(vx_int, static_cast<int16_t>(-LINEAR_SCALE), static_cast<int16_t>(LINEAR_SCALE));
        vy_int = std::clamp(vy_int, static_cast<int16_t>(-LINEAR_SCALE), static_cast<int16_t>(LINEAR_SCALE));
        wz_int = std::clamp(wz_int, static_cast<int16_t>(-ANGULAR_SCALE), static_cast<int16_t>(ANGULAR_SCALE));

        uint8_t send_data[8] = {0};
        send_data[1] = (vx_int >> 8) & 0xFF; 
        send_data[0] = vx_int & 0xFF;        
        send_data[3] = (vy_int >> 8) & 0xFF; 
        send_data[2] = vy_int & 0xFF;        
        send_data[5] = (wz_int >> 8) & 0xFF; 
        send_data[4] = wz_int & 0xFF;        
        send_data[7] = (power_int >> 8) & 0xFF; 
        send_data[6] = power_int & 0xFF;        
        usb_can_->transmit(0x120, send_data, 8);
        RCLCPP_DEBUG(this->get_logger(), "Sent 0x120: vx=%d, vy=%d, wz=%d", vx_int, vy_int, wz_int);
    }
    
    // Member variables
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr twist_sub_;
    rclcpp::Publisher<rm_interfaces::msg::GamePara>::SharedPtr game_para_pub_;
    rclcpp::Publisher<rm_interfaces::msg::OwnPara>::SharedPtr own_para_pub_;
    rclcpp::Publisher<rm_interfaces::msg::AllyPara>::SharedPtr ally_para_pub_;
    rclcpp::TimerBase::SharedPtr spin_dodge_timer_;  // Timer for independent spin dodge control
    usb_can_v2* usb_can_;
    
    // Game state from referee system
    GamePara game_para_;
    OwnPara own_para_;
    AllyPara ally_para_;
    
    // Spin dodge parameters
    static constexpr float HEALTH_THRESHOLD = 0.25f;  // 25% HP threshold
    bool spin_dodge_active_;
    std::chrono::steady_clock::time_point spin_dodge_start_time_;
    std::chrono::steady_clock::time_point last_cmd_vel_time_;  // Track last /cmd_vel reception time
    uint8_t last_hp_deduction_reason_;
    uint16_t last_hp_;  // Track last HP for change detection
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CanTwistNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}