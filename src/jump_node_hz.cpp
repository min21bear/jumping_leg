#include <rclcpp/rclcpp.hpp>
#include <control_msgs/msg/dynamic_joint_state.hpp>
#include <gazebo_msgs/msg/contacts_state.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/float64.hpp>

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>
#include <cmath>
#include <math.h>


class RobotController : public rclcpp::Node
{
public:

  RobotController()
  : Node("robot_controller")
  {
    RCLCPP_INFO(this->get_logger(), "Node start.");
    
    contact_detected_ = false;
    effort_commands_ = {0.0, 0.0, 0.0};
    
    auto qos = rclcpp::QoS(rclcpp::KeepLast(10))
      .reliable()
      .durability_volatile();
    
    define_publishers_and_subscribers(qos);
    
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(10), 
      std::bind(&RobotController::control_loop, this)
    );
    
    rcl_interfaces::msg::ParameterDescriptor p_desc;
    p_desc.read_only = false;
    this->declare_parameter("p", 0.0, p_desc);

    rcl_interfaces::msg::ParameterDescriptor i_desc;
    i_desc.read_only = false;
    this->declare_parameter("i", 0.0, i_desc);

    rcl_interfaces::msg::ParameterDescriptor d_desc;
    d_desc.read_only = false;
    this->declare_parameter("d", 0.0, d_desc);

    rcl_interfaces::msg::ParameterDescriptor up_k_desc;
    up_k_desc.read_only = false;
    this->declare_parameter("up_k", 0.0, up_k_desc);

    rcl_interfaces::msg::ParameterDescriptor up_b_desc;
    up_b_desc.read_only = false;
    this->declare_parameter("up_b", 0.0, up_b_desc);

    rcl_interfaces::msg::ParameterDescriptor base_k_desc;
    base_k_desc.read_only = false;
    this->declare_parameter("base_k", 0.0, base_k_desc);

    rcl_interfaces::msg::ParameterDescriptor base_b_desc;
    base_b_desc.read_only = false;
    this->declare_parameter("base_b", 0.0, base_b_desc);

    rcl_interfaces::msg::ParameterDescriptor input_desc;
    input_desc.read_only = false;
    this->declare_parameter("input", 0.0, input_desc);

    rcl_interfaces::msg::ParameterDescriptor up_eq_point_desc;
    up_eq_point_desc.read_only = false;
    this->declare_parameter("up_eq_point", 0.0, up_eq_point_desc);

    rcl_interfaces::msg::ParameterDescriptor down_eq_point_desc;
    down_eq_point_desc.read_only = false;
    this->declare_parameter("down_eq_point", 0.0, down_eq_point_desc);

    rcl_interfaces::msg::ParameterDescriptor jump_point_desc;
    jump_point_desc.read_only = false;
    this->declare_parameter("jump_point", 0.0, jump_point_desc);

    rcl_interfaces::msg::ParameterDescriptor hz_desc;
    hz_desc.read_only = false;
    this->declare_parameter("hz", 0.0, hz_desc);
  }

private:

  rclcpp::Time last_trigger_time_{this->now()};  

  double error_integral_ = 0.0;
  double previous_error_ = 0.0;

  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr k_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr b_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr m_pub_;
  std_msgs::msg::Float64 msg_k_;
  std_msgs::msg::Float64 msg_b_;
  std_msgs::msg::Float64 msg_m_;

  void define_publishers_and_subscribers(const rclcpp::QoS& qos)
  {
    joint_states_sub_ = this->create_subscription<control_msgs::msg::DynamicJointState>(
      "/dynamic_joint_states",
      qos,
      std::bind(&RobotController::joint_states_callback, this, std::placeholders::_1)
    );
    
    contact_states_sub_ = this->create_subscription<gazebo_msgs::msg::ContactsState>(
      "/shin_link/bumper_states",
      qos,
      std::bind(&RobotController::contact_states_callback, this, std::placeholders::_1)
    );

    effort_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
      "/effort_controller/commands",
      qos
    );

    k_pub_ = this->create_publisher<std_msgs::msg::Float64>(
      "/k_value", qos
    );
    
    b_pub_ = this->create_publisher<std_msgs::msg::Float64>(
      "/b_value", qos
    );
    
    m_pub_ = this->create_publisher<std_msgs::msg::Float64>(
      "/m_value", qos
    );
  }
  
  void joint_states_callback(const control_msgs::msg::DynamicJointState::SharedPtr msg)
  {
    for (size_t i = 0; i < msg->joint_names.size(); ++i) {
      if (i < msg->interface_values.size()) {
        std::string joint_name = msg->joint_names[i];
        
        JointInfo joint_info;
        
        for (size_t j = 0; j < msg->interface_values[i].interface_names.size(); ++j) {
          const std::string& interface_name = msg->interface_values[i].interface_names[j];
          double value = msg->interface_values[i].values[j];
          
          if (interface_name == "position") {
            joint_info.position = value;
          } else if (interface_name == "velocity") {
            joint_info.velocity = value;
          } else if (interface_name == "effort") {
            joint_info.effort = value;
          }
        }
        
        joint_states_[joint_name] = joint_info;
      }
    }
  }
  
  void contact_states_callback(const gazebo_msgs::msg::ContactsState::SharedPtr msg)
  {
    contact_detected_ = !msg->states.empty();
  }

  bool jump_hz_trigger(double hz)
  {
      rclcpp::Time now = this->now();
      
      rclcpp::Duration period = rclcpp::Duration::from_seconds(1.0 / hz);
        
      if ((now - last_trigger_time_) >= period) {
          last_trigger_time_ = now;
          return true;
        }
      return false;
  }

  double hip_input(double p_gain, double i_gain, double d_gain)
  {

    double dt = 0.01;  

    double target_angle = -joint_states_["thigh_to_shin"].position / 2.50048;
    double error = target_angle - joint_states_["hip_to_thigh"].position;
      
    error_integral_ += error * dt;
      
    double error_derivative = (error - previous_error_) / dt;
      
    previous_error_ = error;
      
    return p_gain * error + i_gain * error_integral_ + d_gain * error_derivative;
  }

  double spring_input(double k, double b, double eq_point)
  { 
    double theta = joint_states_["thigh_to_shin"].position - eq_point;
    double d_theta = joint_states_["thigh_to_shin"].velocity;
    
    msg_k_.data = -k*theta;
    msg_b_.data = -b*d_theta;

    return -(k*theta +b*d_theta);
  }

  double jumping(double base_k, double base_b, double jump_k, double jump_b, double input, double up_eq_point, double down_eq_point, double hz)
  { 
    if(jump_hz_trigger(hz)){
      RCLCPP_INFO(this->get_logger(), "timing is NOW");
      msg_m_.data = -input;
      return spring_input(jump_k, jump_b, up_eq_point) - input;
    }
    else{
      return spring_input(base_k, base_b, down_eq_point);
    }
  }

  void publish_effort_commands()
  {
    auto msg = std::make_unique<std_msgs::msg::Float64MultiArray>();
    msg->data = effort_commands_;
    
    effort_pub_->publish(std::move(msg));
    k_pub_->publish(std::move(msg_k_));
    b_pub_->publish(std::move(msg_b_));
    m_pub_->publish(std::move(msg_m_));
  }
  
  void control_loop()
  {
    double p = get_parameter("p").as_double();
    double i = get_parameter("i").as_double();
    double d = get_parameter("d").as_double();

    double base_k = get_parameter("base_k").as_double();
    double base_b = get_parameter("base_b").as_double();
    double up_k = get_parameter("up_k").as_double();
    double up_b = get_parameter("up_b").as_double();
    
    double input = get_parameter("input").as_double();
    double up_eq_point = get_parameter("up_eq_point").as_double();
    double down_eq_point = get_parameter("down_eq_point").as_double();
    double hz = get_parameter("hz").as_double();

    effort_commands_ = {hip_input(p, i, d), jumping(base_k, base_b, up_k, up_b, input, up_eq_point, down_eq_point, hz), 0.0};  // 제어값 입력 예시
    
    publish_effort_commands();
  }

  struct JointInfo {
    double position = 0.0;
    double velocity = 0.0;
    double effort = 0.0;
  };

  rclcpp::Subscription<control_msgs::msg::DynamicJointState>::SharedPtr joint_states_sub_;
  rclcpp::Subscription<gazebo_msgs::msg::ContactsState>::SharedPtr contact_states_sub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr effort_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  
  std::unordered_map<std::string, JointInfo> joint_states_;
  bool contact_detected_;
  std::vector<double> effort_commands_;
};

int main(int argc, char * argv[])
{

  rclcpp::init(argc, argv);
  

  auto node = std::make_shared<RobotController>();
  
  try {
    rclcpp::spin(node);
  } catch (const std::exception& e) {
    RCLCPP_ERROR(node->get_logger(), "fuck: %s", e.what());
  }
  
  rclcpp::shutdown();
  return 0;
}