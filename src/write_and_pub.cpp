#include "rclcpp/rclcpp.hpp"
#include <control_msgs/msg/dynamic_joint_state.hpp>
#include <gazebo_msgs/msg/contacts_state.hpp>
#include <fstream>
#include <unordered_map>
#include <iomanip>
#include <chrono>
#include <string>
#include <algorithm>  // std::min

class TopicLoggerNode : public rclcpp::Node
{
public:
  TopicLoggerNode() : Node("topic_logger_node")
  {
    // 출력 파일 파라미터 선언 및 가져오기
    this->declare_parameter<std::string>("output_file", "topic_data.txt");
    std::string output_file = this->get_parameter("output_file").as_string();

    // 출력 파일을 append 모드로 열고, 파일이 비어있으면 헤더 작성
    output_file_ = std::make_shared<std::ofstream>(output_file, std::ios::app);
    std::ifstream check_file(output_file);
    if (check_file.peek() == std::ifstream::traits_type::eof()) {
      *output_file_ << "nee_angle value1\tjump_high value2" << std::endl;
    }
    check_file.close();

    // QoS 설정
    auto qos = rclcpp::QoS(rclcpp::KeepLast(10))
                 .reliable()
                 .durability_volatile();

    // 토픽 구독자 초기화 (람다 사용)
    define_topic_subscribers(qos);

    // 10ms 주기로 log_data() 호출하는 타이머 생성
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(1),
      [this]() { this->log_data(); }
    );
  }

  ~TopicLoggerNode() override
  {
    if (output_file_ && output_file_->is_open()) {
      output_file_->close();
      RCLCPP_INFO(this->get_logger(), "Log file closed.");
    }
  }

private:
  bool contact_detected_{false};
  bool topic1_received_{false};
  bool topic2_received_{false};

  // 각 관절의 정보를 저장하기 위한 구조체
  struct JointInfo {
    double position{0.0};
    double velocity{0.0};
    double effort{0.0};
  };

  std::unordered_map<std::string, JointInfo> joint_states_;

  rclcpp::Subscription<control_msgs::msg::DynamicJointState>::SharedPtr joint_states_sub_;
  rclcpp::Subscription<gazebo_msgs::msg::ContactsState>::SharedPtr contact_states_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::shared_ptr<std::ofstream> output_file_;

  // 구독자들을 초기화
  void define_topic_subscribers(const rclcpp::QoS & qos)
  {
    joint_states_sub_ = this->create_subscription<control_msgs::msg::DynamicJointState>(
      "/dynamic_joint_states",
      qos,
      [this](const control_msgs::msg::DynamicJointState::SharedPtr msg) {
        this->joint_states_callback(msg);
      }
    );

    contact_states_sub_ = this->create_subscription<gazebo_msgs::msg::ContactsState>(
      "/shin_link/bumper_states",
      qos,
      [this](const gazebo_msgs::msg::ContactsState::SharedPtr msg) {
        this->contact_states_callback(msg);
      }
    );
  }

  // 관절 상태 메시지 콜백 (안전하게 for문 범위 제한)
  void joint_states_callback(const control_msgs::msg::DynamicJointState::SharedPtr msg)
  {
    // 메시지에서 관절 이름과 인터페이스 값을 안전하게 추출
    size_t joint_count = std::min(msg->joint_names.size(), msg->interface_values.size());
    for (size_t i = 0; i < joint_count; ++i) {
      std::string joint_name = msg->joint_names[i];
      JointInfo joint_info;
      size_t interface_count = std::min(msg->interface_values[i].interface_names.size(),
                                        msg->interface_values[i].values.size());
      for (size_t j = 0; j < interface_count; ++j) {
        const std::string & interface_name = msg->interface_values[i].interface_names[j];
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
    topic1_received_ = true;
  }

  // 접촉 상태 메시지 콜백
  void contact_states_callback(const gazebo_msgs::msg::ContactsState::SharedPtr msg)
  {
    contact_detected_ = !msg->states.empty();
    topic2_received_ = true;
  }

  // 조건에 맞으면 joint_states_ 정보 로깅
  void log_data()
  {
    if (topic1_received_ && topic2_received_) {
      if (!contact_detected_) {
        // 존재하는 관절 키가 아닐 경우 0.0 값을 사용하여 안전하게 기록
        double nee_angle = joint_states_.count("thigh_to_shin") ? joint_states_["thigh_to_shin"].position : 0.0;
        double jump_high = joint_states_.count("slider_joint") ? joint_states_["slider_joint"].position : 0.0;
        *output_file_ << "nee_angle : " << std::fixed << std::setprecision(3) << nee_angle << "\t"
                      << "jump_high : " << std::fixed << std::setprecision(3) << jump_high << std::endl;
      }
      // 두 토픽의 수신 플래그 초기화
      topic1_received_ = false;
      topic2_received_ = false;
    }
  }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TopicLoggerNode>());
  rclcpp::shutdown();
  return 0;
}