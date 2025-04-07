#include <rclcpp/rclcpp.hpp>
#include <control_msgs/msg/dynamic_joint_state.hpp>
#include <gazebo_msgs/msg/contacts_state.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

/**
 * @brief ROS2 노드 클래스로 Gazebo에 스폰된 URDF 로봇을 제어합니다.
 * 
 * 이 클래스는 관절 상태와 접촉 정보를 구독하고 관절에 토크 명령을 발행합니다.
 */
class RobotController : public rclcpp::Node
{
public:
  /**
   * @brief 로봇 컨트롤러 노드 생성자
   */
  RobotController()
  : Node("robot_controller")
  {
    // 로그 출력
    RCLCPP_INFO(this->get_logger(), "Node start.");
    
    // 변수 초기화
    contact_detected_ = false;
    effort_commands_ = {0.0, 0.0, 0.0};  // 예시로 3개의 관절에 대한 명령으로 설정
    
    // QoS 설정
    auto qos = rclcpp::QoS(rclcpp::KeepLast(10))
      .reliable()
      .durability_volatile();
    
    // 토픽 구독자 및 발행자 정의 함수 호출
    define_publishers_and_subscribers(qos);
    
    // 타이머 설정 (10Hz로 제어 명령 발행)
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(100), 
      std::bind(&RobotController::control_loop, this)
    );
    
    // ros 파라미터 정의
    rcl_interfaces::msg::ParameterDescriptor param_desc;
    param_desc.read_only = false;
    this->declare_parameter("input", 0.0, param_desc);
  }

private:

  bool DOWN = false;

  /**
   * @brief 토픽 구독자와 발행자를 정의하는 함수
   * @param qos QoS 설정
   * 
   * 구독 토픽: /dynamic_joint_states, /shin_link/bumper_states
   * 발행 토픽: /effort_controller/commands
   */
  void define_publishers_and_subscribers(const rclcpp::QoS& qos)
  {
    // 구독자 정의
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
    
    // 발행자 정의
    effort_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
      "/effort_controller/commands",
      qos
    );
  }
  
  /**
   * @brief /dynamic_joint_states 토픽의 콜백 함수
   * @param msg 수신된 DynamicJointState 메시지
   * 
   * 관절 상태 정보를 처리합니다.
   */
  void joint_states_callback(const control_msgs::msg::DynamicJointState::SharedPtr msg)
  {
    // 메시지에서 관절 상태 정보 추출
    for (size_t i = 0; i < msg->joint_names.size(); ++i) {
      if (i < msg->interface_values.size()) {
        std::string joint_name = msg->joint_names[i];
        
        // 각 관절에 대한 정보 구조체 생성
        JointInfo joint_info;
        
        // 인터페이스 값을 찾아서 저장
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
        
        // 관절 정보 저장
        joint_states_[joint_name] = joint_info;
      }
    }
  }
  
  /**
   * @brief /shin_link/bumper_states 토픽의 콜백 함수
   * @param msg 수신된 ContactsState 메시지
   * 
   * 접촉 상태 정보를 처리해서 T/F로 return.
   */
  void contact_states_callback(const gazebo_msgs::msg::ContactsState::SharedPtr msg)
  {
    // 충돌 감지 여부 확인
    contact_detected_ = !msg->states.empty();
  }

  // slider joint의 상태값을 받아서 하상/상승 구분 
  void state_division()
  {
    if(0 <= joint_states_["slider_joint"].velocity){
      DOWN = true;
    }
    else{
      DOWN = false;
    }
  }

  // 하강, 상승에 맞춰 스프링 입력 계산
  double spring_input_calculation(double down_k, double down_b, double up_k, double up_b)
  { 
    double theta = -joint_states_["thigh_to_shin"].position;
    double d_theta = -joint_states_["thigh_to_shin"].velocity;

    if(DOWN){
      return down_k * theta + down_b * d_theta;
    }
    else{
      return up_k * theta + up_b * d_theta;
    }
  }

  double jumping(double bent_state, double jump_input)
  {
    if(DOWN && contact_detected_){
      return jump_input;
    }
    else{
      return 0;
    }
  }

  /**
   * @brief /effort_controller/commands 토픽에 메시지를 발행하는 함수
   * 
   * 로봇 관절에 적용할 토크 명령을 발행합니다.
   */
  void publish_effort_commands()
  {
    // Float64MultiArray 메시지 생성
    auto msg = std::make_unique<std_msgs::msg::Float64MultiArray>();
    msg->data = effort_commands_;
    
    // 메시지 발행
    effort_pub_->publish(std::move(msg));
  }
  
  /**
   * @brief 제어 로직을 수행하는 함수
   * 
   * 이 함수는 타이머에 의해 주기적으로 호출됩니다.
   */
  void control_loop()
  {
    state_division();

    effort_commands_ = {0, -(spring_input_calculation(0, 0, 0, 0) + jumping(2.5, get_parameter("input").as_double())), 0};  // 제어값 입력 예시
    
    publish_effort_commands();
  }

  // 관절 정보를 저장하기 위한 구조체
  struct JointInfo {
    double position = 0.0;
    double velocity = 0.0;
    double effort = 0.0;
  };

  // 클래스 멤버 변수
  rclcpp::Subscription<control_msgs::msg::DynamicJointState>::SharedPtr joint_states_sub_;
  rclcpp::Subscription<gazebo_msgs::msg::ContactsState>::SharedPtr contact_states_sub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr effort_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  
  std::unordered_map<std::string, JointInfo> joint_states_;
  bool contact_detected_;
  std::vector<double> effort_commands_;
};

/**
 * @brief 메인 함수
 * @param argc 명령행 인수 개수
 * @param argv 명령행 인수 배열
 * @return 종료 코드
 */
int main(int argc, char * argv[])
{
  // ROS2 초기화
  rclcpp::init(argc, argv);
  
  // 노드 생성 및 실행
  auto node = std::make_shared<RobotController>();
  
  try {
    // 노드 실행
    rclcpp::spin(node);
  } catch (const std::exception& e) {
    RCLCPP_ERROR(node->get_logger(), "fuck: %s", e.what());
  }
  
  // ROS2 종료
  rclcpp::shutdown();
  return 0;
}