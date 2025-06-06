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
    
    // 타이머 설정 (100Hz로 제어 명령 발행)
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(10), 
      std::bind(&RobotController::control_loop, this)
    );
    
    // ros 파라미터 정의
    // ros2 run jumpin_robot jump_node --ros-args -p parm__1:=%f -p parm_2:=%f -p parm_3:=%f
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

  //hz에 필요한 타이머 정의
  rclcpp::Time last_trigger_time_{this->now()};  

  // pid 수식에 사용하기 미리 정의
  double error_integral_ = 0.0;
  double previous_error_ = 0.0;

  // msg 변수 정의 및 설정
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr k_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr b_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr m_pub_;
  std_msgs::msg::Float64 msg_k_;
  std_msgs::msg::Float64 msg_b_;
  std_msgs::msg::Float64 msg_m_;

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

  /**
 * @brief 특정 주기(Hz 단위)에 맞춰 bool 값을 반환하는 함수
 * @param hz 원하는 주파수 (Hz)
 * @return 지정된 주기마다 true, 그 외에는 false
 */
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
      
    // 적분 항 
    error_integral_ += error * dt;
      
    // 미분 항
    double error_derivative = (error - previous_error_) / dt;
      
    previous_error_ = error;
      
    return p_gain * error + i_gain * error_integral_ + d_gain * error_derivative;
  }

  // 하강, 상승에 맞춰 스프링 입력 계산
  double spring_input(double k, double b, double eq_point)
  { 
    // RCLCPP_INFO(this->get_logger(), "state: %f", joint_states_["thigh_to_shin"].position);
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
    k_pub_->publish(std::move(msg_k_));
    b_pub_->publish(std::move(msg_b_));
    m_pub_->publish(std::move(msg_m_));
  }
  
  /**
   * @brief 제어 로직을 수행하는 함수
   * 
   * 이 함수는 타이머에 의해 주기적으로 호출됩니다.
   */
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