#include <memory>
#include <string>
#include <vector>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/vector3_stamped.hpp"
#include "dynamixel_sdk/dynamixel_sdk.h"

// =============================================================
// Control Table Address (XM430 Series)
// =============================================================
#define ADDR_OPERATING_MODE         11
#define ADDR_TORQUE_ENABLE          64
#define ADDR_GOAL_VELOCITY          104

// Data Byte Length
#define LEN_GOAL_VELOCITY           4

// Protocol & Settings
#define PROTOCOL_VERSION            2.0
#define BAUDRATE                    4000000
#define DEVICENAME                  "/dev/ttyUSB0" // 포트 확인 필요!

#define DXL_ID_ROLL                 1
#define DXL_ID_PITCH                2

class GimbalMotionNode : public rclcpp::Node
{
public:
  GimbalMotionNode()
  : Node("gimbal_motion_node")
  {
    // 1. 파라미터 선언
    this->declare_parameter("port_name", DEVICENAME);
    this->declare_parameter("baud_rate", BAUDRATE);
    
    // 모터 속도 스케일링 (입력값 -> DXL 값 변환 비율)
    // XM430 속도 단위: 0.229 RPM. 
    // PID 출력이 크면 이 값을 줄이고, 반응이 느리면 키우세요.
    this->declare_parameter("velocity_scale", 80.0); 

    std::string port_name = this->get_parameter("port_name").as_string();
    int baud_rate = this->get_parameter("baud_rate").as_int();
    velocity_scale_ = this->get_parameter("velocity_scale").as_double();

    // 2. 다이나믹셀 초기화
    if (!init_dynamixel(port_name, baud_rate)) {
      RCLCPP_ERROR(this->get_logger(), "Failed to initialize Dynamixel SDK");
      rclcpp::shutdown();
    }

    // 3. 토픽 구독
    subscription_ = this->create_subscription<geometry_msgs::msg::Vector3Stamped>(
      "/gimbal/motor_cmd", 10,
      std::bind(&GimbalMotionNode::topic_callback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "Gimbal Motion Node Started. Ready to move!");
  }

  virtual ~GimbalMotionNode()
  {
    // 종료 시 토크 해제
    stop_motors();
    portHandler_->closePort();
  }

private:
  void topic_callback(const geometry_msgs::msg::Vector3Stamped::SharedPtr msg)
  {
    // msg->vector.x : Roll Command (PID Output)
    // msg->vector.y : Pitch Command (PID Output)
    
    // [방향 매핑]
    // 사용자 설명: 
    // ID1(Roll): 시계방향(CW) -> 각도 증가
    // ID2(Pitch): 렌즈 위(Up) -> 각도 증가
    
    // 파이썬 노드에서 계산된 Cmd 값:
    // Cmd = (0 - 현재각도) * P - D
    // 예: 짐벌이 CW로 기울면(Roll > 0), Cmd는 음수(-)가 나옴.
    // -> 음수 속도 명령을 주면 각도가 감소하는 방향(CCW)으로 움직임.
    // -> 즉, 부호를 그대로 사용하면 됩니다.

    int32_t vel_roll = (int32_t)(msg->vector.x * velocity_scale_*-1.0);
    int32_t vel_pitch = (int32_t)(msg->vector.y * velocity_scale_*-1.0);

    // XM430 속도 제한 (안전장치)
    // 예를 들어 300 RPM 이상 돌지 않도록 막음
    vel_roll = std::clamp(vel_roll, -1023, 1023);
    vel_pitch = std::clamp(vel_pitch, -1023, 1023);

    // SyncWrite로 동시에 명령 전송
    groupSyncWrite_->clearParam();

    uint8_t param_roll[4];
    uint8_t param_pitch[4];

    param_roll[0] = DXL_LOBYTE(DXL_LOWORD(vel_roll));
    param_roll[1] = DXL_HIBYTE(DXL_LOWORD(vel_roll));
    param_roll[2] = DXL_LOBYTE(DXL_HIWORD(vel_roll));
    param_roll[3] = DXL_HIBYTE(DXL_HIWORD(vel_roll));

    param_pitch[0] = DXL_LOBYTE(DXL_LOWORD(vel_pitch));
    param_pitch[1] = DXL_HIBYTE(DXL_LOWORD(vel_pitch));
    param_pitch[2] = DXL_LOBYTE(DXL_HIWORD(vel_pitch));
    param_pitch[3] = DXL_HIBYTE(DXL_HIWORD(vel_pitch));

    groupSyncWrite_->addParam(DXL_ID_ROLL, param_roll);
    groupSyncWrite_->addParam(DXL_ID_PITCH, param_pitch);

    int dxl_comm_result = groupSyncWrite_->txPacket();
    if (dxl_comm_result != COMM_SUCCESS) {
      RCLCPP_WARN(this->get_logger(), "%s", packetHandler_->getTxRxResult(dxl_comm_result));
    }
  }

  bool init_dynamixel(const std::string & port_name, int baud_rate)
  {
    portHandler_ = dynamixel::PortHandler::getPortHandler(port_name.c_str());
    packetHandler_ = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);
    groupSyncWrite_ = new dynamixel::GroupSyncWrite(portHandler_, packetHandler_, ADDR_GOAL_VELOCITY, LEN_GOAL_VELOCITY);

    // 포트 열기
    if (!portHandler_->openPort()) {
      RCLCPP_ERROR(this->get_logger(), "Failed to open port!");
      return false;
    }
    // 통신 속도 설정
    if (!portHandler_->setBaudRate(baud_rate)) {
      RCLCPP_ERROR(this->get_logger(), "Failed to set baudrate!");
      return false;
    }

    // 모터 설정 (ID 1, 2)
    uint8_t dxl_ids[] = {DXL_ID_ROLL, DXL_ID_PITCH};
    for (uint8_t id : dxl_ids) {
      uint8_t dxl_error = 0;
      int dxl_comm_result = COMM_TX_FAIL;

      // 1. 토크 끄기 (설정 변경을 위해)
      packetHandler_->write1ByteTxRx(portHandler_, id, ADDR_TORQUE_ENABLE, 0, &dxl_error);

      // 2. 모드 설정 (1 = Velocity Control Mode)
      // 주의: 만약 Position Control을 원하면 3으로 설정해야 함. 짐벌은 속도제어가 유리.
      dxl_comm_result = packetHandler_->write1ByteTxRx(portHandler_, id, ADDR_OPERATING_MODE, 1, &dxl_error);
      
      if (dxl_comm_result != COMM_SUCCESS) {
        RCLCPP_ERROR(this->get_logger(), "ID %d: Failed to set Velocity Mode. Error: %s", id, packetHandler_->getTxRxResult(dxl_comm_result));
        return false;
      }

      // 3. 토크 켜기
      dxl_comm_result = packetHandler_->write1ByteTxRx(portHandler_, id, ADDR_TORQUE_ENABLE, 1, &dxl_error);
      if (dxl_comm_result != COMM_SUCCESS) {
        RCLCPP_ERROR(this->get_logger(), "ID %d: Failed to enable Torque.", id);
        return false;
      }
    }
    return true;
  }

  void stop_motors()
  {
    // 속도 0으로 정지
    packetHandler_->write4ByteTxRx(portHandler_, DXL_ID_ROLL, ADDR_GOAL_VELOCITY, 0);
    packetHandler_->write4ByteTxRx(portHandler_, DXL_ID_PITCH, ADDR_GOAL_VELOCITY, 0);
    // 토크 해제
    packetHandler_->write1ByteTxRx(portHandler_, DXL_ID_ROLL, ADDR_TORQUE_ENABLE, 0);
    packetHandler_->write1ByteTxRx(portHandler_, DXL_ID_PITCH, ADDR_TORQUE_ENABLE, 0);
  }

  dynamixel::PortHandler * portHandler_;
  dynamixel::PacketHandler * packetHandler_;
  dynamixel::GroupSyncWrite * groupSyncWrite_;
  rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr subscription_;
  
  double velocity_scale_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GimbalMotionNode>());
  rclcpp::shutdown();
  return 0;
}



//ros2 run gimbal_motion gimbal_motion_node