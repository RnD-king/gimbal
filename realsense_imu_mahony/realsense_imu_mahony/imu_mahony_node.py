#!/usr/bin/env python3
import math
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3Stamped

def normalize(v: np.ndarray) -> np.ndarray:
    n = np.linalg.norm(v)
    if n < 1e-9:
        return v
    return v / n

class GimbalImuNode(Node):
    def __init__(self):
        super().__init__('gimbal_imu_controller')

        # =========================
        # 1. 파라미터 설정 (튜닝 포인트)
        # =========================
        self.declare_parameter('gyro_topic', '/camera/camera/gyro/sample')
        self.declare_parameter('accel_topic', '/camera/camera/accel/sample')

        # Mahony 필터 게인 (센서 융합 속도)
        self.declare_parameter('Kp', 4.0)
        self.declare_parameter('Ki', 0.0)

        # [PD 제어 게인]
        # motor_kp: P게인 (클수록 복원력이 강해짐 / 너무 크면 진동)
        # motor_kd: D게인 (클수록 브레이크가 강해짐 / 너무 크면 고주파 소음)
        self.declare_parameter('motor_kp', 4.0)
        self.declare_parameter('motor_kd', 0.1)

        #  파라미터 선언 (기본값 60.0도)
        self.declare_parameter('safety_angle', 60.0)
        self.safety_angle = float(self.get_parameter('safety_angle').value)

        self.gyro_topic = self.get_parameter('gyro_topic').get_parameter_value().string_value
        self.accel_topic = self.get_parameter('accel_topic').get_parameter_value().string_value

        self.Kp = float(self.get_parameter('Kp').value)
        self.Ki = float(self.get_parameter('Ki').value)
        self.motor_kp = float(self.get_parameter('motor_kp').value)
        self.motor_kd = float(self.get_parameter('motor_kd').value)

        # =========================
        # 2. 상태 변수 초기화
        # =========================
        # 기준 자세 (Tare)
        self.ref_set = False
        self.roll_ref = 0.0
        self.pitch_ref = 0.0
        self.yaw_ref = 0.0

        # 데이터 버퍼
        self.last_gyro: Imu | None = None
        self.last_accel: Imu | None = None
        self.last_time = None

        # Quaternion [w, x, y, z]
        self.q = np.array([1.0, 0.0, 0.0, 0.0], dtype=float)
        
        # Bias 보정용 변수
        self.gyro_bias = np.zeros(3, dtype=float)
        self.integral_error = np.zeros(3, dtype=float)

        # 초기 캘리브레이션 (3초)
        self.calibrating = True
        self.calib_start_time = None
        self.gyro_bias_sum = np.zeros(3, dtype=float)
        self.bias_samples = 0
        self.calib_duration = 3.0

        # =========================
        # 3. ROS 통신 설정
        # =========================
        self.sub_gyro = self.create_subscription(
            Imu, self.gyro_topic, self.gyro_callback, qos_profile_sensor_data)

        self.sub_accel = self.create_subscription(
            Imu, self.accel_topic, self.accel_callback, qos_profile_sensor_data)

        # [출력 1] 현재 각도 (모니터링용, Degree)
        self.pub_angle = self.create_publisher(Vector3Stamped, '/gimbal/current_angle', 10)
        
        # [출력 2] 모터 제어 명령 (PD 제어 결과값)
        self.pub_cmd = self.create_publisher(Vector3Stamped, '/gimbal/motor_cmd', 10)

        # 200Hz 루프 실행
        self.timer = self.create_timer(0.005, self.update_filter)

        self.get_logger().info('D435i PD Gimbal Controller Started. Keeping stable for 3 seconds...')

    # =========================
    # 헬퍼 함수
    # =========================
    def wrap_angle(self, angle):
        """ -PI ~ +PI 범위로 각도 정규화 (튀는 현상 방지) """
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

    # =========================
    # 콜백 함수
    # =========================
    def gyro_callback(self, msg: Imu):
        self.last_gyro = msg
        if self.calibrating and self.calib_start_time is None:
            self.calib_start_time = self.get_clock().now()

    def accel_callback(self, msg: Imu):
        self.last_accel = msg

    # =========================
    # 메인 루프 (핵심 로직)
    # =========================
    def update_filter(self):
        if self.last_gyro is None or self.last_accel is None:
            return

        now = self.get_clock().now()
        
        # dt 계산
        if self.last_time is None:
            self.last_time = now
            return
        dt = (now - self.last_time).nanoseconds * 1e-9
        self.last_time = now

        if dt <= 0.0 or dt > 0.1: return

        # ---------------------------------------------------------
        # [1] 좌표계 변환 (D435i -> NED Standard)
        # ---------------------------------------------------------
        # D435i Raw: X(Right), Y(Down), Z(Forward)
        # Algorithm Expects: X(Forward), Y(Right), Z(Down)
        
        gx_raw = self.last_gyro.angular_velocity.x
        gy_raw = self.last_gyro.angular_velocity.y
        gz_raw = self.last_gyro.angular_velocity.z

        ax_raw = self.last_accel.linear_acceleration.x
        ay_raw = self.last_accel.linear_acceleration.y
        az_raw = self.last_accel.linear_acceleration.z

        # Remapping
        # Algo X = Cam Z (전방)
        # Algo Y = Cam X (우측)
        # Algo Z = Cam Y (하방)
        gx, gy, gz = gz_raw, gx_raw, gy_raw
        ax, ay, az = az_raw, ax_raw, ay_raw

        gyro = np.array([gx, gy, gz], dtype=float)
        accel = np.array([ax, ay, az], dtype=float)

        # ---------------------------------------------------------
        # [2] 초기 캘리브레이션
        # ---------------------------------------------------------
        if self.calibrating:
            elapsed = (now - self.calib_start_time).nanoseconds * 1e-9
            if elapsed < self.calib_duration:
                self.gyro_bias_sum += gyro
                self.bias_samples += 1
                return
            else:
                self.gyro_bias = self.gyro_bias_sum / max(1, self.bias_samples)
                self.calibrating = False
                self.get_logger().info(f'Calibration Done! Bias: {self.gyro_bias}')
                
                # 초기 자세 설정
                self.roll_ref = math.atan2(ay, az)
                self.pitch_ref = math.atan2(-ax, math.sqrt(ay*ay + az*az))
                self.yaw_ref = 0.0
                self.ref_set = True
                return

        # ---------------------------------------------------------
        # [3] Mahony Filter Update
        # ---------------------------------------------------------
        gyro_unbiased = gyro - self.gyro_bias
        accel_norm = np.linalg.norm(accel)
        
        # Accel을 이용한 오차 보정
        if accel_norm > 0:
            accel_unit = accel / accel_norm
            
            q0, q1, q2, q3 = self.q
            vx = 2.0 * (q1 * q3 - q0 * q2)
            vy = 2.0 * (q0 * q1 + q2 * q3)
            vz = (q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3)

            ex = (accel_unit[1] * vz - accel_unit[2] * vy)
            ey = (accel_unit[2] * vx - accel_unit[0] * vz)
            ez = (accel_unit[0] * vy - accel_unit[1] * vx)
            
            error = np.array([ex, ey, ez])
            
            if self.Ki > 0:
                self.integral_error += error * dt
                gyro_corr = gyro_unbiased + self.Kp * error + self.Ki * self.integral_error
            else:
                gyro_corr = gyro_unbiased + self.Kp * error
        else:
            gyro_corr = gyro_unbiased

        # Quaternion 적분
        gx_c, gy_c, gz_c = gyro_corr
        q0, q1, q2, q3 = self.q
        
        q_dot0 = 0.5 * (-q1 * gx_c - q2 * gy_c - q3 * gz_c)
        q_dot1 = 0.5 * ( q0 * gx_c + q2 * gz_c - q3 * gy_c)
        q_dot2 = 0.5 * ( q0 * gy_c - q1 * gz_c + q3 * gx_c)
        q_dot3 = 0.5 * ( q0 * gz_c + q1 * gy_c - q2 * gx_c)

        self.q = normalize(self.q + np.array([q_dot0, q_dot1, q_dot2, q_dot3]) * dt)

        # ---------------------------------------------------------
        # [4] 각도 계산 및 Wrapping
        # ---------------------------------------------------------
        q0, q1, q2, q3 = self.q
        
        sinr = 2.0 * (q0 * q1 + q2 * q3)
        cosr = 1.0 - 2.0 * (q1 * q1 + q2 * q2)
        roll = math.atan2(sinr, cosr)

        sinp = 2.0 * (q0 * q2 - q3 * q1)
        if abs(sinp) >= 1: pitch = math.copysign(math.pi / 2, sinp)
        else: pitch = math.asin(sinp)

        siny = 2.0 * (q0 * q3 + q1 * q2)
        cosy = 1.0 - 2.0 * (q2 * q2 + q3 * q3)
        yaw = math.atan2(siny, cosy)

        # Ref 차감 및 Wrapping (-PI ~ PI)
        current_roll  = self.wrap_angle(roll - self.roll_ref)
        current_pitch = self.wrap_angle(pitch - self.pitch_ref)
        current_yaw   = self.wrap_angle(yaw - self.yaw_ref)

        # ---------------------------------------------------------
        # [5] PD 제어 명령 생성 (Target = 0)
        # ---------------------------------------------------------
        # P-Term: 각도 오차 (0 - current)
        error_roll  = 0.0 - current_roll
        error_pitch = 0.0 - current_pitch
        
        # D-Term: 각속도 감쇠 (gx, gy는 remapping된 Roll/Pitch 속도임)
        # 이 속도를 0으로 만들기 위해 반대 방향 힘을 생성
        rate_roll  = gx
        rate_pitch = gy
        
        # PD Command = (Kp * Error) - (Kd * Rate)
        cmd_roll  = (self.motor_kp * error_roll)  - (self.motor_kd * rate_roll)
        cmd_pitch = (self.motor_kp * error_pitch) - (self.motor_kd * rate_pitch)

        # 2. 가상 벽 로직 (Virtual Wall Logic)
        limit_rad = math.radians(self.safety_angle)

        # --- Roll 축 ---
        if current_roll > limit_rad:
            # 양의 한계를 넘음: 더 밀어내는(+) 힘은 차단, 돌아오는(-) 힘은 허용
            if cmd_roll > 0: cmd_roll = 0.0
        elif current_roll < -limit_rad:
            # 음의 한계를 넘음: 더 밀어내는(-) 힘은 차단, 돌아오는(+) 힘은 허용
            if cmd_roll < 0: cmd_roll = 0.0

        # --- Pitch 축 ---
        if current_pitch > limit_rad:
            if cmd_pitch > 0: cmd_pitch = 0.0
        elif current_pitch < -limit_rad:
            if cmd_pitch < 0: cmd_pitch = 0.0
        
        # ---------------------------------------------------------
        # [6] Publish
        # ---------------------------------------------------------
        # 1. 현재 각도 (Degree)
        msg_ang = Vector3Stamped()
        msg_ang.header.stamp = now.to_msg()
        msg_ang.header.frame_id = "gimbal_link"
        msg_ang.vector.x = math.degrees(current_roll)
        msg_ang.vector.y = math.degrees(current_pitch)
        msg_ang.vector.z = math.degrees(current_yaw)
        self.pub_angle.publish(msg_ang)

        # 2. 모터 명령 (Velocity or Torque Command)
        msg_cmd = Vector3Stamped()
        msg_cmd.header.stamp = now.to_msg()
        msg_cmd.vector.x = cmd_roll
        msg_cmd.vector.y = cmd_pitch
        msg_cmd.vector.z = 0.0 # Yaw 제어 필요 시 추가
        self.pub_cmd.publish(msg_cmd)

def main():
    rclpy.init()
    node = GimbalImuNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()






#전체 실행 (카메라 + 필터)
#ros2 launch realsense_imu_mahony bringup_launch.py


# 계산된 Roll, Pitch, Yaw 확인
#ros2 topic echo /gimbal/current_angle

#앞으로 기울면 y증가
#렌즈쪽에서 바라볼때 시계방향 x 감소













