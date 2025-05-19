import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from pan_tilt_lidar.utils import get_lidar_measurement, BaseController
import serial
import time

class PanTiltLidarNode(Node):
    def __init__(self):
        super().__init__('pan_tilt_lidar_node')
        self.gimbal = BaseController('/dev/ttyUSB1', 115200)
        self.lidar_ser = serial.Serial('/dev/ttyACM0', 230400, timeout=1)
        self.subscription = self.create_subscription(
            String,
            'keyboard_input',  # 修改为键盘输入主题
            self.start_scan_callback,
            10
        )
        self.scan_data = []

    def start_scan_callback(self, msg):
        if msg.data.strip().lower() == "start":
            self.get_logger().info('收到启动扫描信号: "%s"' % msg.data)
            self.perform_scan()
        else:
            self.get_logger().info(f'收到无关信号: "{msg.data}"，忽略。')

    def perform_scan(self):
        horiz_start = -30
        horiz_end = 30
        horiz_step = 2
        vert_start = -10
        vert_end = 40
        vert_step = 2

        total_steps = ((horiz_end - horiz_start) // horiz_step + 1) * ((vert_end - vert_start) // vert_step + 1)
        current_step = 1
        self.get_logger().info("开始3D扫描……")

        for vert_angle in range(vert_start, vert_end + 1, vert_step):
            self.gimbal.move_gimbal(angle_x=horiz_start, angle_y=vert_angle)
            time.sleep(5)
            for horiz_angle in range(horiz_start, horiz_end + 1, horiz_step):
                self.get_logger().info(f"Step {current_step}/{total_steps}，移动到 水平 {horiz_angle}°，垂直 {vert_angle}°")
                self.gimbal.move_gimbal(angle_x=horiz_angle, angle_y=vert_angle)
                time.sleep(1)
                distance = get_lidar_measurement(self.lidar_ser)
                self.get_logger().info(f"角度 (H={horiz_angle}°, V={vert_angle}°) 测得平均距离：{distance:.2f} mm")
                self.scan_data.append((horiz_angle, vert_angle, distance))
                current_step += 1
                time.sleep(0.5)

        self.gimbal.close()
        self.lidar_ser.close()
        self.get_logger().info("扫描完成，数据已保存。")

def main(args=None):
    rclpy.init(args=args)
    node = PanTiltLidarNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()