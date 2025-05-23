import serial
import json
import time
import math
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D # 3D 绘图

# ---------------------- 云台控制部分 -------------
class BaseController:
    def __init__(self, port='/dev/ttyUSB1', baudrate=115200):
        self.ser = serial.Serial(port, baudrate, timeout=1)
        print(f"[Gimbal] 已连接到 {port}")
    def send_command(self, command):
        """发送 JSON 指令到设备"""
        command_str = json.dumps(command) + "\n"
        self.ser.write(command_str.encode('utf-8'))
        print("[Gimbal] 已发送指令：", command)
    def close(self):
        self.ser.close()
    def move_gimbal(self, angle_x, angle_y=0, speed=50,acc=10):
        """
        控制云台运动
        angle_x：水平方向角度（单位：度）
        angle_y：垂直方向角度（单位：度）
        """
        cmd = {"T": 133, "X": angle_x, "Y": angle_y, "SPD":
        speed, "ACC": acc}
        self.send_command(cmd)

# ---------------------- 激光测距部分 ----------------------
LIDAR_PACKET_SIZE = 195  # 数据包长度
LIDAR_POINT_SIZE = 15    # 每个测距点长度
LIDAR_HEADER = b'\xAA'   # 包头
LIDAR_DATA_OFFSET = 10   # 数据起始偏移
LIDAR_MAX_ATTEMPTS = 100 # 最大尝试次数

def parse_data(data):
    # 检查起始符
    if data[0] != 0xAA:
        print("Invalid data packet: Start byte not found.")
        return []

    measurements = []
    # 从第11个字节开始解析每个测量数据点
    for i in range(10, 195, 15):
        if i + 15 < len(data):  # 确保不会越界
            # 距离数据distance
            distance1 = data[i + 1]
            distance1 = hex(distance1)
            interval1 = distance1[2:].upper()
            str1 = str(interval1)
            distance2 = data[i]
            distance2 = hex(distance2)
            interval2 = distance2[2:].upper()
            str2 = str(interval2)
            string1 = str1 + str2
            distance = int(string1, 16)
            measurements.append(distance)
    return measurements

def get_lidar_measurement(ser):
    raw_data = b""
    attempts = 0
    while attempts < LIDAR_MAX_ATTEMPTS:
        if ser.in_waiting > 0:
            raw_data += ser.read(ser.in_waiting)
        start_index = raw_data.find(LIDAR_HEADER)
        if start_index != -1 and len(raw_data) - start_index >= LIDAR_PACKET_SIZE:
            packet = raw_data[start_index:start_index+LIDAR_PACKET_SIZE]
            raw_data = raw_data[start_index+LIDAR_PACKET_SIZE:]
            data_list = list(bytearray(packet))
            measurements = parse_data(data_list)
            if measurements:
                return sum(measurements) / float(len(measurements))
        attempts += 1
        time.sleep(0.05)
    raise TimeoutError("激光雷达数据读取超时")
# ---------------------- 主流程 ----------------------
if __name__ == '__main__':
    # 初始化云台控制（根据实际情况修改端口号）
    gimbal = BaseController('/dev/ttyUSB1', 115200)
    # 初始化激光雷达串口（根据实际情况修改端口号和波特率）
    try:
        lidar_ser = serial.Serial('/dev/ttyACM0', 230400, timeout=1)
        print("[Lidar] 已连接到 /dev/ttyACM0")
    except Exception as e:
        print("[Lidar] 连接失败:", e)
        exit(1)
    # 扫描参数设置（单位：度）
    horiz_start = -30# 水平起始角
    horiz_end = 30# 水平结束角
    horiz_step = 2# 水平步进
    vert_start = -10# 垂直起始角
    vert_end = 40# 垂直结束角
    vert_step = 2# 垂直步进
    wait_time = 1.5 # 每次云台运动后等待时间（秒）
    # 存储扫描数据，数据格式：(horiz_angle, vert_angle,distance)
    scan_data = []
    total_steps = ((horiz_end - horiz_start) // horiz_step + 1) * ((vert_end - vert_start) // vert_step + 1)
    current_step = 1
    print("开始3D扫描……")
    for vert_angle in range(vert_start, vert_end + 1, vert_step):
        gimbal.move_gimbal(angle_x=horiz_start, angle_y=vert_angle)
        time.sleep(5)
        for horiz_angle in range(horiz_start, horiz_end + 1, horiz_step):
            print(f"\n[Scan] Step {current_step}/{total_steps}，移动到 水平 {horiz_angle}°，垂直 {vert_angle}°")
            gimbal.move_gimbal(angle_x=horiz_angle, angle_y=vert_angle)
            time.sleep(1) # 等待云台到位并稳定
            distance = get_lidar_measurement(lidar_ser)
            print(f"[Scan] 角度 (H={horiz_angle}°, V={vert_angle}°) 测得平均距离：{distance:.2f} mm")
            scan_data.append((horiz_angle, vert_angle, distance))
            current_step += 1
            time.sleep(0.5)
    # 关闭串口
    gimbal.close()
    lidar_ser.close()
    # ---------------------- 数据处理与可视化 --------------------
    if scan_data:
        xs, ys, zs = [], [], []
        for h_angle, v_angle, d in scan_data:
            theta = math.radians(h_angle) # 水平角（方位角）
            phi = math.radians(v_angle)# 垂直角（仰角）
            # 球坐标转换为直角坐标
            x = d * math.cos(phi) * math.cos(theta)
            y = d * math.cos(phi) * math.sin(theta)
            z = d * math.sin(phi)
            xs.append(x)
            ys.append(y)
            zs.append(z)
        # 绘制3D点云
        fig = plt.figure(figsize=(10, 8))
        ax = fig.add_subplot(111, projection='3d')
        ax.scatter(xs, ys, zs, c='blue', marker='o', label='扫描点')
        # 绘制传感器自身位置
        ax.scatter(0, 0, 0, c='red', marker='^', s=100, label='传感器位置')
        ax.set_xlabel('X 轴 (mm)')  
        ax.set_ylabel('Y 轴 (mm)')
        ax.set_zlabel('Z 轴 (mm)')
        ax.set_title("3D 激光雷达点云")
        ax.legend()
        plt.show()
        # 保存数据到文件
        with open('scan_data.json', 'w') as f:
            json.dump(scan_data, f, indent=4)
        print("扫描数据已保存到 scan_data.json")
    else:
        print("未采集到任何数据点")