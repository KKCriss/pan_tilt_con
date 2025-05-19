def parse_lidar_data(data):
    # 解析激光测距数据
    if data[0] != 0xAA:
        print("Invalid data packet: Start byte not found.")
        return []

    measurements = []
    for i in range(10, 195, 15):
        if i + 15 < len(data):
            distance1 = data[i + 1]
            distance2 = data[i]
            distance = (distance2 << 8) | distance1  # 组合两个字节为一个距离值
            measurements.append(distance)
    return measurements

def control_gimbal(gimbal, angle_x, angle_y=0, speed=50, acc=10):
    # 控制云台运动
    gimbal.move_gimbal(angle_x, angle_y, speed, acc)