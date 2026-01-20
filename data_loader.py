import numpy as np
import serial
import serial.tools.list_ports
import threading
import time
import re
import os
import sys


class DataLoader:
    """数据加载器，支持动态维度设置"""

    def __init__(self, file_path, num_dimensions):
        """
        初始化数据加载器
        :param file_path: .txt文件路径
        :param num_dimensions: 数据维度（列数）
        """
        self.file_path = file_path
        self.num_dimensions = num_dimensions

    def load_data(self):
        """加载数据并返回二维数组 (num_points × num_dimensions)"""
        data = []
        with open(self.file_path, 'r') as f:
            for line in f:
                values = list(map(float, line.split()))
                if len(values) >= self.num_dimensions:
                    data.append(values[:self.num_dimensions])
                elif len(values) > 0:  # 如果行中有数据但不够维度数
                    # 用最后一个值填充缺失的维度
                    extended_values = values[:]
                    while len(extended_values) < self.num_dimensions:
                        extended_values.append(values[-1] if values else 0.0)
                    data.append(extended_values[:self.num_dimensions])
        if not data:
            # 如果没有有效数据，创建一些示例数据
            print(f"警告: 文件 {self.file_path} 中没有足够的数据，创建示例数据")
            data = [[0.0] * self.num_dimensions for _ in range(100)]
        return np.array(data)


class DataSimulator:
    """数据模拟器，生成实时数据"""

    def __init__(self, num_dimensions=3):
        self.num_dimensions = num_dimensions
        self.time_counter = 0
        self.frequency = 2  # 模拟信号频率

    def get_next_data_point(self):
        """获取下一个数据点"""
        # 生成带有正弦波特征的模拟数据
        t = self.time_counter * 0.01  # 时间步长
        data_point = []

        for i in range(self.num_dimensions):
            # 每个维度有不同的频率和相位
            freq_offset = 0.5 * i
            phase_offset = i * np.pi / 3
            value = np.sin(2 * np.pi * (self.frequency + freq_offset) * t + phase_offset)
            data_point.append(value)

        self.time_counter += 1
        return np.array(data_point)

    def reset(self):
        """重置模拟器计数器"""
        self.time_counter = 0


class SerialDataReceiver:
    """串口数据接收器，自动检测并连接串口，接收下位机数据"""

    def __init__(self, num_dimensions=3, baudrate=115200, timeout=1):
        """
        初始化串口数据接收器
        :param num_dimensions: 数据维度（列数）
        :param baudrate: 波特率
        :param timeout: 超时时间
        """
        self.num_dimensions = num_dimensions
        self.baudrate = baudrate
        self.timeout = timeout
        self.serial_port = None
        self.is_connected = False
        self.data_queue = []  # 存储接收到的数据
        self.receive_thread = None
        self.stop_flag = False
        self.lock = threading.Lock()

        # 状态回调函数
        self.status_callback = None

        # 握手协议相关
        self.handshake_commands = ['hello', 'ping', 'test']
        self.handshake_responses = ['ack', 'pong', 'ok']

    def set_status_callback(self, callback):
        """设置状态回调函数"""
        self.status_callback = callback

    def log_status(self, message):
        """记录状态信息"""
        if self.status_callback:
            self.status_callback(message)

    def find_available_ports(self):
        """查找可用的串口"""
        ports = []
        for port in serial.tools.list_ports.comports():
            ports.append(port.device)
        self.log_status(f"找到可用串口: {ports}")
        return ports

    def handshake_with_device(self, test_serial):
        """与设备进行握手"""
        for cmd in self.handshake_commands:
            try:
                # 清空输入缓存
                test_serial.flushInput()

                # 发送握手命令
                test_serial.write((cmd + '\n').encode('utf-8'))
                self.log_status(f"发送握手命令: {cmd}")

                # 等待响应
                start_time = time.time()
                while time.time() - start_time < 1.0:  # 最多等待1秒
                    if test_serial.in_waiting > 0:
                        response = test_serial.readline().decode('utf-8', errors='ignore').strip().lower()
                        self.log_status(f"收到响应: {response}")

                        # 检查响应是否匹配预期
                        for expected_resp in self.handshake_responses:
                            if expected_resp in response:
                                self.log_status(f"握手成功: {cmd} -> {response}")
                                return True
                    time.sleep(0.01)  # 短暂休眠
            except Exception as e:
                self.log_status(f"握手过程中出错: {str(e)}")
                continue

        return False

    def auto_detect_and_connect(self):
        """自动检测并连接到合适的串口"""
        available_ports = self.find_available_ports()
        if not available_ports:
            self.log_status("未找到任何可用串口")
            return False

        self.log_status(f"正在尝试连接串口...")

        for port in available_ports:
            try:
                self.log_status(f"尝试连接到 {port}...")

                # 尝试打开串口
                test_serial = serial.Serial(
                    port=port,
                    baudrate=self.baudrate,
                    timeout=self.timeout
                )

                # 等待设备稳定
                time.sleep(0.5)

                # 与设备握手
                if self.handshake_with_device(test_serial):
                    self.log_status(f"在 {port} 上成功握手")

                    # 关闭测试连接
                    test_serial.close()

                    # 现在正式连接
                    self.serial_port = serial.Serial(
                        port=port,
                        baudrate=self.baudrate,
                        timeout=self.timeout
                    )

                    if self.serial_port.is_open:
                        self.log_status(f"成功连接到串口: {port}")
                        self.is_connected = True

                        # 启动接收线程
                        self.start_receiving()

                        return True
                    else:
                        self.log_status(f"无法打开串口: {port}")

                else:
                    self.log_status(f"在 {port} 上握手失败")

                # 关闭测试连接
                test_serial.close()

            except serial.SerialException as se:
                self.log_status(f"串口连接错误 {port}: {str(se)}")
                continue
            except Exception as e:
                self.log_status(f"无法连接到串口 {port}: {str(e)}")
                continue

        self.log_status("未能自动连接到任何串口")
        return False

    def connect_to_port(self, port_name):
        """手动连接到指定串口"""
        try:
            self.serial_port = serial.Serial(
                port=port_name,
                baudrate=self.baudrate,
                timeout=self.timeout
            )

            if self.serial_port.is_open:
                self.log_status(f"成功连接到串口: {port_name}")
                self.is_connected = True

                # 启动接收线程
                self.start_receiving()

                return True
            else:
                self.log_status(f"无法打开串口: {port_name}")
                return False

        except Exception as e:
            self.log_status(f"连接串口 {port_name} 失败: {str(e)}")
            return False

    def start_receiving(self):
        """启动数据接收线程"""
        if not self.is_connected:
            self.log_status("请先连接串口")
            return

        self.stop_flag = False
        self.receive_thread = threading.Thread(target=self._receive_loop)
        self.receive_thread.daemon = True
        self.receive_thread.start()
        self.log_status("数据接收线程已启动")

    def send_handshake_command(self):
        """发送握手命令"""
        if self.is_connected and self.serial_port and self.serial_port.is_open:
            # 尝试几种不同的握手命令
            for cmd in self.handshake_commands:
                try:
                    self.serial_port.write((cmd + '\n').encode('utf-8'))
                    self.log_status(f"发送握手命令: {cmd}")
                    time.sleep(0.1)  # 短暂延迟
                    break  # 发送第一个命令后退出
                except Exception as e:
                    self.log_status(f"发送握手命令失败: {str(e)}")
                    continue

    def _receive_loop(self):
        """数据接收循环"""
        handshake_sent = False
        last_data_time = time.time()

        while not self.stop_flag and self.is_connected:
            try:
                if self.serial_port and self.serial_port.is_open and self.serial_port.in_waiting > 0:
                    # 读取一行数据
                    try:
                        line = self.serial_port.readline().decode('utf-8', errors='ignore').strip()
                    except UnicodeDecodeError:
                        # 如果解码失败，尝试其他方式
                        raw_bytes = self.serial_port.read(self.serial_port.in_waiting or 1)
                        line = raw_bytes.decode('utf-8', errors='ignore').strip()

                    if line:
                        # 检查是否是握手响应
                        lower_line = line.lower()
                        is_handshake_response = any(resp in lower_line for resp in self.handshake_responses)

                        if is_handshake_response:
                            self.log_status(f"收到握手响应: {line}")
                            last_data_time = time.time()
                        else:
                            # 解析数据
                            parsed_data = self._parse_line(line)
                            if parsed_data is not None:
                                # 添加到数据队列
                                with self.lock:
                                    self.data_queue.append(parsed_data)

                                self.log_status(f"接收到数据: {parsed_data}")
                                last_data_time = time.time()

                # 如果还没有发送握手命令，定期发送
                if not handshake_sent:
                    self.send_handshake_command()
                    handshake_sent = True

                # 检查是否长时间没有收到数据
                if time.time() - last_data_time > 10:  # 10秒没收到数据
                    self.log_status("长时间未收到数据，尝试重新握手")
                    self.send_handshake_command()
                    last_data_time = time.time()

                time.sleep(0.01)  # 短暂休眠

            except Exception as e:
                self.log_status(f"接收数据时出错: {str(e)}")
                break

        self.log_status("数据接收线程结束")

    def _parse_line(self, line):
        """解析接收到的一行数据"""
        try:
            # 检查是否是握手响应，如果是则不解析为数据
            lower_line = line.lower()
            if any(resp in lower_line for resp in self.handshake_responses):
                return None

            # 支持多种分隔符: 空格、逗号、制表符等
            # 分割字符串，支持多种分隔符
            values = re.split(r'[,\s\t;]+', line.strip())
            values = [v for v in values if v]  # 移除空字符串

            # 转换为浮点数
            float_values = []
            for val in values:
                try:
                    # 处理带小数点的数字，如 "1.234"
                    if '.' in val:
                        float_val = float(val)
                    else:
                        float_val = float(val)
                    float_values.append(float_val)
                except ValueError:
                    continue  # 跳过无法转换的值

            # 检查数据维度
            if len(float_values) >= self.num_dimensions:
                return np.array(float_values[:self.num_dimensions])
            elif len(float_values) > 0:  # 如果有数据但不足维度数
                # 用最后一个值填充缺失的维度
                extended_values = float_values[:]
                while len(extended_values) < self.num_dimensions:
                    extended_values.append(float_values[-1])
                return np.array(extended_values[:self.num_dimensions])
            else:
                self.log_status(f"解析到空数据行: {line}")
                return None

        except Exception as e:
            self.log_status(f"解析数据行失败 '{line}': {str(e)}")
            return None

    def get_latest_data(self):
        """获取最新数据点"""
        with self.lock:
            if len(self.data_queue) > 0:
                return self.data_queue[-1].copy()  # 返回最新的数据点
            else:
                return None

    def get_new_data(self):
        """获取并清空新接收的数据"""
        with self.lock:
            new_data = self.data_queue.copy()
            self.data_queue.clear()  # 清空队列
            return new_data

    def disconnect(self):
        """断开串口连接"""
        self.log_status("正在断开串口连接...")
        self.stop_flag = True

        if self.receive_thread and self.receive_thread.is_alive():
            self.receive_thread.join(timeout=2)  # 等待最多2秒

        if self.serial_port and self.serial_port.is_open:
            self.serial_port.close()
            self.is_connected = False
            self.log_status("已断开串口连接")
        else:
            self.log_status("串口已关闭或不存在")

    def send_command(self, command):
        """发送命令到下位机"""
        if self.is_connected and self.serial_port and self.serial_port.is_open:
            try:
                self.serial_port.write((command + '\n').encode('utf-8'))
                self.log_status(f"发送命令: {command}")
                return True
            except Exception as e:
                self.log_status(f"发送命令失败: {str(e)}")
                return False
        else:
            self.log_status("串口未连接")
            return False

    def __del__(self):
        """析构函数，确保断开连接"""
        self.disconnect()


# 示例用法和测试代码
if __name__ == "__main__":
    # 测试串口数据接收器
    receiver = SerialDataReceiver(num_dimensions=3, baudrate=9600)


    # 设置状态回调
    def print_status(msg):
        print(msg)


    receiver.set_status_callback(print_status)

    # 自动检测并连接串口
    if receiver.auto_detect_and_connect():
        print("串口连接成功，开始接收数据...")

        # 等待几秒钟接收数据
        time.sleep(5)

        # 获取最新数据
        latest_data = receiver.get_latest_data()
        if latest_data is not None:
            print(f"最新数据: {latest_data}")

        # 获取新数据
        new_data = receiver.get_new_data()
        print(f"新接收的数据数量: {len(new_data)}")

        # 断开连接
        receiver.disconnect()
    else:
        print("无法连接到串口")




def get_resource_path(relative_path):
    """获取资源文件的绝对路径（兼容打包前后）"""
    if hasattr(sys, '_MEIPASS'):
        # 打包后：从临时解压目录获取文件
        return os.path.join(sys._MEIPASS, relative_path)
    # 开发环境：从当前目录获取文件
    return os.path.join(os.path.abspath("."), relative_path)


def load_data():
    # 使用动态获取的路径，而不是硬编码的相对路径
    file_path = get_resource_path('accel_data.txt')

    try:
        with open(file_path, 'r') as f:
            data = f.read()
        return data
    except Exception as e:
        # 添加错误日志以便调试
        print(f"❌ 读取文件失败: {file_path}")
        print(f"错误详情: {str(e)}")
        raise