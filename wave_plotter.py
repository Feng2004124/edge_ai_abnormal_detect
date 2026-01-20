import numpy as np
from PyQt5.QtWidgets import QWidget, QApplication
from PyQt5.QtCore import Qt, QPoint, QTimer
from PyQt5.QtGui import QPainter, QPen, QColor
import sys
from data_loader import DataSimulator, SerialDataReceiver  # 导入串口接收器


class WavePlotter(QWidget):
    """波形绘制器，支持静态、实时和记录波形显示"""

    def __init__(self, data=None, dt=0.01, num_dimensions=3, parent=None):
        super().__init__(parent)
        self.dt = dt
        self.num_dimensions = num_dimensions
        self.setWindowTitle("加速度波形显示")

        # 设置固定尺寸
        self.resize(1000, 600)
        self.setStyleSheet("background-color: #2e2e2e;")

        # 存储历史数据用于静态显示
        self.static_data = data
        self.time = np.arange(0, len(data) * dt, dt) if data is not None else np.array([])

        # 实时数据缓冲区
        self.real_time_buffer = []  # 存储所有实时数据点
        self.buffer_size = 1000  # 增大缓冲区以存储更多数据

        # 记录的数据
        self.recorded_data = []  # 存储记录的数据点
        self.is_recording = False  # 记录状态标志

        # 实时模式标志
        self.is_real_time_mode = False
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_real_time_data)

        # 初始化实时数据模拟器
        self.simulator = DataSimulator(num_dimensions)

        # 初始化串口数据接收器
        self.serial_receiver = SerialDataReceiver(num_dimensions=num_dimensions)
        self.use_serial_data = False  # 是否使用串口数据

        # 状态回调函数
        self.status_callback = None

        # 显示控制
        self.display_dimensions = [True] * num_dimensions  # 控制哪些维度显示

    def set_status_callback(self, callback):
        """设置状态回调函数"""
        self.status_callback = callback

    def log_status(self, message):
        """记录状态信息"""
        if self.status_callback:
            self.status_callback(message)

    def change_num_dimensions(self, new_num_dimensions):
        """更改数据维度"""
        old_num_dimensions = self.num_dimensions
        self.num_dimensions = new_num_dimensions

        # 更新显示维度控制数组
        old_display_state = self.display_dimensions[:]
        self.display_dimensions = [False] * new_num_dimensions

        # 保留原有的显示状态
        for i in range(min(len(old_display_state), new_num_dimensions)):
            self.display_dimensions[i] = old_display_state[i]

        # 如果新维度大于原维度，设置新增维度为显示状态
        for i in range(len(old_display_state), new_num_dimensions):
            self.display_dimensions[i] = True

        # 重新初始化模拟器
        self.simulator = DataSimulator(new_num_dimensions)

        # 更新串口接收器
        self.serial_receiver = SerialDataReceiver(num_dimensions=new_num_dimensions)

        self.log_status(f"数据维度已从 {old_num_dimensions} 更改为 {new_num_dimensions}")

        # 重绘
        self.update()

    def change_dt(self, new_dt):
        """更改采样时间间隔"""
        old_dt = self.dt
        self.dt = new_dt
        self.log_status(f"采样时间间隔已从 {old_dt} 更改为 {new_dt}")

        # 更新时间轴
        if self.static_data is not None:
            self.time = np.arange(0, len(self.static_data) * self.dt, self.dt)

    def refresh_interface(self):
        """刷新界面 - 清除记录数据和实时绘制"""
        self.log_status("界面已刷新，清除记录数据和实时绘制")

        # 清除记录数据
        self.recorded_data = []
        self.static_data = None
        self.time = np.array([])

        # 清除实时数据
        self.real_time_buffer = []

        # 停止实时模式
        self.is_real_time_mode = False
        self.timer.stop()

        # 重置记录状态
        self.is_recording = False

        # 重绘
        self.update()

    def start_real_time_mode(self, interval_ms=50):
        """启动实时模式"""
        self.is_real_time_mode = True
        self.timer.start(interval_ms)
        self.log_status(f"启动实时模式，更新频率: {interval_ms}ms")
        self.update()  # 触发重绘

    def stop_real_time_mode(self):
        """停止实时模式并切换到记录数据"""
        self.is_real_time_mode = False
        self.timer.stop()
        self.log_status("停止实时模式，开始绘制记录数据！")

        # 如果有记录的数据，则切换到显示记录的数据
        if len(self.recorded_data) > 0:
            # 将记录的数据转换为numpy数组
            recorded_array = np.array(self.recorded_data)
            # 更新静态数据显示
            self.static_data = recorded_array
            self.time = np.arange(0, len(recorded_array) * self.dt, self.dt)

        self.update()  # 重绘

    def start_recording(self):
        """开始记录数据"""
        self.is_recording = True
        self.recorded_data = []  # 清空之前的记录
        self.log_status("开始记录数据...")

    def stop_recording(self):
        """停止记录数据"""
        self.is_recording = False
        self.log_status(f"记录结束，共记录 {len(self.recorded_data)} 个数据点")

    def enable_serial_mode(self):
        """启用串口数据模式"""
        self.log_status("正在尝试连接串口...")
        success = self.serial_receiver.auto_detect_and_connect()
        if success:
            self.use_serial_data = True
            self.log_status("已启用串口数据模式")
        else:
            self.log_status("串口连接失败，使用模拟数据")
            self.use_serial_data = False

    def disable_serial_mode(self):
        """禁用串口数据模式"""
        self.use_serial_data = False
        self.serial_receiver.disconnect()
        self.log_status("已禁用串口数据模式，使用模拟数据")

    def update_real_time_data(self):
        """更新实时数据并重绘"""
        # 根据模式获取数据
        if self.use_serial_data and self.serial_receiver.is_connected:
            # 从串口获取最新数据
            new_data_point = self.serial_receiver.get_latest_data()
            if new_data_point is None:
                # 如果没有新数据，使用上次的数据或模拟数据
                if len(self.real_time_buffer) > 0:
                    new_data_point = self.real_time_buffer[-1]
                else:
                    new_data_point = self.simulator.get_next_data_point()
        else:
            # 使用模拟数据
            new_data_point = self.simulator.get_next_data_point()

        if new_data_point is not None:
            # 添加到实时缓冲区
            self.real_time_buffer.append(new_data_point)

            # 限制缓冲区大小
            if len(self.real_time_buffer) > self.buffer_size:
                self.real_time_buffer.pop(0)

            # 如果正在记录，则同时保存到记录数据中
            if self.is_recording:
                self.recorded_data.append(new_data_point.copy())

            # 触发重绘
            self.update()

    def add_real_time_data(self, data_point):
        """外部接口：添加实时数据点"""
        if len(data_point) != self.num_dimensions:
            raise ValueError(f"数据点维度应为 {self.num_dimensions}")

        self.real_time_buffer.append(np.array(data_point))

        # 限制缓冲区大小
        if len(self.real_time_buffer) > self.buffer_size:
            self.real_time_buffer.pop(0)

        # 如果正在记录，则同时保存到记录数据中
        if self.is_recording:
            self.recorded_data.append(np.array(data_point).copy())

        # 触发重绘
        self.update()

    def set_display_dimension(self, dim_index, visible):
        """设置指定维度是否显示"""
        if 0 <= dim_index < len(self.display_dimensions):
            self.display_dimensions[dim_index] = visible
            self.update()  # 重绘

    def draw_real_time_wave(self, painter, dimension):
        """绘制实时波形（固定左端，持续扩展）"""
        if not self.real_time_buffer or not self.display_dimensions[dimension]:
            return

        colors = [(255, 0, 0), (0, 255, 0), (0, 0, 255), (255, 255, 0), (255, 0, 255),
                  (0, 255, 255), (128, 128, 128), (128, 0, 128), (0, 128, 128), (128, 128, 0)]
        color = colors[dimension % len(colors)]

        pen = QPen()
        pen.setColor(QColor(*color))
        pen.setWidth(2)
        painter.setPen(pen)

        # 计算缩放因子 - 根据实际数据长度调整
        total_data_points = len(self.real_time_buffer)
        if total_data_points == 0:
            return

        # 计算当前可视区域能显示多少数据点
        available_width = self.width() - 70  # 减去左边距
        x_scale = available_width / max(1, total_data_points)  # 动态缩放

        # 如果数据点过多，需要压缩显示
        if total_data_points > available_width:
            # 计算采样步长
            step = max(1, total_data_points // available_width)
            sampled_indices = range(0, total_data_points, step)
        else:
            # 数据点较少，全部显示
            sampled_indices = range(total_data_points)

        # 构建点列表
        points = []
        for i in sampled_indices:
            data_point = self.real_time_buffer[i]
            # x位置基于索引计算，保持左侧固定
            x = 50 + (i * available_width / total_data_points)
            # 归一化值转换为屏幕坐标
            y = self.height() // 2 - int(data_point[dimension] * (self.height() // 4))
            points.append((int(x), int(y)))

        # 绘制多边形线
        if len(points) > 1:
            painter.drawPolyline([QPoint(x, y) for x, y in points])

    def draw_recorded_wave(self, painter, dimension):
        """绘制记录的波形"""
        if len(self.recorded_data) == 0 or not self.display_dimensions[dimension]:
            return

        colors = [(255, 0, 0), (0, 255, 0), (0, 0, 255), (255, 255, 0), (255, 0, 255),
                  (0, 255, 255), (128, 128, 128), (128, 0, 128), (0, 128, 128), (128, 128, 0)]
        color = colors[dimension % len(colors)]

        pen = QPen()
        pen.setColor(QColor(*color))
        pen.setWidth(2)  # 记录波形使用粗线
        painter.setPen(pen)

        # 计算缩放因子
        total_data_points = len(self.recorded_data)
        if total_data_points == 0:
            return

        available_width = self.width() - 70  # 减去左边距
        x_scale = available_width / max(1, total_data_points)  # 动态缩放

        # 如果数据点过多，需要压缩显示
        if total_data_points > available_width:
            # 计算采样步长
            step = max(1, total_data_points // available_width)
            sampled_indices = range(0, total_data_points, step)
        else:
            # 数据点较少，全部显示
            sampled_indices = range(total_data_points)

        # 构建点列表
        points = []
        for i in sampled_indices:
            data_point = self.recorded_data[i]
            # x位置基于索引计算，保持左侧固定
            x = 50 + (i * available_width / total_data_points)
            y = self.height() // 2 - int(data_point[dimension] * (self.height() // 4))
            points.append((int(x), int(y)))

        # 绘制多边形线
        if len(points) > 1:
            painter.drawPolyline([QPoint(x, y) for x, y in points])

    def draw_static_wave(self, painter, dimension):
        """绘制静态波形"""
        if self.static_data is None or len(self.static_data) == 0 or not self.display_dimensions[dimension]:
            return

        colors = [(255, 0, 0), (0, 255, 0), (0, 0, 255), (255, 255, 0), (255, 0, 255),
                  (0, 255, 255), (128, 128, 128), (128, 0, 128), (0, 128, 128), (128, 128, 0)]
        color = colors[dimension % len(colors)]

        pen = QPen()
        pen.setColor(QColor(*color))
        pen.setWidth(1)  # 静态波形使用细线
        painter.setPen(pen)

        # 计算缩放因子
        total_data_points = len(self.static_data)
        if total_data_points == 0:
            return

        available_width = self.width() - 70  # 减去左边距
        x_scale = available_width / max(1, total_data_points)  # 动态缩放

        # 如果数据点过多，需要压缩显示
        if total_data_points > available_width:
            # 计算采样步长
            step = max(1, total_data_points // available_width)
            sampled_indices = range(0, total_data_points, step)
        else:
            # 数据点较少，全部显示
            sampled_indices = range(total_data_points)

        # 构建点列表
        points = []
        for i in sampled_indices:
            # x位置基于索引计算，保持左侧固定
            x = 50 + (i * available_width / total_data_points)
            y = self.height() // 2 - int(self.static_data[i, dimension] * (self.height() // 4))
            points.append((int(x), int(y)))

        # 绘制多边形线
        if len(points) > 1:
            painter.drawPolyline([QPoint(x, y) for x, y in points])

    def paintEvent(self, event):
        """重绘事件"""
        if self.width() <= 0 or self.height() <= 0:
            return

        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)

        # 绘制坐标轴
        self.draw_axes(painter)

        if self.is_real_time_mode:
            # 绘制实时波形
            for dim in range(self.num_dimensions):
                if self.display_dimensions[dim]:  # 只绘制选中的维度
                    self.draw_real_time_wave(painter, dim)

            # 绘制实时标签
            self.draw_real_time_labels(painter)
        else:
            # 绘制记录的数据（如果存在）或原始静态数据
            if len(self.recorded_data) > 0 and self.static_data is not None and len(self.static_data) > 0:
                # 优先显示记录的数据（已更新到static_data中）
                for dim in range(self.num_dimensions):
                    if self.display_dimensions[dim]:  # 只绘制选中的维度
                        self.draw_static_wave(painter, dim)
            elif self.static_data is not None and len(self.static_data) > 0:
                # 显示原始静态数据
                for dim in range(self.num_dimensions):
                    if self.display_dimensions[dim]:  # 只绘制选中的维度
                        self.draw_static_wave(painter, dim)
            elif len(self.recorded_data) > 0:
                # 如果只有记录数据而没有原始静态数据
                for dim in range(self.num_dimensions):
                    if self.display_dimensions[dim]:  # 只绘制选中的维度
                        self.draw_recorded_wave(painter, dim)

    def draw_axes(self, painter):
        """绘制坐标轴"""
        pen = QPen()
        pen.setColor(QColor(200, 200, 200))
        pen.setWidth(2)
        painter.setPen(pen)

        # 绘制x轴
        x_start = 50
        x_end = self.width() - 20
        y_center = self.height() // 2
        painter.drawLine(x_start, y_center, x_end, y_center)

        # 绘制y轴
        y_start = 30
        y_end = self.height() - 30
        painter.drawLine(50, y_start, 50, y_end)

        self.draw_labels(painter, x_start, x_end, y_center, y_start, y_end)

    def draw_labels(self, painter, x_start, x_end, y_center, y_start, y_end):
        """绘制坐标轴标签"""
        # 使用黑色字体
        painter.setPen(QColor(0, 0, 0))
        painter.setFont(self.font(10))

        # 显示当前模式
        if self.is_real_time_mode:
            mode_text = "实时模式"
            if self.use_serial_data:
                mode_text += " (串口)"
            else:
                mode_text += " (模拟)"
            if self.is_recording:
                mode_text += f" (记录中, {len(self.recorded_data)}点)"
        elif len(self.recorded_data) > 0:
            mode_text = f"记录数据 ({len(self.recorded_data)} 点)"
        else:
            mode_text = "静态模式"

        painter.drawText(x_end - 180, 20, mode_text)

        # 显示当前显示的数据范围
        if self.is_real_time_mode:
            total_points = len(self.real_time_buffer)
            if total_points > 0:
                time_range = total_points * self.dt
                painter.drawText(50, 20, f"时间: 0 - {time_range:.2f}s ({total_points}点)")

        painter.drawText(x_end - 80, y_center + 20, "时间 (s)")
        painter.drawText(20, y_start - 10, "幅值")
        painter.drawText(20, y_end + 20, "幅值")

        self.draw_ticks(painter, x_start, x_end, y_center, y_start, y_end)

    def draw_ticks(self, painter, x_start, x_end, y_center, y_start, y_end):
        """绘制坐标轴刻度"""
        # X轴刻度
        num_ticks = 5
        for i in range(1, num_ticks):
            x_pos = x_start + (x_end - x_start) * i / max(1, num_ticks)
            painter.drawLine(int(x_pos), y_center - 3, int(x_pos), y_center + 3)

        # Y轴刻度
        num_ticks = 5
        for i in range(1, num_ticks):
            y_pos = y_start + (y_end - y_start) * i / max(1, num_ticks)
            painter.drawLine(50 - 3, int(y_pos), 50 + 3, int(y_pos))

    def draw_real_time_labels(self, painter):
        """绘制实时模式下的标签"""
        # 根据维度数量生成标签
        labels = []
        for i in range(self.num_dimensions):
            if i < 3:
                labels.append(["X轴", "Y轴", "Z轴"][i])
            elif i == 3:
                labels.append("温度")  # 特殊处理第4维为温度
            else:
                labels.append(f"维度{i + 1}")

        colors = [(255, 0, 0), (0, 255, 0), (0, 0, 255), (255, 255, 0), (255, 0, 255),
                  (0, 255, 255), (128, 128, 128), (128, 0, 128), (0, 128, 128), (128, 128, 0)]

        visible_dims = sum(self.display_dimensions)  # 计算可见维度数量
        displayed_count = 0
        for i in range(min(self.num_dimensions, len(labels))):
            if self.display_dimensions[i]:  # 只显示选中的维度
                painter.setFont(self.font(10))
                painter.setPen(QColor(*colors[i]))
                source = " (串口)" if self.use_serial_data else " (模拟)"
                status = " (记录)" if self.is_recording else ""
                painter.drawText(self.width() - 100, self.height() - 30 - displayed_count * 20,
                                 f"{labels[i]}{source}{status}")
                displayed_count += 1

    def font(self, size):
        """创建字体对象"""
        from PyQt5.QtGui import QFont
        font = QFont()
        font.setPointSize(size)
        return font