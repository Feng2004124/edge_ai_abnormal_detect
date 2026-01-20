import sys
from PyQt5.QtWidgets import (QApplication, QPushButton, QVBoxLayout, QHBoxLayout, QWidget, QTextEdit, QGroupBox, QLabel,
                             QSpinBox, QCheckBox, QDoubleSpinBox, QGridLayout)
from data_loader import DataLoader
from normalizer import Normalizer
from wave_plotter import WavePlotter
import functools  # 添加这行导入

# ================== 用户可配置参数 ==================#
# 数据文件路径
DATA_FILE = "accel_data.txt"
# 数据维度 (当前为3，未来支持更多维度)
NUM_DIMENSIONS = 3
# 采样时间间隔 (秒)
DT = 0.01
# 归一化范围 (默认为[-1, 1])
NORM_RANGE = (-1, 1)


# ====================================================

def main():
    # 1. 加载数据
    loader = DataLoader(DATA_FILE, NUM_DIMENSIONS)
    raw_data = loader.load_data()

    # 2. 归一化处理
    normalizer = Normalizer(NORM_RANGE)
    normalized_data = normalizer.normalize(raw_data)

    # 3. 创建主窗口
    app = QApplication(sys.argv)
    main_window = QWidget()
    main_window.setWindowTitle("异常检测数据监控")

    # 主布局 - 水平分割
    main_layout = QHBoxLayout()

    # 左侧：波形显示区域
    plotter = WavePlotter(data=normalized_data, dt=DT, num_dimensions=NUM_DIMENSIONS)
    main_layout.addWidget(plotter, 4)  # 占4份空间

    # 右侧：控制面板
    right_panel = QWidget()
    right_layout = QVBoxLayout()

    # 状态显示区域
    status_group = QGroupBox("系统状态")
    status_layout = QVBoxLayout()
    status_text = QTextEdit()
    status_text.setMaximumHeight(200)  # 增加高度
    status_text.setMinimumHeight(150)  # 设置最小高度
    status_text.setReadOnly(True)
    status_layout.addWidget(status_text)
    status_group.setLayout(status_layout)
    right_layout.addWidget(status_group)

    # 参数控制区域
    params_group = QGroupBox("参数控制")
    params_layout = QGridLayout()

    # 实时数据显示频率
    freq_label = QLabel("更新频率 (ms):")
    freq_spinbox = QSpinBox()
    freq_spinbox.setRange(10, 1000)
    freq_spinbox.setValue(50)

    # 维度控制
    dim_label = QLabel("数据维度:")
    dim_spinbox = QSpinBox()
    dim_spinbox.setRange(1, NUM_DIMENSIONS)
    dim_spinbox.setValue(NUM_DIMENSIONS)

    # 采样时间间隔
    dt_label = QLabel("采样间隔 (s):")
    dt_spinbox = QDoubleSpinBox()
    dt_spinbox.setRange(0.001, 1.0)
    dt_spinbox.setSingleStep(0.001)
    dt_spinbox.setValue(DT)
    dt_spinbox.setDecimals(4)

    # 添加到网格布局
    params_layout.addWidget(freq_label, 0, 0)
    params_layout.addWidget(freq_spinbox, 0, 1)
    params_layout.addWidget(dim_label, 1, 0)
    params_layout.addWidget(dim_spinbox, 1, 1)
    params_layout.addWidget(dt_label, 2, 0)
    params_layout.addWidget(dt_spinbox, 2, 1)
    params_group.setLayout(params_layout)
    right_layout.addWidget(params_group)

    # 维度显示开关
    dims_group = QGroupBox("维度显示")
    dims_layout = QVBoxLayout()
    dim_checkboxes = []
    dim_names = ["X轴", "Y轴", "Z轴", "维度4", "维度5", "维度6", "维度7", "维度8", "维度9", "维度10"]

    # 初始化维度复选框
    for i in range(min(10, NUM_DIMENSIONS)):
        checkbox = QCheckBox(dim_names[i])
        checkbox.setChecked(True)
        dim_checkboxes.append(checkbox)
        dims_layout.addWidget(checkbox)

    dims_group.setLayout(dims_layout)
    right_layout.addWidget(dims_group)

    # 控制按钮区域
    buttons_group = QGroupBox("控制按钮")
    buttons_layout = QVBoxLayout()

    start_real_time_btn = QPushButton("开始实时模式")
    stop_real_time_btn = QPushButton("停止实时模式")
    start_record_btn = QPushButton("开始记录数据")
    stop_record_btn = QPushButton("停止记录数据")
    enable_serial_btn = QPushButton("启用串口数据")
    disable_serial_btn = QPushButton("禁用串口数据")
    refresh_btn = QPushButton("界面刷新")

    buttons_layout.addWidget(start_real_time_btn)
    buttons_layout.addWidget(enable_serial_btn)
    buttons_layout.addWidget(disable_serial_btn)
    buttons_layout.addWidget(start_record_btn)
    buttons_layout.addWidget(stop_record_btn)
    buttons_layout.addWidget(refresh_btn)
    buttons_layout.addWidget(stop_real_time_btn)

    buttons_group.setLayout(buttons_layout)
    right_layout.addWidget(buttons_group)

    right_panel.setLayout(right_layout)
    main_layout.addWidget(right_panel, 1)  # 占1份空间

    main_window.setLayout(main_layout)
    main_window.resize(1400, 800)  # 增加窗口尺寸
    main_window.show()

    # 连接按钮事件
    def start_real_time():
        interval = freq_spinbox.value()
        plotter.start_real_time_mode(interval_ms=interval)

    def stop_real_time():
        plotter.stop_real_time_mode()

    def start_recording():
        plotter.start_recording()

    def stop_recording():
        plotter.stop_recording()

    def enable_serial():
        plotter.enable_serial_mode()
        status_text.append("正在尝试连接串口...")

    def disable_serial():
        plotter.disable_serial_mode()
        status_text.append("已禁用串口数据模式")

    def refresh_interface():
        plotter.refresh_interface()
        status_text.append("界面已刷新，清除记录数据和实时绘制")

    def change_num_dimensions():
        new_dim = dim_spinbox.value()

        # ✅ 修复1：确保维度不超过实际数据维度
        if new_dim > NUM_DIMENSIONS:
            new_dim = NUM_DIMENSIONS
            dim_spinbox.setValue(new_dim)
            status_text.append(f"⚠️ 维度上限为{NUM_DIMENSIONS}，已自动设置为{new_dim}")
            return

        # ✅ 修复2：调用plotter时使用有效维度
        plotter.change_num_dimensions(new_dim)

        # 界面更新逻辑 (保持不变)
        dims_layout = dims_group.layout()
        for checkbox in dim_checkboxes:
            try:
                checkbox.stateChanged.disconnect()
            except TypeError:
                pass

        for i in reversed(range(dims_layout.count())):
            child = dims_layout.itemAt(i).widget()
            if child:
                child.setParent(None)

        dim_checkboxes.clear()
        dim_names = ["X轴", "Y轴", "Z轴", "维度4", "维度5", "维度6", "维度7", "维度8", "维度9", "维度10"]
        for i in range(min(10, new_dim)):
            checkbox = QCheckBox(dim_names[i])
            checkbox.setChecked(True)
            dim_checkboxes.append(checkbox)
            dims_layout.addWidget(checkbox)

        for i, checkbox in enumerate(dim_checkboxes):
            checkbox.stateChanged.connect(functools.partial(plotter.set_display_dimension, i))

    def change_dt():
        new_dt = dt_spinbox.value()
        plotter.change_dt(new_dt)

    def validate_dim_input():
        current_value = dim_spinbox.value()
        if current_value < 1 or current_value > 10:
            status_text.append(f"警告: 数据维度超出范围 [1, 10]，当前值: {current_value}")

    def validate_dt_input():
        current_value = dt_spinbox.value()
        if current_value < 0.001 or current_value > 1.0:
            status_text.append(f"警告: 采样间隔超出范围 [0.001, 1.0]，当前值: {current_value}")

    # 连接事件
    start_real_time_btn.clicked.connect(start_real_time)
    stop_real_time_btn.clicked.connect(stop_real_time)
    start_record_btn.clicked.connect(start_recording)
    stop_record_btn.clicked.connect(stop_recording)
    enable_serial_btn.clicked.connect(enable_serial)
    disable_serial_btn.clicked.connect(disable_serial)
    refresh_btn.clicked.connect(refresh_interface)

    # 新增按钮事件
    dim_spinbox.valueChanged.connect(change_num_dimensions)
    dim_spinbox.valueChanged.connect(validate_dim_input)
    dt_spinbox.valueChanged.connect(change_dt)
    dt_spinbox.valueChanged.connect(validate_dt_input)

    # 将状态显示函数传递给plotter
    plotter.set_status_callback(lambda msg: status_text.append(msg))

    sys.exit(app.exec_())


if __name__ == "__main__":
    main()