import sys
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
from PyQt5 import QtWidgets, QtCore
import numpy as np

class PIDTunerGUI(Node):
    def __init__(self):
        super().__init__('pid_tuner_gui')

        # ROS Publishers
        self.pub_target = self.create_publisher(Float64MultiArray, '/target_positions', 100)
        self.pub_pid = self.create_publisher(Float64MultiArray, '/pi_gains', 100)

        # ROS Subscriber
        self.sub_joint_state = self.create_subscription(
            JointState, '/joint_states', self.joint_state_callback, 10
        )

        # Dati giunti
        self.joint_names = [
            'front_sh', 'front_knee', 'front_ank',
            'back_sh', 'back_knee', 'back_ank'
        ]
        self.current_positions = np.zeros(6)
        self.target_positions = np.zeros(6)
        self.pid_params = np.zeros((6,3))  # P,I,D per giunto

        # QSettings per salvataggio automatico
        self.settings = QtCore.QSettings("MyOrg", "PIDTunerGUI")
        self.load_settings()

        # GUI
        self.app = QtWidgets.QApplication(sys.argv)
        self.window = QtWidgets.QWidget()
        self.window.setWindowTitle('PID Tuner')
        self.main_layout = QtWidgets.QVBoxLayout()
        self.window.setLayout(self.main_layout)

        # --- Selettore giunto ---
        self.joint_selector = QtWidgets.QComboBox()
        self.joint_selector.addItems(self.joint_names)
        self.joint_selector.currentIndexChanged.connect(self.update_joint_selection)
        self.selected_joint = 0
        self.main_layout.addWidget(QtWidgets.QLabel("Select Joint:"))
        self.main_layout.addWidget(self.joint_selector)

        # --- PID Sliders + spinbox ---
        self.sliders_pid = []
        self.spin_pid = []
        for i, name in enumerate(['P','I','D']):
            layout = QtWidgets.QHBoxLayout()
            label = QtWidgets.QLabel(name)
            slider = QtWidgets.QSlider(QtCore.Qt.Horizontal)
            slider.setMinimum(0)
            slider.setMaximum(1000)
            slider.setValue(int(self.pid_params[0,i]*100))
            slider.valueChanged.connect(self.update_pid_from_slider)
            spin = QtWidgets.QDoubleSpinBox()
            spin.setMinimum(0)
            spin.setMaximum(10)
            spin.setSingleStep(0.01)
            spin.setValue(self.pid_params[0,i])
            spin.valueChanged.connect(self.update_pid_from_spinbox)
            layout.addWidget(label)
            layout.addWidget(slider)
            layout.addWidget(spin)
            self.main_layout.addLayout(layout)
            self.sliders_pid.append(slider)
            self.spin_pid.append(spin)

        # --- Target slider + spinbox ---
        layout_target = QtWidgets.QHBoxLayout()
        self.slider_target = QtWidgets.QSlider(QtCore.Qt.Horizontal)
        self.slider_target.setMinimum(-180)
        self.slider_target.setMaximum(180)
        self.slider_target.setValue(0)
        self.slider_target.valueChanged.connect(self.update_target_from_slider)
        self.spin_target = QtWidgets.QDoubleSpinBox()
        self.spin_target.setMinimum(-180)
        self.spin_target.setMaximum(180)
        self.spin_target.setSingleStep(1)
        self.spin_target.setValue(0)
        self.spin_target.valueChanged.connect(self.update_target_from_spinbox)
        layout_target.addWidget(QtWidgets.QLabel("Target"))
        layout_target.addWidget(self.slider_target)
        layout_target.addWidget(self.spin_target)
        self.main_layout.addLayout(layout_target)

        # --- Timer GUI ---
        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.update_gui)
        self.timer.start(50)  # GUI refresh a 20Hz

        # --- Timer Pubblicazione ROS a 100Hz ---
        self.pub_timer = QtCore.QTimer()
        self.pub_timer.timeout.connect(self.publish_all)
        self.pub_timer.start(10)  # 10ms = 100Hz

        self.window.show()
        self.window.closeEvent = self.closeEvent

    # --- ROS callback ---
    def joint_state_callback(self, msg: JointState):
        indices = [msg.name.index(j) for j in self.joint_names if j in msg.name]
        self.current_positions = np.array([msg.position[i] for i in indices])

    # --- Update sliders/spinbox ---
    def update_joint_selection(self):
        self.selected_joint = self.joint_selector.currentIndex()
        for i in range(3):
            self.sliders_pid[i].setValue(int(self.pid_params[self.selected_joint,i]*100))
            self.spin_pid[i].setValue(self.pid_params[self.selected_joint,i])
        self.slider_target.setValue(int(self.target_positions[self.selected_joint]*180/np.pi))
        self.spin_target.setValue(self.target_positions[self.selected_joint]*180/np.pi)

    def update_pid_from_slider(self):
        for i in range(3):
            val = self.sliders_pid[i].value()/100
            self.pid_params[self.selected_joint,i] = val
            self.spin_pid[i].setValue(val)

    def update_pid_from_spinbox(self):
        for i in range(3):
            val = self.spin_pid[i].value()
            self.pid_params[self.selected_joint,i] = val
            self.sliders_pid[i].setValue(int(val*100))

    def update_target_from_slider(self):
        val_rad = self.slider_target.value()*np.pi/180
        self.target_positions[self.selected_joint] = val_rad
        self.spin_target.setValue(float(self.slider_target.value()))

    def update_target_from_spinbox(self):
        val_rad = self.spin_target.value() * np.pi / 180
        self.target_positions[self.selected_joint] = val_rad
        self.slider_target.setValue(int(self.spin_target.value()))

    # --- Aggiorna solo GUI ---
    def update_gui(self):
        # Aggiorna eventuali valori GUI (ad es. slider) se vuoi
        pass

    # --- Pubblica tutti i joint a 100Hz ---
    def publish_all(self):
        # PID
        pid_msg = Float64MultiArray()
        pid_msg.data = self.pid_params.flatten().tolist()
        self.pub_pid.publish(pid_msg)

        # Target positions
        target_msg = Float64MultiArray()
        target_msg.data = self.target_positions.tolist()
        self.pub_target.publish(target_msg)

    # --- Salvataggio e caricamento ---
    def save_settings(self):
        for i, name in enumerate(self.joint_names):
            self.settings.setValue(f"{name}/P", self.pid_params[i,0])
            self.settings.setValue(f"{name}/I", self.pid_params[i,1])
            self.settings.setValue(f"{name}/D", self.pid_params[i,2])
            self.settings.setValue(f"{name}/Target", self.target_positions[i])
        self.settings.sync()

    def load_settings(self):
        for i, name in enumerate(self.joint_names):
            self.pid_params[i,0] = float(self.settings.value(f"{name}/P", 0.0))
            self.pid_params[i,1] = float(self.settings.value(f"{name}/I", 0.0))
            self.pid_params[i,2] = float(self.settings.value(f"{name}/D", 0.0))
            self.target_positions[i] = float(self.settings.value(f"{name}/Target", 0.0))

    def closeEvent(self, event):
        self.save_settings()
        event.accept()

    def run(self):
        sys.exit(self.app.exec_())

def main(args=None):
    rclpy.init(args=args)
    gui_node = PIDTunerGUI()
    try:
        gui_node.run()
    finally:
        gui_node.save_settings()
        gui_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
