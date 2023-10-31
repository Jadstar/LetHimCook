# GUI.py
from PyQt5.QtWidgets import QApplication, QWidget, QGraphicsView, QGraphicsScene, QGraphicsEllipseItem, QLabel
from PyQt5.QtWidgets import QVBoxLayout, QLabel, QSlider, QPushButton, QSpinBox, QHBoxLayout
from PyQt5.QtCore import QTimer
import random
from PyQt5.QtGui import QBrush, QColor
from PyQt5.QtCore import Qt
from cooking import FLIP_TEMP, DONE_TEMP
from cooking import Patty
import sys
import swift

class PattyVisualizer(QWidget):
    def __init__(self,pattylist,env):
        super().__init__()
        self.pattylist = pattylist
        self.env = env
        self.initUI()

    def initUI(self):
        self.setWindowTitle('Patty Cooking Simulator')
        self.setGeometry(100, 100, 400, 400)
        self.layout = QVBoxLayout()
        self.labels = []
        for patty in self.pattylist:
            label = QLabel(f"Patty Temperature: {patty.temperature}°C")
            self.labels.append(label)
            self.layout.addWidget(label)

        # self.view = QGraphicsView(self)
        # self.view.setGeometry(50, 50, 300, 300)
        # # self.scene = QGraphicsScene(self)
        # self.view.setScene(self.scene)

        self.label = QLabel(self)
        self.label.setGeometry(100, 350, 200, 40)

        # Create a container widget for the labels
        labels_container = QWidget()
        labels_layout = QVBoxLayout()
        for label in self.labels:
            labels_layout.addWidget(label)
        labels_container.setLayout(labels_layout)
        self.layout.addWidget(labels_container)

        # Emergency Stop Button
        self.emergency_stop_btn = QPushButton("Emergency Stop", self)
        self.emergency_stop_btn.clicked.connect(self.handle_emergency_stop)
        self.layout.addWidget(self.emergency_stop_btn)

        self.setLayout(self.layout)

    def handle_emergency_stop(self):
        if not self.emergency_stop_requested:  # If currently in the "Emergency Stop" state
            self.emergency_stop_requested = True
            self.emergency_stop_btn.setStyleSheet("background-color: green")
            self.emergency_stop_btn.setText("START")
        else:  # If currently in the "START" state
            self.show_confirmation_dialog()

    def updateTemperatures(self):
        for i, patty in enumerate(self.pattylist):
            patty.heat(random.uniform(0, 2),self.env)  # Simulate random heating
            self.labels[i].setText(f"Patty Temperature: {patty.temperature:.1f}°C")

    def show_confirmation_dialog(self):
        from PyQt5.QtWidgets import QMessageBox
        msg_box = QMessageBox(self)
        msg_box.setWindowTitle("Confirmation")
        msg_box.setText("Are you sure the area is safe to start?")
        msg_box.setStandardButtons(QMessageBox.Yes | QMessageBox.No)
        msg_box.setDefaultButton(QMessageBox.No)
        reply = msg_box.exec_()
        if reply == QMessageBox.Yes:
            self.emergency_stop_requested = False
            self.emergency_stop_btn.setStyleSheet("background-color: red")
            self.emergency_stop_btn.setText("Emergency Stop")
            # Additional logic to start the process can be added here

    def update_display(self, temperature):
        self.label.setText(f"Current Patty Temperature: {temperature}°C")
        if temperature < FLIP_TEMP:
            color = Qt.red
        elif temperature < DONE_TEMP:
            color = Qt.yellow
        else:
            color = Qt.green
        self.patty.setBrush(QBrush(color))
if __name__ == '__main__':
    env = swift.Swift()
    env.launch(realtime=True)


    app = QApplication(sys.argv)

    # Create a list of Patty objects
    pattylist = [Patty(), Patty(), Patty()]

    window = PattyVisualizer(pattylist,env)
    window.setWindowTitle('Patty Temperature Visualizer')
    window.setGeometry(100, 100, 400, 200)
    window.show()

    sys.exit(app.exec_())
