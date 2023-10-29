# GUI.py
from PyQt5.QtWidgets import QWidget, QGraphicsView, QGraphicsScene, QGraphicsEllipseItem, QLabel
from PyQt5.QtWidgets import QVBoxLayout, QLabel, QSlider, QPushButton, QSpinBox, QHBoxLayout

from PyQt5.QtGui import QBrush, QColor
from PyQt5.QtCore import Qt
from cooking import FLIP_TEMP, DONE_TEMP

class PattyVisualizer(QWidget):
    def __init__(self):
        super().__init__()
        self.initUI()

    def initUI(self):
        # Window properties
        self.setWindowTitle('Patty Cooking Simulator')
        self.setGeometry(100, 100, 400, 400)
        
        # Graphics view
        self.view = QGraphicsView(self)
        self.view.setGeometry(50, 50, 300, 300)
        self.scene = QGraphicsScene(self)
        self.view.setScene(self.scene)
        
        # Patty as an ellipse item
        self.patty = QGraphicsEllipseItem(125, 125, 50, 50)
        self.patty.setBrush(QBrush(Qt.red))
        self.scene.addItem(self.patty)
        
        # Temperature label
        self.label = QLabel(self)
        self.label.setGeometry(100, 350, 200, 40)

        # Emergency Stop Button
        self.emergency_stop_btn = QPushButton("Emergency Stop", self)
        self.emergency_stop_btn.setGeometry(125, 380, 150, 40)
        self.emergency_stop_requested = False  # Flag to indicate if emergency stop was requested
        self.emergency_stop_btn.clicked.connect(self.handle_emergency_stop)

    def handle_emergency_stop(self):
        if not self.emergency_stop_requested:  # If currently in the "Emergency Stop" state
            self.emergency_stop_requested = True
            self.emergency_stop_btn.setStyleSheet("background-color: green")
            self.emergency_stop_btn.setText("START")
        else:  # If currently in the "START" state
            self.show_confirmation_dialog()

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
