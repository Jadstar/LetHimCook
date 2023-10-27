# GUI.py
from PyQt5.QtWidgets import QWidget, QGraphicsView, QGraphicsScene, QGraphicsEllipseItem, QLabel
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

    def update_display(self, temperature):
        self.label.setText(f"Current Patty Temperature: {temperature}°C")
        if temperature < FLIP_TEMP:
            color = Qt.red
        elif temperature < DONE_TEMP:
            color = Qt.yellow
        else:
            color = Qt.green
        self.patty.setBrush(QBrush(color))
