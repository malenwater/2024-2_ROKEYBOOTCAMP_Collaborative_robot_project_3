import sys
from PyQt5.QtWidgets import QApplication, QWidget, QPushButton, QVBoxLayout, QLabel, QHBoxLayout
from PyQt5.QtGui import QPixmap, QImage
from .UI_Node import ROS2Thread
import cv2
class RobotControlUI(QWidget):
    def __init__(self):
        super().__init__()
        self.initUI()

        self.ros_thread = ROS2Thread()
        self.ros_thread.image_signal_tb1.connect(self.update_label_img1)
        self.ros_thread.image_signal_tb2.connect(self.update_label_img2)
        self.ros_thread.start()

    def initUI(self):
        self.setWindowTitle('다중 로봇 제어')
        self.setGeometry(100, 100, 800, 500)

        layout = QVBoxLayout()
        img_layout = QHBoxLayout()
        self.label_img1 = QLabel(self)
        self.label_img2 = QLabel(self)
        img_layout.addWidget(self.label_img1)
        img_layout.addWidget(self.label_img2)
        layout.addLayout(img_layout)

        btn_layout = QHBoxLayout()
        buttons = [("순찰", self.run_patrol), ("정지", self.stop_robot),
                   ("복귀", self.return_to_base), ("궤도 포격", self.run_orbital_strike),
                   ("광물 회수", self.collect_minerals),("광물 생성", self.generate_minerals)]
        
        for text, method in buttons:
            btn = QPushButton(text, self)
            btn.clicked.connect(method)
            btn_layout.addWidget(btn)
        
        layout.addLayout(btn_layout)
        self.setLayout(layout)

    def update_label_img1(self, img):
        self.set_label_image(self.label_img1, img)

    def update_label_img2(self, img):
        self.set_label_image(self.label_img2, img)

    def set_label_image(self, label, img):
        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)  # BGR을 RGB로 변환
        height, width, channel = img.shape
        bytes_per_line = 3 * width
        qimg = QImage(img.data, width, height, bytes_per_line, QImage.Format_RGB888)
        pixmap = QPixmap.fromImage(qimg)
        label.setPixmap(pixmap)


    def run_patrol(self):
        self.ros_thread.node.publish_Command("1", "1")
        self.ros_thread.node.publish_Command("2", "1")

    def stop_robot(self):
        self.ros_thread.node.publish_Command("1", "3")
        self.ros_thread.node.publish_Command("2", "3")

    def return_to_base(self):
        self.ros_thread.node.publish_Command("1", "2")
        self.ros_thread.node.publish_Command("2", "2")

    def run_orbital_strike(self):
        self.ros_thread.node.publish_Command("0", "4")
        
    def collect_minerals(self):
        self.ros_thread.node.publish_Command("0", "6")
        
    def generate_minerals(self):
        self.ros_thread.node.publish_Command("0", "5")

def main():
    app = QApplication(sys.argv)
    ex = RobotControlUI()
    ex.show()
    sys.exit(app.exec_())
if __name__ == '__main__':
    main()
