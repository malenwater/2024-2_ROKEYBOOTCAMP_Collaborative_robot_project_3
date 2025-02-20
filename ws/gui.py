import sys
import subprocess
from PyQt5.QtWidgets import QApplication, QWidget, QPushButton, QVBoxLayout, QLabel

class RobotControlUI(QWidget):
    def __init__(self):
        super().__init__()

        self.initUI()
    
    def initUI(self):
        self.setWindowTitle('다중 로봇 제어')
        self.setGeometry(100, 100, 300, 200)

        layout = QVBoxLayout()

        self.label = QLabel('로봇을 선택하여 실행하세요', self)
        layout.addWidget(self.label)

        self.btn_robot1 = QPushButton('1번 로봇 이동 시작', self)
        self.btn_robot1.clicked.connect(self.run_robot1)
        layout.addWidget(self.btn_robot1)

        self.btn_robot2 = QPushButton('2번 로봇 이동 시작', self)
        self.btn_robot2.clicked.connect(self.run_robot2)
        layout.addWidget(self.btn_robot2)

        self.setLayout(layout)
    
    def run_robot1(self):
        self.label.setText('1번 로봇 이동 중...')
        subprocess.Popen(["python3", "1.py"]) # <- 1.py

    def run_robot2(self):
        self.label.setText('2번 로봇 이동 중...')
        subprocess.Popen(["python3", "2.py"]) # <- 2.py 

if __name__ == '__main__':
    app = QApplication(sys.argv)
    ex = RobotControlUI()
    ex.show()
    sys.exit(app.exec_())

