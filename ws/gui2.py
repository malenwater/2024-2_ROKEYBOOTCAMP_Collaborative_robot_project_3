import sys
import subprocess
from PyQt5.QtWidgets import QApplication, QWidget, QPushButton, QVBoxLayout, QLabel
import signal
import os

class RobotControlUI(QWidget):
    def __init__(self):
        super().__init__()
        self.robot1_process = None
        self.robot2_process = None
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

        self.btn_return_base = QPushButton('거점 복귀', self)
        self.btn_return_base.clicked.connect(self.return_to_base)
        layout.addWidget(self.btn_return_base)

        self.setLayout(layout)
    
    def run_robot1(self):
        self.label.setText('1번 로봇 이동 중...')
        self.robot1_process = subprocess.Popen(["python3", "robot1_move.py"])

    def run_robot2(self):
        self.label.setText('2번 로봇 이동 중...')
        self.robot2_process = subprocess.Popen(["python3", "robot2_move.py"])
    
    def return_to_base(self):
        self.label.setText('로봇 거점 복귀 중...')
        # 실행 중인 로봇 프로세스 종료
        if self.robot1_process:
            os.kill(self.robot1_process.pid, signal.SIGTERM)
        if self.robot2_process:
            os.kill(self.robot2_process.pid, signal.SIGTERM)
        
        # 두 대의 로봇을 동시에 거점으로 복귀
        subprocess.Popen(["python3", "return_base.py"])

if __name__ == '__main__':
    app = QApplication(sys.argv)
    ex = RobotControlUI()
    ex.show()
    sys.exit(app.exec_())

