#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'TestDes1.ui'
#
# Created by: PyQt5 UI code generator 5.6
#
# WARNING! All changes made in this file will be lost!

from PyQt5 import QtCore, QtGui, QtWidgets
import time
import numpy as np
import RPi.GPIO as GPIO
import copy


class MotorCommand():
    def __init__(self, type, start, end, speed):
        self.type = type
        self.start = start
        self.end = end
        self.speed = abs(speed)
        if speed < 0:
            self.dir = False
        else:
            self.dir = True


class WorkerObject(QtCore.QThread):
    ang_pos = QtCore.pyqtSignal(float)
    ang_speed = QtCore.pyqtSignal(float)
    time_left = QtCore.pyqtSignal(float)
    com_comp_perc = QtCore.pyqtSignal(float)
    overall_comp_perc = QtCore.pyqtSignal(float)
    overall_step = QtCore.pyqtSignal(int)
    fin_flag = QtCore.pyqtSignal(bool)
    fin_angle = QtCore.pyqtSignal(float)

    def __init__(self):
        super(WorkerObject, self).__init__()
        self.commands = []
        self.current_angle = 1
        self.threadactive = True
    
    def Raw(self, j):
        dirpin = 11
        steppin = 7
        enpin = 13

        self.overall_step.emit(j)
        start_angle = (self.commands[j]).start
        final_angle = (self.commands[j]).end
        delta_angle = final_angle - start_angle
        cum_tot = 0
        inc = 3.6 / 30
        speed_rpm = (self.commands[j]).speed
        speed_ang = 6 * speed_rpm
        time_list = []
        cur_angle = self.current_angle

        if not self.commands[j].dir:
            GPIO.output(dirpin, GPIO.HIGH)
            if delta_angle >= 0:
                delta_angle += (-1) * 360
        else:
            GPIO.output(dirpin, GPIO.LOW)
            if delta_angle <= 0:
                delta_angle += 360

        num_steps = int(abs(delta_angle) / inc)
        t_step = (inc / speed_ang)
    
        for i in range(num_steps):
            if self.threadactive:
                GPIO.output(steppin, GPIO.HIGH)
                time.sleep(t_step*0.2)
                GPIO.output(steppin, GPIO.LOW)
                time.sleep(t_step*0.8)
                cur_angle += inc
        t_rem = (num_steps - i - 1) * t_step
        self.ang_pos.emit(cur_angle)
        self.ang_speed.emit(speed_rpm)
        self.time_left.emit(t_rem)
        self.current_angle = cur_angle

    def RotateCommand(self, j, cur_angle):
        dirpin = 11
        steppin = 7
        enpin = 13

        self.overall_step.emit(j)
        start_angle = (self.commands[j]).start
        final_angle = (self.commands[j]).end
        delta_angle = final_angle - start_angle
        cum_tot = 0
        inc = 3.6 / 30
        speed_rpm = (self.commands[j]).speed
        speed_ang = 6 * speed_rpm
        cur_angle = self.current_angle
        time_list = []

        if not self.commands[j].dir:
            GPIO.output(dirpin, GPIO.HIGH)
            if delta_angle >= 0:
                delta_angle += (-1) * 360
        else:
            GPIO.output(dirpin, GPIO.LOW)
            if delta_angle <= 0:
                delta_angle += 360

        num_steps = int(abs(delta_angle) / inc)
        t_step = (inc / speed_ang)
        # First Pulse Length
        del1 = t_step * 0.05
        # Time estimate for time.time commands
        del2 = -0.00016
        # del2 = 0
        delay = t_step - del1 + del2
        act_v = speed_rpm

        tot_lag = 0

        exe_times = np.zeros([num_steps])
        vels = np.zeros([num_steps])
        beg = time.time()
        bl = beg

        for i in range(num_steps):
            if self.threadactive:

                GPIO.output(steppin, GPIO.HIGH)
                time.sleep(del1)
                # start loop timer and make copy
                time_a = time.time()
                sp_s = time_a

                GPIO.output(steppin, GPIO.LOW)

                if self.commands[j].dir:
                    cur_angle += inc
                else:
                    cur_angle += (-1) * inc

                if cur_angle < 0:
                    cur_angle += 360
                elif cur_angle > 360:
                    cur_angle += (-360)

                if i > 0:
                    # last_exe_time = sp_s-bl
                    exe_times[i] = sp_s - bl
                    last_lag = exe_times[i] - t_step
                    av_exe = np.mean(exe_times[1:i + 1])
                    av_lag = av_exe - t_step
                    tot_lag += last_lag
                    # print(av_exe)
                    act_v = (inc / exe_times[i]) * (1 / 6.0)
                    vels[i] = act_v

                cum_tot += inc
                t_rem = (num_steps - i - 1) * t_step
                prog = (cum_tot / abs(delta_angle)) * 100
                bl = sp_s

                time_b = time.time()
                iv = time_b - time_a
                new_delay = delay - iv - (tot_lag * 0.99)

                if new_delay > 0:
                    time.sleep(new_delay)

                self.ang_pos.emit(cur_angle)
                self.ang_speed.emit(act_v)
                self.time_left.emit(t_rem)
                self.com_comp_perc.emit(prog)

            if len(exe_times[np.nonzero(exe_times)]) > 0:
                av_time = np.mean(exe_times[np.nonzero(exe_times)])
                self.ang_speed.emit((inc / av_time) / 6)

        self.ang_pos.emit(cur_angle)
        self.current_angle = cur_angle

    def run(self):
        if self.commands is not None:
            dirpin = 11
            steppin = 7
            enpin = 13
            GPIO.setmode(GPIO.BOARD)

            GPIO.setwarnings(False)
            GPIO.setup(dirpin, GPIO.OUT)
            GPIO.setup(steppin, GPIO.OUT)
            GPIO.setup(enpin, GPIO.OUT)

            GPIO.output(dirpin, GPIO.HIGH)
            GPIO.output(dirpin, GPIO.LOW)
            GPIO.output(enpin, GPIO.LOW)

            num_coms = len(self.commands)
            #Reset progress bar
            self.overall_comp_perc.emit(0)

            for j in range(num_coms):
                if self.threadactive:
                    self.overall_step.emit(j)
                    if self.commands[j].type == 'Rotate':
                        self.RotateCommand(j,  self.current_angle)
                    elif self.commands[j].type =='Raw':
                        self.Raw(j)
                    elif self.commands[j].type == 'SetAngle':
                        self.current_angle = self.commands[j].start
                self.overall_comp_perc.emit(((j+1)/num_coms)*100)

            self.fin_angle.emit(self.current_angle)

        if self.threadactive:
            self.fin_flag.emit(True)
        else:
            self.time_left.emit(0)
            self.com_comp_perc.emit(100)

    def stop(self):
        self.threadactive = False
        self.wait()


class Ui_Form(object):
    filename = QtCore.pyqtSignal(str)

    def setupUi(self, Form):
        Form.setObjectName("Motor Controller")
        Form.resize(426, 316)
        self.gridLayout = QtWidgets.QGridLayout(Form)
        self.gridLayout.setObjectName("gridLayout")
        self.verticalLayout = QtWidgets.QVBoxLayout()
        self.verticalLayout.setObjectName("verticalLayout")
        self.progressBar = QtWidgets.QProgressBar(Form)
        self.progressBar.setProperty("value", 0)
        self.progressBar.setObjectName("progressBar")
        self.verticalLayout.addWidget(self.progressBar)
        self.progressBar_overall = QtWidgets.QProgressBar(Form)
        self.progressBar_overall.setProperty("value", 0)
        self.progressBar_overall.setObjectName("progressBar_overall")
        self.verticalLayout.addWidget(self.progressBar_overall)
        self.gridLayout.addLayout(self.verticalLayout, 3, 0, 1, 2)
        self.stopButton = QtWidgets.QPushButton(Form)
        self.stopButton.setObjectName("stopButton")
        self.gridLayout.addWidget(self.stopButton, 6, 1, 1, 1)
        self.goButton = QtWidgets.QPushButton(Form)
        self.goButton.setObjectName("goButton")
        self.gridLayout.addWidget(self.goButton, 6, 0, 1, 1)
        self.horizontalLayout_2 = QtWidgets.QHBoxLayout()
        self.horizontalLayout_2.setObjectName("horizontalLayout_2")
        self.lcd_angle = QtWidgets.QLCDNumber(Form)
        self.lcd_angle.setDigitCount(3)
        self.lcd_angle.setObjectName("lcd_angle")
        self.horizontalLayout_2.addWidget(self.lcd_angle)
        self.lcd_speed = QtWidgets.QLCDNumber(Form)
        self.lcd_speed.setDigitCount(4)
        self.lcd_speed.setObjectName("lcd_speed")
        self.horizontalLayout_2.addWidget(self.lcd_speed)
        self.lcd_time = QtWidgets.QLCDNumber(Form)
        self.lcd_time.setDigitCount(3)
        self.lcd_time.setObjectName("lcd_time")
        self.horizontalLayout_2.addWidget(self.lcd_time)
        self.gridLayout.addLayout(self.horizontalLayout_2, 1, 0, 1, 2)
        self.verticalLayout_3 = QtWidgets.QVBoxLayout()
        self.verticalLayout_3.setObjectName("verticalLayout_3")
        self.verticalLayout_2 = QtWidgets.QVBoxLayout()
        self.verticalLayout_2.setObjectName("verticalLayout_2")
        self.horizontalLayout = QtWidgets.QHBoxLayout()
        self.horizontalLayout.setObjectName("horizontalLayout")
        self.file_label = QtWidgets.QLabel(Form)
        self.file_label.setObjectName("file_label")
        self.horizontalLayout.addWidget(self.file_label)
        self.fileButton = QtWidgets.QPushButton(Form)
        self.fileButton.setObjectName("fileButton")
        self.horizontalLayout.addWidget(self.fileButton)
        self.verticalLayout_2.addLayout(self.horizontalLayout)
        self.horizontalLayout_3 = QtWidgets.QHBoxLayout()
        self.horizontalLayout_3.setObjectName("horizontalLayout_3")
        self.step_label = QtWidgets.QLabel(Form)
        self.step_label.setObjectName("step_label")
        self.step_label.setAlignment(QtCore.Qt.AlignLeft)
        self.horizontalLayout_3.addWidget(self.step_label)
        self.action_label = QtWidgets.QLabel(Form)
        self.action_label.setObjectName("action_label")
        self.action_label.setAlignment(QtCore.Qt.AlignRight)
        self.horizontalLayout_3.addWidget(self.action_label)
        self.verticalLayout_2.addLayout(self.horizontalLayout_3)
        self.verticalLayout_3.addLayout(self.verticalLayout_2)
        self.gridLayout.addLayout(self.verticalLayout_3, 0, 0, 1, 2)
        self.goButton.setEnabled(False)
        self.stopButton.setEnabled(False)

        self.fileButton.clicked.connect(self.getfiles)
        self.worker = WorkerObject()

        self.read_file_name = ""
        self.goButton.clicked.connect(self.run_command)
        self.stopButton.clicked.connect(self.stop_button_press)
        self.worker.ang_pos.connect(self.update_LCD_angle)
        self.worker.ang_speed.connect(self.update_LCD_speed)
        self.worker.time_left.connect(self.update_LCD_time)
        self.worker.com_comp_perc.connect(self.update_prog)
        self.worker.overall_comp_perc.connect(self.update_prog_overall)
        self.worker.overall_step.connect(self.update_step_labels)
        self.worker.fin_flag.connect(self.fin_func)
        self.worker.fin_angle.connect(self.update_ang)
        self.commands = None
        self.text_commands = None
        self.current_angle = 0
        self.retranslateUi(Form)

        QtCore.QMetaObject.connectSlotsByName(Form)

    def stop_button_press(self):
        self.action_label.setText('Aborted')
        # self.worker.terminate()
        self.fileButton.setEnabled(True)
        self.goButton.setEnabled(True)
        self.stopButton.setEnabled(False)
        self.worker.stop()

    def update_ang(self,par):
        self.current_angle = par

    def fin_func(self, par):
        self.goButton.setEnabled(True)
        self.stopButton.setEnabled(False)
        self.fileButton.setEnabled(True)
        self.action_label.setText('Finished')

    def update_step_labels(self, par):
        self.step_label.setText('Step: {}/{}'.format(par+1, len(self.text_commands)))
        self.action_label.setText(self.text_commands[par])

    def run_command(self):
        self.goButton.setEnabled(False)
        self.fileButton.setEnabled(False)
        self.stopButton.setEnabled(True)
        self.worker.threadactive = True
        self.worker.commands = self.commands
        self.worker.current_angle = self.current_angle
        self.worker.start()

    def update_LCD_angle(self, par):
        self.lcd_angle.display(par)
        self.current_angle = par

    def update_prog(self, par):
        self.progressBar.setValue(par)

    def update_prog_overall(self, par):
        self.progressBar_overall.setValue(par)

    def update_LCD_speed(self, par):
        self.lcd_speed.display(par)

    def update_LCD_time(self, par):
        self.lcd_time.display(par)

    def retranslateUi(self, Form):
        _translate = QtCore.QCoreApplication.translate
        Form.setWindowTitle(_translate("Form", "Motor Controller"))
        self.stopButton.setText(_translate("Form", "Stop"))
        self.goButton.setText(_translate("Form", "Go"))
        self.file_label.setText(_translate("Form", "Choose a File"))
        self.fileButton.setText(_translate("Form", "Choose File"))
        self.step_label.setText(_translate("Form", "Step "))
        self.action_label.setText(_translate("Form", "Action"))

    def getfiles(self):
        fname = QtWidgets.QFileDialog.getOpenFileName()
        self.update_LCD_speed(0)
        if fname[0]:
            commands, lines = self.get_commands(fname[0])
        elif len(self.read_file_name) > 0:
            commands, lines = self.get_commands(self.read_file_name)

        self.commands = commands
        self.text_commands = lines


    def get_commands(self, fname):
        commands = []
        lines = []
        self.file_label.setText(fname)
        self.read_file_name = fname
        self.goButton.setEnabled(True)
        with open(fname, 'r') as rfile:

            for line in rfile:
                lines.append(line.rstrip('\n'))
                try:
                    if 'Rotate' in line:
                        com_num = (line.split('('))[1].rstrip(')\n')
                        values = com_num.split(',')
                        commands.append(MotorCommand('Rotate', float(values[0]), float(values[1]), float(values[2])))
                        self.action_label.setText('Ready')
                    if 'Raw' in line:
                        com_num = (line.split('('))[1].rstrip(')\n')
                        values = com_num.split(',')
                        commands.append(MotorCommand('Raw', float(values[0]), float(values[1]), float(values[2])))
                        self.action_label.setText('Ready')

                    if 'SetAngle' in line:
                        set_angle = float((line.split('('))[1].rstrip(')\n'))
                        commands.append(MotorCommand('SetAngle', set_angle, 0, 0))
                        self.current_angle = set_angle
                        self.update_LCD_angle(set_angle)
                except ValueError:
                    print('Error')

            self.step_label.setText('Step: 0/{}'.format(len(lines)))

        return commands, lines


if __name__ == "__main__":
    import sys

    app = QtWidgets.QApplication(sys.argv)
    Form = QtWidgets.QWidget()
    ui = Ui_Form()
    ui.setupUi(Form)
    Form.show()
    sys.exit(app.exec_())
