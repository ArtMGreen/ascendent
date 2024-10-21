import threading
from threading import Timer
from math import atan, sin, pi, cos, acos
from subprocess import call
import time

import xr_config as cfg
import xr_gpio as gpio
from xr_servo import Servo
from xr_oled import Oled
from xr_motor import RobotDirection
from xr_ultrasonic import Ultrasonic
from xr_car_light import Car_light
from xr_camera import Camera


from socket_class import SocketClass


class Bobot:
    def __init__(self, addr, port):
        self.motor = Servo()
        self.go = RobotDirection()
        self.ult = Ultrasonic()
        self.lights = Car_light()
        self.socketclass = SocketClass(addr, port)
        self.oled = Oled()

        self.camera = Camera()
        self.cam_t = threading.Thread(target=self.camera.run, args=())

        self.SPEED = 0
        self.instruction = []


    def change_speed(self, new_speed):
        self.SPEED = new_speed
        self.go.set_speed(1, self.SPEED-1)
        self.go.set_speed(2, self.SPEED)
        cfg.LEFT_SPEED = self.SPEED - 1
        cfg.RIGHT_SPEED = self.SPEED
        self.go.save_speed()
        self.go.motor_init()


    def get_instruction(self):
        self.socketclass.send("what")
        self.instruction = self.socketclass.receive().split()



    def find_object(self):
        self.socketclass.send("find")
        _, x, y, h = self.socketclass.receive().split()
        return float(x), float(y), float(h)


    def send_fail(self):
        self.socketclass.send("fail")
        self.instruction = self.socketclass.receive()

    def starting(self, color):
        self.socketclass.send(f"start | {color}")
        self.socketclass.receive()

    def check_way(self, direction="forward"):
        if direction == "right":
            self.motor.set(7, 0)
        elif direction == "left":
            self.motor.set(7, 180)
        dist = self.ult.get_distance()
        self.motor.set(7, 90)
        if dist == 0 or dist > 10:
            return True
        return False


    def check_ir(self): # Правый ИК-датчик
        left_sensor = not gpio.digital_read(gpio.IRF_L)  # Левый ИК-датчик
        right_sensor = not gpio.digital_read(gpio.IRF_R)  # Правый ИК-датчик
        # Центральный ИК-датчик
        if left_sensor:
            return "right"
        if right_sensor:
            return "left"
        return "ok"


    def move(self, direction, distance):
        distance = abs(distance)

        if direction == "forward":
            times = distance / self.SPEED
            start = time.time()
            self.go.forward()
            while self.check_way() and time.time() - start < times:
                check = self.check_ir()
                if check == "ok":
                    time.sleep(0.001)
                else:
                    self.go.stop()
                    if check == "right":
                        self.go.right()
                    else:
                        self.go.left()
                    time.sleep(5 / 180 * (pi*13)/ self.SPEED)
                    self.go.forward()
            self.go.stop()
            if not self.check_way():
                return "Wall"

        if direction == "back":
            times = distance / self.SPEED
            self.go.back()
            time.sleep(times)
            self.go.stop()

        if direction == "right": #83, 157
            if self.check_way("right"):
                #distance += 5
                if distance == 90:
                    distance = 83
                if distance == 180:
                    distance = 157

                #times = distance / 180 * (pi*13)/ self.SPEED
                times = distance/self.SPEED
                self.go.right()
                time.sleep(times)
                self.go.stop()
            else:
                return "Wall"

        if direction == "left": #84, 156
            if self.check_way("left"):
                #distance += 10
                if distance == 90:
                    distance = 84
                if distance == 180:
                    distance = 156

                #times = distance / 180 * (pi * 13) / self.SPEED
                times = distance/self.SPEED
                self.go.left()
                time.sleep(times)
                self.go.stop()
            else:
                return "Wall"

        return "Good"


    def act(self, x, y, h, action, thing="ball"):
        x0, y0, h0 = 8.5, 15.5, 5.5
        dx, dh, dy = x - x0, h - h0, y-y0
        d = (dh**2 + dy**2)**0.5

        alpha = int(90 - abs(atan(dy/dx)) * 360 / (2*pi)//5*5) #angle between manipulator and goal
        tetha = int(abs(acos((392 - d**2) / 392)) * 360 / (
                    2 * pi) // 5 * 5)  # angle for second servo (теорема косинусов)
        betha = int(180 - abs(acos(d / 28)) * 360 / (
                    2 * pi) // 5 * 5 + abs(atan(dh / dy))*360/(2*pi)//5*5)

        if alpha != 0 and x < x0:
            self.move("right", alpha )
        elif alpha != 0 and x > x0:
            self.move("left", alpha)

        x, y, h = self.find_object()
        while (y < 19 or y > 39):
            if y < 19:
                self.move("back", 5)
            elif y > 39:
                self.move("forward", 5)
            x, y, h = self.find_object()

        if action == "take":
            self.motor.set(1, 180)
            self.motor.set(2, 90)
            self.motor.set(3, 90)
            self.motor.set(4, 30)
            time.sleep(1)
            self.motor.set(2, tetha)
            self.motor.set(1, betha)
            time.sleep(1)
            self.motor.set(4, 90 if thing == "ball" else 75)

        elif action == "throw":
            self.motor.set(1, 180)
            self.motor.set(2, 90)
            self.motor.set(3, 90)
            time.sleep(0.5)
            self.motor.set(1, betha)
            self.motor.set(2, tetha)
            time.sleep(0.5)
            self.motor.set(4, 45)

        elif action == "press":
            self.motor.set(1, 180)
            self.motor.set(3, 180)
            self.motor.set(4, 90)
            time.sleep(0.5)
            self.motor.set(2, tetha)
            self.motor.set(1, betha)
            time.sleep(0.5)

        time.sleep(1)

        self.motor.set(1, 180)
        self.motor.set(2, 90)
        self.motor.set(3, 90)

        time.sleep(0.5)

        if alpha != 0 and x < x0:
            self.move("left", alpha)
        elif alpha != 0:
            self.move("right", alpha)


    def maze(self):
        flag = True
        self.motor.set(7, 90)
        df = self.ult.get_distance()
        k = 0
        if df < 32:
            self.move("left", 90)
            k+=1
            self.move("forward", 0.75)
            self.move("right", 90)
            self.move("forward", 0.75)
            for i in range(3):
                k+=1
                self.motor.set(7, 0)
                df = self.ult.get_distance()
                if df > 32:
                    self.move("right", 90)
                    self.motor.set(7, 90)
                    break
                else:
                    if i == 2:
                        return "Fail"
                    self.move("forward", 0.75)
                    self.move("right", 90)
                    self.move("forward", 0.75)


        self.move("forward", 0.32)
        x, y, h = self.find_object()
        self.act(x, y, h, "take")
        self.move("back", 0.32)
        if k == 1:
            self.move("left", 90)
        elif k == 2:
            self.move("left", 180)
        elif k == 3:
            self.move("right", 180)
        return "Good"



    def do_action(self, instruction):
        if instruction[0] == "move":
            res = self.move(instruction[1], int(instruction[2]))
            if res == "Wall":
                return "Fail"

        elif instruction[0] == "take":
            x, y, h = self.find_object()
            #self.act(x, y, h, "take", thing=thing)


        elif instruction[0] == "throw":
            x, y, h = self.find_object()
            self.act(x, y, h, "throw")


        elif instruction[0] == "press":
            x, y, h = self.find_object()
            self.act(x, y, h, "press")


        elif instruction[0] == "maze":
            r = self.maze()
            if r == "Fail":
               return "Fail"





    def do_pose(self, color):

        #self.lights.init_led(color)
        self.lights.set_ledgroup(1, 8, color)
        self.lights.set_ledgroup(2, 8, color)
        """self.motor.set(1, 120)
        self.motor.set(2, 120)
        self.motor.set(3, 0)
        self.motor.set(4, 45)
        time.sleep(0.5)
        self.motor.set(1, 180)
        self.motor.set(2, 180)
        self.motor.set(4, 75)
        time.sleep(0.5)
        self.motor.set(2, 120)
        time.sleep(0.5)
        self.motor.set(2, 180)
        time.sleep(0.5)
        self.motor.set(2, 120)
        time.sleep(0.5)
        self.motor.set(2, 180)
        time.sleep(0.05)"""
        self.motor.set(1, 180)
        self.motor.set(2, 90)
        self.motor.set(3, 90)
        self.motor.set(4, 90)


    def process(self):
        color = input("What color do i use?")
        start = time.time()
        self.oled.disp_default()
        self.cam_t.start()
        self.change_speed(90)
        self.do_pose(color)
        self.starting(color)
        self.get_instruction()
        input("Go?")
        r = "Start"
        while time.time() - start < 115:
            if r != "Fail" and r != "Start":
                self.get_instruction()

            r = self.do_action(self.instruction)
            if r == "Fail":
                self.send_fail()

        self.do_pose(color)


ADDR, PORT = "192.168.212.254", 9998
bobotik = Bobot(ADDR, PORT)
bobotik.process()
"""bobotik.change_speed(100)
while True:
    s = input()
    if s == "end":
        break
    eval(s)"""
