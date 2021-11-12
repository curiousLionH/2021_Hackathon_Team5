import time
import pygame
import numpy as np
from math import sin, cos, atan2, pi, radians, degrees

from Crosswalk import Crosswalk

class Brain1:
    def __init__(self, database):
        self.database = database
        self.global_path = []
        self.trophy_x, self.trophy_y = 0, 0
        self.crossing={1:(250,150),2:(50,400),3:(350,400),4:(250,550),5:(550,400),6:(550,300),7:(700,400),8:(600,500),9:(700,600),10:(950,150)}
        self.current_pos = []
        self.respawn_points = [[50, 50], [50, 750], [950, 50], [950, 750]]
        self.startpoint = []    # 현위치, 리스폰점 4개 중 트로피랑 가장 가까운 점
        self.position = None
        self.degree = None
        self.steer_flag = 0
        self.global_path_dict = {1:[2,3], 2:[1,3,4], 3:[1,2,4,5], 4:[2,3], 5:[3,6,7], 6:[5,10], 7:[5,8,9,10], 8:[7,9], 9:[7,8], 10:[6,7,9]}


    def run(self):
        while True:
            if self.database.stop:
                break

            time.sleep(0.001)
            _ = pygame.event.get()

            
            '''
            ☆☆☆☆☆☆☆☆☆☆☆☆☆☆☆☆☆☆☆☆☆☆☆☆☆☆☆☆☆☆☆☆☆☆☆☆☆☆☆☆☆☆☆☆☆☆☆
            ☆☆☆☆☆ DO NOT CHANGE ANOTHER CODE IN 2021-Hackathon-Simulator!!! ☆☆☆☆☆
            ☆☆☆☆☆☆☆☆☆☆☆☆☆☆☆☆ ONLY CHANGE Brain.py!!!☆☆☆☆☆☆☆☆☆☆☆☆☆☆☆☆
            ☆☆☆☆☆☆☆☆☆☆☆☆☆☆☆☆☆☆☆☆☆☆☆☆☆☆☆☆☆☆☆☆☆☆☆☆☆☆☆☆☆☆☆☆☆☆☆

            1. How can I get a lidar / gps / imu data?
                Lidar : data = self.database.lidar.data
                Gps : data = self.database.car.position
                IMU : data = self.database.car.direction

            2. How can I move a car?
                self.database.control.up()
                self.database.control.down()
                self.database.control.right()
                self.database.control.left()

                OR

                self.up(num)
                self.down(num)
                self.right(num)
                self.left(num)
                ☆☆☆☆☆ num in here is number of acceleration ☆☆☆☆☆

                ☆☆☆☆☆
                In one loop,
                you can only change the acceleration up to 5 and the angle up to 8!!
                Maximum speed of car is 15 and maximum angle of car can rotate is 8!!
                ☆☆☆☆☆

            3. How can I get a car status data?
                self.database.car.direction
                self.database.car.speed

            4. How can I get a v2x data?
                self.database.v2x_data
            '''

            # Implement Your Algorithm HERE!!

            # EXAMPLE CODE1: 속도 3으로 유지하면서 오른쪽으로 회전하기

            # print(self.database.v2x_data)

            if self.database.car.position is not None:
                self.current_pos = self.database.car.position
                # print(self.current_pos)
            
                if 'Trophy' in self.database.v2x_data.keys():
                    (self.trophy_x, self.trophy_y) = self.database.v2x_data['Trophy']
                    self.first_waypoint()
                    # print(self.database.v2x_data['Trophy'])

            # self.first_waypoint()

            # print(self.crossing.keys())

            # self.steer_forward()
            # if self.database.car.speed <= 7:
            #     self.up()
            # elif self.database.car.speed > 7:
            #     self.down()
            
            
            # if self.database.lidar.data!=None:
            #     self.go(self.database.lidar.data[90],self.database.car.speed)

            if self.database.lidar.data is not None:
                
                if self.steer_flag == 0:
                    self.steer_forward(self.database.lidar.data[90], self.database.car.speed)
                elif self.steer_flag == 1:
                    self.steer_right()
                elif self.steer_flag == - 1:
                    self.steer_left()
                else:
                    self.steer_backward()
                #print(self.database.v2x_data.keys())
    
                





    def up(self, num: int = 1):
        for i in range(num):
            self.database.control.up()

    def down(self, num: int = 1):
        for i in range(num):
            self.database.control.down()

    def right(self, num: int = 1):
        for i in range(num):
            self.database.control.right()

    def left(self, num: int = 1):
        for i in range(num):
            self.database.control.left()

    def first_waypoint(self):
        
        if self.current_pos[1] >= 400:
            if self.current_pos[0] <= 350:
                # 4, 2
                list = [4, 2]
            elif self.current_pos[0] <= 700:
                #  3,5,7,8
                list = [3,5,7,8]
            else:
                # 7, 9
                list = [7,9]
        else:
            if self.current_pos[0] <= 350:
                # 1, 2
                list = [1, 2]
            elif self.current_pos[0] <= 700:
                #  3, 5, 6, 7
                list = [3,5,6,7]
            else:
                # 7, 10
                list = [7, 10]
        (p_x, p_y) = self.crossing[list[0]]
        min_dist = 100000000000000000000000000
        min_index = 0
        for i in list:
            dist =  (self.trophy_x - p_x)**2 + (self.trophy_y - p_y)**2
            if min_dist > dist:
                min_dist = dist
                min_index = i
        # print(min_index)        # 테스트용
        self.global_path.append(min_index)

 
    def global_path_planning(self):
        current_index = self.global_path[0]
        list = self.global_path_dict[current_index]
        min_dist = 100000000000000000000000000
        min_index = 0
        for i in list:
            (cross_x, cross_y) = self.crossing[i]
            dist = (self.trophy_x - cross_x)**2 + (self.trophy_y - cross_y)**2
            if min_dist > dist:
                min_dist = dist
                min_index = i
        if min_index in self.global_path:
            print(self.global_path)
            return
        self.global_path.append(min_index)   
        

    def steer_forward(self, lidar : int, speed : int):

        steering_gain = 1


        front_right = np.average(self.database.lidar.data[0:59])
        front_left = np.average(self.database.lidar.data[120:179])
        
        self.position = (front_left - front_right) / (front_right + front_left)
    
        #if abs(self.position) < 0.1:
        #     steering_gain = 1
        # elif abs(self.position) > 0.1 and abs(self.position):
        #     steering_gain = 2

        if self.position > 0:
             self.left(steering_gain)
        else:
             self.right(steering_gain)

        # if self.position > 0:
        #     self.left(steering_gain)
        # else:
        #     self.right(steering_gain)

        if self.database.lidar.data[89] < 70 and front_left < front_right:
            self.degree = self.database.car.direction
            
            self.steer_flag = 1
        elif self.database.lidar.data[89] < 70 and front_left > front_right:
            self.degree = self.database.car.direction
            self.steer_flag = -1
        elif self.database.lidar.data[89] < 70:
            self.degree = self.database.car.direction
            self.steer_flag = -1
            
            
            

        if lidar == 100:
            self.velocity_control(15)
        elif lidar < 100:
            self.velocity_control(7)
            # if speed > 5:
            #     self.down(3)

        print('forward', self.position, self.database.car.direction, steering_gain)



    def steer_backward(self):



        pass

    def steer_right(self):
        self.velocity_control(5)

        # print('right', self.degree, self.database.car.direction, self.database.lidar.data[89])
        self.right(8)
        self.right(8)
        
        self.right(8)
        self.right(8)
        self.right(8)
        self.right(8)
        self.right(8)
        self.right(8)
        if abs(self.database.car.direction - self.degree) >= 90:
            self.steer_flag = 0
               



    def steer_left(self):
        self.velocity_control(5)

        # print('left', self.degree, self.database.car.direction, self.database.lidar.data[89])
        self.left(8)
        self.left(8)
        
        self.left(8)
        self.left(8)
        self.left(8)
        self.left(8)
        self.left(8)
        self.left(8)
        if abs(self.database.car.direction - self.degree) >= 90:
            self.steer_flag = 0


    def velocity_control(self, speed):
        if self.database.car.speed >= speed:
            self.down()
        else:
            self.up()


    # def first_waypoint(self):
    #     current_pos = self.database.car.position
    #     (trophy_x, trophy_y) = self.database.v2x_data['Trophy']
    #     if current_pos[1] >= 400:
    #         if current_pos[0] <= 350:
    #             # 4, 2
    #             trophy_x - 


    def respawn(self):
            closestpoint = self.respawn_points[0]
            min_dist = (self.trophy_x - closestpoint[0])**2 + (self.trophy_y - closestpoint[1])**2
            for i in self.respawn_points:
                dist =  (self.trophy_x - i[0])**2 + (self.trophy_y - i[1])**2
                if dist <= min_dist:
                    min_dist = dist
                    closestpoint = i
            cur_dist = (self.trophy_x - self.current_pos[0])**2 + (self.trophy_y - self.current_pos[0])**2
            if cur_dist <= min_dist:
                min_dist = cur_dist
                closestpoint = self.current_pos
            self.startpoint.append(closestpoint[0])
            self.startpoint.append(closestpoint[1])



    def GPS_tracking(self, total_track):
        self.velocity_control(7)
        steering_gain = 0.01
        lateral_error_gain = 1
        heading_error_gain = 1

        if self.database.car.position is not None and self.database.car.direction is not None and self.database.car.speed is not None:
            
            p_curr = self.database.car.position
            car_yaw = radians(self.database.car.direction)

            x_local = cos(car_yaw) * (total_track[0] - p_curr[0]) - sin(car_yaw) * (total_track[1] - p_curr[1])
            y_local = sin(car_yaw) * (total_track[0] - p_curr[0]) + cos(car_yaw) * (total_track[1] - p_curr[1])

            distance = ((p_curr[0] - total_track[0]) ** 2 + (p_curr[1] - total_track[1]) ** 2) ** 0.5
            distance = distance.tolist()
            min_index = distance.index(min(distance))

            lateral_error = x_local[min_index]
            heading_error = atan2(x_local[min_index + 5] - x_local[min_index], y_local[min_index + 5] - y_local[min_index])

            wheelcmd_lateral = degrees(atan2(lateral_error_gain * lateral_error, self.database.car.speed))
            wheelcmd_heading = degrees(heading_error_gain * heading_error)
            wheelcmd = round(steering_gain * (wheelcmd_heading + wheelcmd_lateral))
            print(wheelcmd)

            if wheelcmd < 0:
                self.left(abs(wheelcmd))
            elif wheelcmd == 0:
                pass
            else:
                self.right(abs(wheelcmd))




