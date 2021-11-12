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
        self.prev_trophy_x, self.prev_trophy_y = 0, 0
        self.crossing={1:(250,150),2:(50,400),3:(350,400),4:(250,550),5:(550,400),6:(550,300),7:(700,400),8:(600,500),9:(700,600),10:(950,150), 11:(0,0)}
        self.previous_pos = [0, 0]
        self.current_pos = [0, 0]        
        self.car_direction = 0
        self.car_speed = 0
        self.lidar = []
        self.respawn_points = [[50, 50], [50, 750], [950, 50], [950, 750]]
        self.startpoint = []    # 현위치, 리스폰점 4개 중 트로피랑 가장 가까운 점
        self.position = 0
        self.degree = 0
        self.steer_flag = 0
        self.global_path_dict = {1:[2,3], 2:[1,3,4], 3:[1,2,4,5], 4:[2,3], 5:[3,6,7], 6:[5,10], 7:[5,8,9,10], 8:[7,9], 9:[7,8], 10:[6,7]}
        self.curr_dest_index = 0
        self.curr_dest_x, self.curr_dest_y = 0, 0
        self.crosswalk_point = []
        self.global_path_point = []

        self.onetwo = [(250, 250), (50, 250)]
        self.onethree = [(250, 50), (450, 50), (450, 150), (350, 150)]
        self.twothree = []
        self.twofour = [(50, 550)]
        self.threefour = [(350, 750), (250, 750)]
        self.threefive = []
        self.fivesix = []
        self.fiveseven = []
        self.sixten = [(700, 300), (700, 50), (950, 50)]
        self.seveneight = [(700, 500)]
        self.sevennine = []
        self.seventen = [(810, 400), (810, 150)]
        self.eightnine = [(700, 500)]
        

    def run(self):
        while True:
            if self.database.stop:
                break
            
            time.sleep(0.001)
            _ = pygame.event.get()

            while(True):
                if self.database.car.position is None:
                    print("while문")
                    continue
                if self.database.car.speed is None:
                    print("while문")
                    continue
                if 'Trophy' not in self.database.v2x_data.keys():
                    print("while문")
                    continue
                if self.database.car.direction is None:
                    print("while문")
                    continue
                if self.database.lidar.data is None:
                    print("while문")
                    continue

                self.current_pos = self.database.car.position
                self.car_speed = self.database.car.speed
                (self.trophy_x, self.trophy_y) = self.database.v2x_data['Trophy']
                self.car_direction = self.database.car.direction
                self.lidar = self.database.lidar.data
                break


            if self.respawn_car_flag() or self.respawn_trophy_flag():
                self.crossing[11] = (self.trophy_x, self.trophy_y)
                self.global_path_point = []      
                self.first_waypoint()
                self.global_path_planning()
                
                # for key in self.global_path:
                #     self.global_path_point.append(self.crossing[key])
                print(self.global_path_point)
                

                # print(self.database.v2x_data['Trophy'])

            
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

            self.crosswalk()
            if len(self.crosswalk_point) > 0:
                if self.crosswalk_reach_flag():
                    print("신호등")
                    self.velocity_control(0, 15)

                   
                   


            # self.curr_dest_index = 0    # 고정
            self.curr_dest_x, self.curr_dest_y = self.crossing[self.global_path[0]]
            angle_error = np.arctan2((self.curr_dest_y - self.current_pos[1]),(self.curr_dest_x - self.current_pos[0])) * 180/pi
            angle_error = angle_error - ((-1)*(self.car_direction) + 180)

            # self.steer_flag = self.steering_flag(angle_error)           


            self.calculation_car_yaw()

            # print(self.steer_flag)
            if self.steer_flag == 0:
                self.steer_forward(self.lidar[90], self.car_speed)
            elif self.steer_flag == 1:
                self.steer_right()
            elif self.steer_flag == - 1:
                self.steer_left()
            else:
                self.steer_backward()
            #print(self.database.v2x_data.keys())
            
            if self.reach():
                self.global_path.remove(self.global_path[0])

            self.previous_pos = self.current_pos
            self.prev_trophy_x, self.prev_trophy_y = self.trophy_x, self.trophy_y
            # self.crosswalk()





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

    def steering_flag(self, angle_error):
        if angle_error > 0:
            return 1
        elif angle_error < 0:
            return -1
        else:
            return 0


    def first_waypoint(self):
        self.global_path = []
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
        print("first way point", min_index)        # 테스트용
        self.global_path.append(min_index)
        self.global_path_point.append(self.crossing[min_index])

 
    def global_path_planning(self):
        while True:
            current_index = self.global_path[len(self.global_path)-1]
            list = self.global_path_dict[current_index]
            min_dist = 100000000000000000000000000
            min_index = 0
            for i in list:
                (cross_x, cross_y) = self.crossing[i]
                dist = (self.trophy_x - cross_x)**2 + (self.trophy_y - cross_y)**2
                if min_dist > dist:
                    min_dist = dist
                    min_index = i
                    print("global path planning", min_dist, min_index)

            prev_index = self.global_path[len(self.global_path)-1]
            (prev_cross_x, prev_cross_y) = self.crossing[prev_index]

            if min_index in self.global_path:
                self.global_path.append(11)
                self.global_path_point.append(self.crossing[11])
                print("global path", self.global_path)
                break
            elif min_dist > (self.trophy_x - prev_cross_x)**2 + (self.trophy_y - prev_cross_y)**2:
                self.global_path.append(11)
                self.global_path_point.append(self.crossing[11])
                print("global path", self.global_path)
                break
            else:
                temp = self.path_maker(prev_index, min_index)
                if temp is not None:
                    for i in temp:
                        self.global_path_point.append(i)
                self.global_path.append(min_index)
                self.global_path_point.append(self.crossing[min_index])

    def path_maker(self, prev, curr):
        if prev == 1:
            if curr == 2:
                return self.onetwo
            if curr == 3:
                return self.onethree
        elif prev == 2:
            if curr == 1:
                return self.onetwo.reverse
            if curr == 3:
                return None
            if curr == 4:
                return self.twofour
        elif prev == 3:
            if curr == 1:
                return self.onethree.reverse
            if curr == 2:
                return None
            if curr == 4:
                return self.threefour
            if curr == 5:
                return None
        elif prev == 4:
            if curr == 2:
                return self.twofour.reverse
            if curr == 3:
                return self.threefour.reverse
        elif prev == 5:
            return None
        elif prev == 6:
            if curr == 5:
                return None
            if curr == 10:
                return self.sixten
        elif prev == 7:
            if curr == 5:
                return None
            if curr == 8:
                return self.seveneight
            if curr == 9:
                return None
            if curr == 10:
                return self.seventen
        elif prev == 8:
            if curr == 7:
                return self.seveneight.reverse
            if curr == 9:
                return self.eightnine
        elif prev == 9:
            if curr == 7:
                return None
            if curr == 8:
                return self.eightnine.reverse
        elif prev == 10:
            if curr == 6:
                return self.sixten.reverse
            if curr == 7:
                return self.seventen.reverse




        



    def reach(self):
        dist = (self.curr_dest_x - self.current_pos[0])**2 + (self.curr_dest_y - self.current_pos[1])**2
        if dist < 20:       # 20은 실험값
            return True
        else:
            return False

    def respawn_car_flag(self):
        dist = (self.previous_pos[0] - self.current_pos[0])**2 + (self.previous_pos[1] - self.current_pos[1])**2
        if dist > 6000:   # 600000은 실험값
            return True
        else:
            return False

    def respawn_trophy_flag(self):
        if self.prev_trophy_x != self.trophy_x or self.prev_trophy_y != self.trophy_y:
            return True
        else:
            return False

    def crosswalk(self):
        self.crosswalk_point = []
        crosswalk = list(self.database.v2x_data.values())
        for i in range(1, len(crosswalk)):
            if crosswalk[i][1] == 'red':
                # print("crosswalk print", crosswalk[i][2])
                self.crosswalk_point.append(crosswalk[i][2])
        # print("end")

    def crosswalk_reach_flag(self):
        for i in self.crosswalk_point:
            (crosswalk_x, crosswalk_y) = i
        dist = (crosswalk_x - self.current_pos[0])**2 + (crosswalk_y - self.current_pos[1])**2  
        if dist < 5000:     # 3500은 실험값
            return True
        else:
            return False 


    def calculation_car_yaw(self):
        if self.database.car.direction is not None:
            if self.database.car.direction <= 90:
                self.car_yaw = self.database.car.direction
            elif self.database.car.direction > 90 and self.database.car.direction <= 180:
                self.car_yaw = self.database.car.direction - 90
            elif self.database.car.direction > 180 and self.database.car.direction <= 270:
                self.car_yaw = self.database.car.direction - 180
            else:
                self.car_yaw = self.database.car.direction - 270


    def steer_forward(self, lidar : int, speed : int):

        steering_gain = 2

        front_right = np.average(self.lidar[0:59])
        front_left = np.average(self.lidar[120:179])
        
        self.position = (front_left - front_right) / (front_right + front_left)
        self.degree = self.car_direction


        # print('forward', self.position, self.car_direction, self.car_yaw % 90)
        if lidar == 100:
            self.velocity_control(15)
        elif lidar < 100:
            self.velocity_control(6)

        if self.car_yaw % 90 == 0: 
            # print('그냥 직진')
            pass
        else:

            if self.position > 0 and self.car_yaw % 90 < 45:
                self.right(steering_gain)
            elif self.position > 0 and self.car_yaw % 90 > 45:
                self.left(steering_gain)
            elif self.position < 0 and self.car_yaw % 90 < 45:
                self.right(steering_gain)
            elif self.position < 0 and self.car_yaw % 90 > 45:
                self.left(steering_gain)
            


        if self.lidar[89] < 70 and front_left < front_right:
            self.steer_flag = 1
        elif self.lidar[89] < 70 and front_left > front_right:
            self.steer_flag = -1
 






    def steer_backward(self):



        pass

    def steer_right(self):
        self.velocity_control(6)
        # print('right', self.degree, self.car_direction, self.lidar[89])
        for i in range(8):
            self.right(8)

        if abs(self.car_direction - self.degree) >= 90:
            self.steer_flag = 0
               



    def steer_left(self):
        self.velocity_control(6)
        # print('left', self.degree, self.car_direction, self.lidar[89])
        for i in range(8):
            self.left(8)


        if abs(self.car_direction - self.degree) >= 90:
            self.steer_flag = 0



    def velocity_control(self, speed, down=10):
        if self.database.car.speed >= speed:
            self.down(down)
        else:
            self.up()

    def respawnpoint(self):
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
            
    def respawn(self):
        cur_dist = (self.trophy_x - self.current_pos[0])**2 + (self.trophy_y - self.current_pos[0])**2 
        min_dist = (self.trophy_x - self.startpoint[0])**2 + (self.trophy_y - self.startpoint[0])**2
        if cur_dist > min_dist:
            self.steer_left()



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




