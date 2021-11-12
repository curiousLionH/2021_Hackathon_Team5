import time
import pygame


class Brain1:
    def __init__(self, database):
        self.database = database
        self.global_path = []
        self.trophy_x, self.trophy_y = 0, 0
        self.crossing={1:(250,150),2:(50,400),3:(350,400),4:(250,550),5:(550,400),6:(550,300),7:(700,400),8:(600,500),9:(700,600),10:(950,150)}
        self.current_pos = []

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
            self.right()

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

            if self.database.car.speed <= 2:
                self.up()
            elif self.database.car.speed > 3:
                self.down()

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
        print(min_index)        # 테스트용
        self.global_path.append(min_index)

        

