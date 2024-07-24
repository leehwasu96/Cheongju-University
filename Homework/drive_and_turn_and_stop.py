#!/usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
import math

class DriveAndParkingNode:
    def __init__(self):
         # 현재 작성중인 노드의 이름 정의
        rospy.init_node('drive_and_parking_node', anonymous=True)
        
        # Publisher(속도)와 Subscriber(LiDAR 데이터, Odometry 데이터) 정의
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.scan_callback)
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)
        
        self.stop_distance = 2.0      # 정지해야 할 거리 (전방 2.0m)
        self.stop_distance_2 = 0.5    # 정지해야 할 거리 (후방 0.5m)
        self.linear_speed = 0.2       # 직진 속도 (m/s)
        self.angular_speed = 0.2      # 회전 속도 (rad/s)
        self.turn_flag = False        # 전진 후, 회전을 시작하기 위한 boolean
        self.rear_flag = False        # 회전 후, 후진을 시작하기 위한 boolean
        self.current_angle = 0.0      # 현재 로봇의 각도
        self.target_angle = None      # 로봇이 회전해야 하는 목표 각도


    """
     로봇의 LiDAR 데이터를 지속적으로 불러올 수 있도록 하는 Callback 함수
     해당 Callback 함수를 통해 전,후방 LiDAR 데이터를 지속적으로 불러올 수 있음
    """
    def scan_callback(self, msg):

        # LiDAR 데이터의 중앙 인덱스 계산
        center_index = len(msg.ranges) // 2
        
         # 전방 5개의 LiDAR 데이터 추출
        front_distances = msg.ranges[:5] + msg.ranges[-5:]
        
         # 후방 5개의 LiDAR 데이터 추출
        rear_distances = msg.ranges[center_index - 5 : center_index + 6]

         # 해당 범위 내 가장 가까운 거리 확인
        closest_front_distance = min(front_distances)
        closest_rear_distance = min(rear_distances)

         # 만약 회전 flag와 후진 flag가 모두 False인 경우, 전진 또는 정지하도록 코드 구성
        if (self.turn_flag == False) and (self.rear_flag == False):

            if closest_front_distance < self.stop_distance:
                  # 거리가 설정값보다 짧으면 정지 명령을 보냄
                rospy.loginfo("Obstacle detected in front. Stopping the robot.")
                self.stop_robot()
                
                  # 로봇을 회전할 수 있도록 하는 boolean을 True로 변경 
                self.turn_flag = True
                rospy.sleep(0.5)

            else:
                   # 거리가 설정값보다 길면 직진 명령을 보냄
                rospy.loginfo("Closest distance in front 10 degrees: {:.2f} m".format(closest_front_distance))
                self.move_forward()
                
         # 만약 회전 flag가 True인 경우, 제자리 180도 회전을 할 수 있도록 코드 구성        
        elif (self.turn_flag == True) and (self.rear_flag == False):
            self.target_angle = 3.14 # 로봇이 회전해야 하는 목표 각도 설정 (단위: radian)
              # 정지 후, 180도 회전 명령을 보냄  
            rospy.loginfo("Robot is rotating...")
            
            while not rospy.is_shutdown():
                angle_diff = self.target_angle - self.current_angle

                  # 회전 오차를 고려하여 로봇이 180도 회전한 경우, 회전을 정지 !
                if abs(angle_diff) < 0.02:  # 오차 허용 범위 내 도달 시 정지 (단위: radian)
                       # 로봇이 180도 회전을 마치면 잠시 정지시킨 뒤, 후진하기 위한 boolean 값을 True로 변경
                    self.rear_flag = True
                    self.stop_robot()
                    break
                
                self.turn_robot()
                rospy.sleep(0.1)
        
         # 만약 로봇이 180도 제자리 회전을 마쳐 후진 flag가 True가 된 경우, 후진할 수 있도록 코드 구성
        elif (self.rear_flag):
              # 180도 회전을 마친 후, 로봇이 후진하도록 속도 명령을 보냄
            if closest_rear_distance > self.stop_distance_2:
                rospy.loginfo("Closest distance in rear 10 degrees: {:.2f} m".format(closest_rear_distance))
                self.move_back()
            else:
                self.stop_robot()
        
        # Error 방지
        else:
            rospy.loginfo("Code Error")

    """
     로봇의 Odometry 데이터를 지속적으로 불러올 수 있도록 하는 Callback 함수
     해당 Callback 함수를 통해 로봇의 주행기록계(위치 및 방향) 정보를 지속적으로 불러올 수 있으며,
     이를 통해 로봇의 현재 각도 정보를 알 수 있음
    """
    # 로봇의 주행기록계(위치 및 방향)를 통해 로봇의 현재 각도 정보를 받아오기
    def odom_callback(self, msg):
        orientation_q = msg.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        _, _, yaw = euler_from_quaternion(orientation_list)
        self.current_angle = yaw
        #print(math.degrees(self.current_angle)) # 로봇의 각도를 radian에서 degree로 변경 후, 터미널에 출력 (단위: degree) 


    """
    로봇의 실제 전진/정지/회전/후진 명령을 생성하도록 하는 메서드 정의
    """
    # 로봇이 정지하도록 속도 명령 생성
    def stop_robot(self):
        # 로봇을 정지시키는 메시지 생성
        stop_msg = Twist()
        stop_msg.linear.x = 0.0
        stop_msg.angular.z = 0.0
        self.cmd_vel_pub.publish(stop_msg)

    # 로봇이 전진하도록 속도 명령 생성
    def move_forward(self):
        # 로봇을 직진시키는 메시지 생성
        move_msg = Twist()
        move_msg.linear.x = self.linear_speed
        move_msg.angular.z = 0.0
        self.cmd_vel_pub.publish(move_msg)
        
    # 로봇이 회전하도록 속도 명령 생성
    def turn_robot(self):
        move_msg = Twist()
        move_msg.linear.x = 0.0
        move_msg.angular.z = self.angular_speed
        self.cmd_vel_pub.publish(move_msg)
    
    # 로봇이 후진하도록 속도 명령 생성
    def move_back(self):
        move_msg = Twist()
        move_msg.linear.x = -self.linear_speed
        move_msg.angular.z = 0.0
        self.cmd_vel_pub.publish(move_msg)

    # 코드를 지속적으로 반복
    def run(self):
        rospy.spin()

if __name__ == '__main__':
    node = DriveAndParkingNode()
    # 데이터를 안전하게 받아올 수 있게 잠시 코드를 정지
    rospy.sleep(1)
    node.run()

