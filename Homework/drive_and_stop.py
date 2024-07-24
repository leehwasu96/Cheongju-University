#!/usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist


class DriveAndStopNode:
    def __init__(self):
         # 현재 작성중인 노드의 이름 정의
        rospy.init_node('drive_and_stop_node', anonymous=True)
        
        # Publisher(속도)와 Subscriber(LiDAR 데이터, Odometry 데이터) 정의
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.scan_callback)
        
        self.stop_distance = 2.0      # 정지해야 할 거리 (전방 2.0m)
        self.stop_distance_2 = 0.5    # 정지해야 할 거리 (후방 0.5m)
        self.linear_speed = 0.2       # 직진 속도 (m/s)

    """
     로봇의 LiDAR 데이터를 지속적으로 불러올 수 있도록 하는 Callback 함수
     해당 Callback 함수를 통해 전방 LiDAR 데이터를 지속적으로 불러올 수 있음
    """
    def scan_callback(self, msg):
        
         # 전방 10개의 LiDAR 데이터 추출
        front_distances = msg.ranges[:5] + msg.ranges[-5:]
        
         # 해당 범위 내 가장 가까운 거리 확인
        closest_front_distance = min(front_distances)

        if closest_front_distance < self.stop_distance:
            # 거리가 설정값보다 짧으면 정지 명령을 보냄
            rospy.loginfo("Obstacle detected in front. Stopping the robot.")
            self.stop_robot()
            rospy.sleep(0.5)

        else:
            # 거리가 설정값보다 길면 직진 명령을 보냄
            rospy.loginfo("Closest distance in front 10 degrees: {:.2f} m".format(closest_front_distance))
            self.move_forward()
                

    """
    로봇의 실제 전진/정지 명령을 생성하도록 하는 메서드 정의
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
        
    # 코드를 지속적으로 반복
    def run(self):
        rospy.spin()


if __name__ == '__main__':
    node = DriveAndStopNode()
    # 데이터를 안전하게 받아올 수 있게 잠시 코드를 정지
    rospy.sleep(1)
    node.run()
