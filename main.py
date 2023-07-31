#!/usr/bin/env python


import rospy
import tf
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from time import sleep


class myRobot():


   def __init__(self):
       self.euler = 0
       # Publisher head
       self.head_pub = rospy.Publisher('/head_controller/command', JointTrajectory, queue_size=10)
       # Subscriber odometria
       self.sub_odo = rospy.Subscriber('/mobile_base_controller/odom', Odometry, self.callback_odometria, queue_size=1)
       # Subscriber laser
       self.laser_sub =rospy.Subscriber('/scan_raw', LaserScan, self.callback_laser)
       # Publisher base
       self.pub_vel = rospy.Publisher('/nav_vel', Twist, queue_size=1)
       self.v = Twist()
       self.rate = rospy.Rate(10)
       self.centro = 10
       self.direita = 0
       self.esquerda = 0
      
       self.state = 'inicio'


   def mover_cabeca(self, sentido):
       if sentido == 'esquerda':
           x = 2
           y = 0
           self.state = 'virar_esquerda'
       if sentido == 'direita':
           x = -2
           y = 0
       if sentido == 'centro':
           x = 0
           y = 0
       msg = JointTrajectory()
       msg.joint_names = ['head_1_joint','head_2_joint']
       point = JointTrajectoryPoint()
       point.positions = [x, y]
       point.velocities = [0.0,0.0]
       point.time_from_start = rospy.Duration(1.0)
       msg.points = [point]
       self.head_pub.publish(msg)
       self.rate.sleep()


   def callback_odometria(self, msg):
       quaternion = (
           msg.pose.pose.orientation.x,
           msg.pose.pose.orientation.y,
           msg.pose.pose.orientation.z,
           msg.pose.pose.orientation.w)
       self.euler = tf.transformations.euler_from_quaternion(quaternion)[2]


   def callback_laser(self, msg):
       self.esquerda = msg.ranges[len(msg.ranges)-61]
       self.direita = msg.ranges[61]
       self.centro = msg.ranges[len(msg.ranges)//2]


   def moveStraight(self):
       self.v.linear.x = 0.3
       self.pub_vel.publish(self.v)


   def stop(self):
       self.v.linear.x = 0
       self.pub_vel.publish(self.v)


   def turn(self, sens):
       if sens == 'direita':
           orientacao = self.euler - 1.57
       elif sens == 'esquerda':
           orientacao = self.euler + 1.57
       else:
           orientacao = 0.0


       self.state = 'girando'
       while abs(self.euler - orientacao) > 0.1:
           self.v.angular.z = -0.5 if sens == 'direita' else 0.5
           self.pub_vel.publish(self.v)
           self.rate.sleep()


       self.v.angular.z = 0.0
       self.pub_vel.publish(self.v)
       self.state = 'parar_giro'


   def girar_para_zero_graus(self):
       orientacao = 0.0
       print(self.euler-orientacao)
       while abs(self.euler - orientacao) > 0.01:
           if self.euler > orientacao:
               print('aa')
               self.v.angular.z = -0.1
               self.pub_vel.publish(self.v)
               self.rate.sleep()
           else:
               print('bb')
               self.v.angular.z = 0.1
               self.pub_vel.publish(self.v)
               self.rate.sleep()
          


       self.v.angular.z = 0.0
       self.pub_vel.publish(self.v)
       self.state = 'parar_giro'
#---------------------------------------------------------
   def decision(self):
       if self.state == 'inicio':
           sleep(2)
           if self.euler != 0.000:
               self.girar_para_zero_graus()
           self.state = 'siga'
       if self.centro > 0.8 and self.state == 'siga':
           self.moveStraight()
       if self.centro <= 0.8 and self.state == 'siga':
           self.stop()
           self.state = 'pare'
#---------------------------------------------------------
       if self.state == 'pare':                  
           if self.esquerda > 1.5:
               self.state = 'virar_esquerda'
           if self.direita > 1.5:
               self.state = 'virar_direita'
           if self.esquerda > 1.5 and self.direita > 1.5:
               self.state = 'ligar_camera'
#---------------------------------------------------------
       if self.state == 'virar_esquerda':
           self.turn('esquerda')
       if self.state == 'virar_direita':
           self.turn('direita')
#---------------------------------------------------------
       if self.state == 'parar_giro':
           self.state = 'siga'
#---------------------------------------------------------
       if self.state == 'ligar_camera':
           self.mover_cabeca('esquerda')
           sleep(3)
           self.mover_cabeca('direita')
           sleep(3)
           self.mover_cabeca('centro')
           sleep(3)








if __name__ == '__main__':
   rospy.init_node('nodeName')
   tiago = myRobot()
   while not rospy.is_shutdown():
       tiago.decision()
       tiago.rate.sleep()




