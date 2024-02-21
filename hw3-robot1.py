import rospy
import turtlesim.srv
import numpy as np
import json
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist

# Birinci Robot
def cb_robot1(data):
    global pose1
    global poseflag1
    pose1 = data
    poseflag1 = True
    
# İkinci Robot
def cb_robot2(data):
    global pose2
    global poseflag2
    pose2 = data
    poseflag2 = True

def hata_hesap(uzaklik):

    u1 = pose2.x - pose1.x
    u2 = pose2.y - pose1.y
    v_hata = np.sqrt((u1)**2 + (u2)**2) - uzaklik
    theta_goal = np.arctan2(u2, u1)
    u3 = theta_goal - pose1.theta
    th_hata = np.arctan2(np.sin(u3), np.cos(u3))
    return v_hata,th_hata

def PID(hata_toplam, v_hata, th_hata):
    Kp = 0.43
    Ki = 0.007
    p = Kp * v_hata 
    I = Ki * v_hata    
    V = p + I
    hata_toplam += v_hata
    Kh = 0.69
    W = Kh * th_hata
    return hata_toplam, V, W

def hareket(uzaklik, hata_toplam):

    while poseflag1 == False and poseflag2 == False:
        rospy.sleep(0.005)
    
    while True:
        v_hata, th_hata = hata_hesap(uzaklik)

        hata_toplam, V, W = PID(hata_toplam, v_hata, th_hata)
            
        vel_msg.linear.x = V
        vel_msg.angular.z = W
        vel_pub.publish(vel_msg) 
                
        loop_rate.sleep() 
        

with open('hw3.json') as f:
    veri = json.load(f)

pose1 = Pose()
pose2 = Pose()
vel_msg = Twist()
poseflag1 = False
poseflag2 = False

if __name__=='__main__':
    rospy.init_node('robot1', anonymous = True) 

    rospy.Subscriber('/turtle1/pose', Pose, cb_robot1)
    
    rospy.Subscriber('/turtle2/pose', Pose, cb_robot2) 

    vel_pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=5)
    
    loop_rate = rospy.Rate(5)
    
    rospy.wait_for_service('spawn')
    spawner = rospy.ServiceProxy("spawn", turtlesim.srv.Spawn)
    spawner(0,0,0, 'turtle1')

    hareket(veri[0]["Takip_Mesafesi"], 0)
    
    rospy.spin()
