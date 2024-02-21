import rospy
import turtlesim.srv
import numpy as np
import random
import json
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist

def callback(data):
    global pose
    global poseflag
    pose = data
    poseflag = True
   
def robot2_hareket(V, W):

    while poseflag == False:
        rospy.sleep(0.005)

    vel_msg = Twist()   

    while True:
        
        vel_msg.linear.x = V
        vel_pub.publish(vel_msg)
        rospy.sleep(1)

        yon_kontrol(W)

        loop_rate.sleep()  

def donus(W,th):

    vel_msg = Twist()
    while True:
        aci = abs(pose.theta - th)
        
        if aci > 0.1:
            vel_msg.angular.z = W
            vel_pub.publish(vel_msg)

        else:
            vel_msg.angular.z = 0
            vel_pub.publish(vel_msg)
            break

def yon_kontrol(W):

    vel_msg = Twist()

    while poseflag == False:
        rospy.sleep(0.01)

    # Sol duvar ve açı: π/2 - π
    if pose.x < 0.1 and pose.theta>0:
        th = np.pi - pose.theta
        donus(W,th)

    # Sol duvar ve açı: π - 3π/2    
    elif pose.x <0.1 and pose.theta<0 :
        th = -np.pi - pose.theta
        donus(W,th)

    # Sağ duvar ve açı: 0 - π/2
    elif pose.x >11 and pose.theta>0:
        th = np.pi - pose.theta
        donus(W,th)

    # Sağ duvar ve açı: 3π/2 - 2π
    elif pose.x > 11 and pose.theta<0:
        th = -np.pi - pose.theta
        donus(W,th)

    # Üst duvar 
    elif pose.y > 11:
        th = -pose.theta
        donus(W,th)

    # Alt duvar
    elif pose.y < 0.1:
        th = -pose.theta
        donus(W,th)


with open('hw3.json') as f:
    veri = json.load(f)

pose = Pose()
poseflag = False

if __name__ == '__main__':
    rospy.init_node('robot2', anonymous = True) 
    
    rospy.Subscriber('/turtle2/pose', Pose, callback) 

    vel_pub = rospy.Publisher('/turtle2/cmd_vel', Twist, queue_size = 5)
    
    loop_rate = rospy.Rate(5)

    # Spawn
    rospy.wait_for_service('spawn')
    spawner = rospy.ServiceProxy("spawn", turtlesim.srv.Spawn)
    spawner(random.randint(1,10),random.randint(1,10),random.uniform(-3.14,3.14),'turtle2')
    
    # Kill
    rospy.wait_for_service('kill')
    killer = rospy.ServiceProxy("kill",turtlesim.srv.Kill)
    killer('turtle1')
    
    robot2_hareket(veri[1]["V"],veri[1]["W"])

    rospy.spin()