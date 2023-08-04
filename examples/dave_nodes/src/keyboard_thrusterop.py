#!/usr/bin/env python
'''
Node to directly control the thrusters through keyboard
'''
import os

import rospy
import sys
import  tty, termios
from uuv_gazebo_ros_plugins_msgs.msg import FloatStamped

class ThrusterOp:
    def __init__(self, namespace='smilodon'):
        # Keyboards==>publishers
        self.keyboard2thrust = dict()
        self.keyboard2thrust[0] = rospy.Publisher('/%s/thrusters/%d/input'%(namespace,0), FloatStamped, queue_size=1)
        self.keyboard2thrust[1] = rospy.Publisher('/%s/thrusters/%d/input'%(namespace,1), FloatStamped, queue_size=1)
        self.keyboard2thrust[2] = rospy.Publisher('/%s/thrusters/%d/input'%(namespace,2), FloatStamped, queue_size=1)
        self.keyboard2thrust[3] = rospy.Publisher('/%s/thrusters/%d/input'%(namespace,3), FloatStamped, queue_size=1)
        self.keyboard2thrust[4] = rospy.Publisher('/%s/thrusters/%d/input'%(namespace,4), FloatStamped, queue_size=1)
        self.keyboard2thrust[5] = rospy.Publisher('/%s/thrusters/%d/input'%(namespace,5), FloatStamped, queue_size=1)
        # Init the msg
        self.msg = FloatStamped()
        self.msg.data = 500  #the number of data affects the velocity
        # Init the publish frquency
        self.rate = rospy.Rate(10)   

    # 读取按键       
    def keyboradloop(self):
        while not rospy.is_shutdown():
            fd = sys.stdin.fileno()
            old_settings = termios.tcgetattr(fd)
            #不产生回显效果
            old_settings[3] = old_settings[3] & ~termios.ICANON & ~termios.ECHO
            try :
                tty.setraw(fd)
                ch = sys.stdin.read(1)
            finally :
                termios.tcsetattr(fd,termios.TCSADRAIN, old_settings)
    
            if ch == "q":
                self.keyboard2thrust[0].publish(self.msg)
            elif ch == "w":
                self.keyboard2thrust[1].publish(self.msg)
            elif ch == "e":
                self.keyboard2thrust[2].publish(self.msg)
            elif ch == "r":
                self.keyboard2thrust[3].publish(self.msg)
            elif ch == "t":
                self.keyboard2thrust[4].publish(self.msg)
            elif ch == "y":
                self.keyboard2thrust[5].publish(self.msg)
            elif ch == "m":
                self.msg.data = 0
            elif ch == "2":
                self.msg.data-=300
            self.rate.sleep()



if __name__ == '__main__':
    # Start the node
    node_name = os.path.splitext(os.path.basename(__file__))[0]
    rospy.init_node(node_name)
    rospy.loginfo('Starting [%s] node' % node_name)

    # Get params
    ns = 'smilodon'
    if rospy.has_param('~namespace'):
        ns = rospy.get_param('~namespace')

    teleop = ThrusterOp(namespace = ns)
    try:
        teleop.keyboradloop()
    except rospy.ROSInterruptException:
        rospy.loginfo('Shutting down [%s] node' % node_name)
    
    


