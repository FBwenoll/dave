#!/usr/bin/env python
'''
Node to directly control the thrusters through keyboard
'''
import os

import rospy
import sys
import  tty, termios
from uuv_gazebo_ros_plugins_msgs.msg import FloatStamped

#msgs are dict() in which the params are FloatStamped()
class Thrusters:
    def __init__(self,namespace="virgil",thruster_number="7"):
        self.thruster_number = thruster_number
        # Init the msg& publisher
        self.thruster_pub = dict()
        for i in range(thruster_number+1):
            self.thruster_pub[i] = rospy.Publisher('/%s/thrusters/%d/input'%(namespace,i), FloatStamped, queue_size=1)

        # public params
        # the velocity of i th thruster
        self.thruster = dict()
        for i in range(0,self.thruster_number+1):
            self.thruster[i] = FloatStamped()
            self.thruster[i].data = 0

    def control(self):
        #check out whether the type of msgs is correct 
        if type(self.thruster) != dict and self.thruster[0] != FloatStamped:
            rospy.ERROR("the tpye of msgs is NOT dict() or the key of msgs is not FloatStamped().\n")
        else:
            for i in range(self.thruster_number+1):
                self.thruster_pub[i].publish(self.thruster[i])





class ThrusterOp:
    def __init__(self, namespace='smilodon'):
        self.ts = Thrusters(namespace=namespace,thruster_number=7)
        # Init the publish frquency
          
       
    def keyboradloop(self):
        rate = rospy.Rate(10) 
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
    
            if ch == "w":
                self.ts.thruster[0].data = 500
            elif ch == "s":
                self.ts.thruster[1].data = -500
            if ch == "a":
                self.ts.thruster[2].data = 300
            elif ch == "d":
                self.ts.thruster[3].data = 300
            if ch == "e":
                self.ts.thruster[4].data+=300
            elif ch == "d":
                self.ts.thruster[5].data-=300
            if ch == "r":
                self.ts.thruster[6].data+=300
            if ch == "M":
                break     
            self.ts.control()
            rate.sleep()


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
    
    


