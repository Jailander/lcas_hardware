#! /usr/bin/env python

import rospy
#import sys
import math
import yaml


import matplotlib.pyplot as plt
import actionlib
import thorvald_penetrometer.msg
from sensor_msgs.msg import Joy

class penetrometer_probe_client(object):
    
    def __init__(self) :
        self.plotted=False
        self.probing=False
        rospy.on_shutdown(self._on_node_shutdown)
        self.client = actionlib.SimpleActionClient('/thorvald_penetrometer', thorvald_penetrometer.msg.ProbeSoilAction)
        
        self.client.wait_for_server()
        rospy.Subscriber("/joy", Joy, self.joy_callback)
        rospy.loginfo(" ... Init done")
    
    def joy_callback(self, data):
        if data.buttons[7] and not self.probing:
            self.do_probe()
    
    def do_probe(self):
        self.probing=True
        probe_radius = rospy.get_param('~probe_radius', 0.0127/2.0)
        probe_area = math.pow(probe_radius,2)*math.pi
        kparea = 1000*probe_area
        probegoal = thorvald_penetrometer.msg.ProbeSoilGoal()
    
        
        probegoal.order = 0
        #navgoal.origin = orig
    
        # Sends the goal to the action server.
        self.client.send_goal(probegoal)#,self.done_cb, self.active_cb, self.feedback_cb)
    
        # Waits for the server to finish performing the action.
        self.client.wait_for_result()
    
        # Prints out the result of executing the action
        ps = self.client.get_result()  # A FibonacciResult
        print ps
        if ps.result:
            to_kpa = lambda newts : newts/kparea
            in_kpa = [to_kpa(x) for x in ps.force]
            #plt.plot(ps.depth, in_kpa)
            #self.plotted=True
            #plt.show()
            d={}
            d['depth']=ps.depth
            d['newtons']=ps.force
            d['kpa']=in_kpa
            yml = yaml.safe_dump(d, default_flow_style=False)
            tim = str(rospy.Time.now())+'.yaml'
            fh = open(tim, "w")
            s_output = str(yml)
            fh.write(s_output)
            fh.close()


        else:
            rospy.logerr("Probe Failed")

        self.probing=False

    def _on_node_shutdown(self):
        self.client.cancel_all_goals()
#        if self.plotted:
#            plt.close('all')
        #sleep(2)


if __name__ == '__main__':
    rospy.init_node('penetrometer_client')
    ps = penetrometer_probe_client()
