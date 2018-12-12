#! /usr/bin/env python
import os
import rospy
#import sys
import math
import yaml


import matplotlib.pyplot as plt
import actionlib
import thorvald_penetrometer.msg
from sensor_msgs.msg import Joy

from sensor_msgs.msg import NavSatFix
from std_msgs.msg import String

class penetrometer_probe_client(object):
    
    def __init__(self) :
        self.predata =None
        self.plotted=False
        self.probing=False
        self.last_fix=None
        self.current_node='none'
        self.resseting=False
        rospy.on_shutdown(self._on_node_shutdown)
        self.client = actionlib.SimpleActionClient('/thorvald_penetrometer', thorvald_penetrometer.msg.ProbeSoilAction)
        
        self.client.wait_for_server()
        rospy.Subscriber("/joy", Joy, self.joy_callback)
        rospy.Subscriber("/rtk_fix", NavSatFix, self.fix_callback)
        rospy.Subscriber("/current_node", String, self.node_callback)
        self.pub = rospy.Publisher("/scanned_node", String)
        rospy.loginfo(" ... Init done")
        rospy.spin()
        
    
    def node_callback(self, data):
        self.current_node=data
        
    def fix_callback(self, data):
        self.last_fix=data
    
    def joy_callback(self, data):
        
        if self.predata:
            if data.buttons[7] and not self.predata.buttons[7]: 
                if not self.probing:
                    print "probing"
                    self.do_probe()
            if data.buttons[6] and not self.predata.buttons[6]:
                if not self.resseting:
                    self.reset_penetrometer()
        self.predata=data
    
    
    def reset_penetrometer(self):    
        self.resseting=True
        os.system("rosnode kill /thorvald_penetrometer")
        rospy.sleep(10)
        self.resseting=False
    
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

        to_kpa = lambda newts : newts/kparea
        in_kpa = [to_kpa(x) for x in ps.force]
        #plt.plot(ps.depth, in_kpa)
        #self.plotted=True
        #plt.show()
        d={}
        d['depth']=ps.depth
        d['newtons']=ps.force
        d['kpa']=in_kpa
        d['result']= ps.result
        tim = str(rospy.Time.now())
        if self.last_fix:
            d['coord']={}
            d['coord']['lat']= self.last_fix.latitude
            d['coord']['lon']= self.last_fix.longitude
        d['timestamp']=tim
        d['node']=str(self.current_node.data)
        yml = yaml.safe_dump(d, default_flow_style=False)
        fim = str(self.current_node.data)+'-'+tim+'.yaml'
        fh = open(fim, "w")
        rospy.loginfo("SAVING")
        s_output = str(yml)
        fh.write(s_output)
        fh.close()
        self.pub.publish(self.current_node.data)

        if not ps.result:
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
