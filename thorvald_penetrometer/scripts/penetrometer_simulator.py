#!/usr/bin/env python

import rospy
#import actionlib
import serial
import threading
#import yaml



class PenetrometerSim(object):

    _special_cmd=['@','!','z']
    
    def __init__(self, name):
        self.cancelled = False
        self.running=True
        self.reply_buf=[]
        self.last_response = ''
        self.config={}
        
        rospy.on_shutdown(self._on_shutdown)
        self.serial_port = rospy.get_param('~serial_port', '/dev/tnt1')     

        rospy.loginfo("opening serial port")
        self.ser = serial.Serial(self.serial_port, 57600, timeout=0, parity=serial.PARITY_NONE)
        thread = threading.Thread(target=self.read_from_port)#, args=(serial_port,))
        thread.start()
        

        rospy.loginfo("ALL done ...")
        
        
        while self.running:
            self.process_data()
            rospy.sleep(0.01)            
        
        #self.write_config_to_file()
        self.running=False
        self.ser.close()
        
    def process_data(self):
        if self.reply_buf:
            current = self.reply_buf.pop(0)
            if current[0] not in self._special_cmd:
                current[0] = current[0].capitalize()
                reply = ''.join(current)
                self.send_reply(reply)
            elif current[0] == '@':
                self.send_reply('@1')
            elif current[0] == '!':
                self.send_reply('!1')
            elif current[0] == 'z':
                self.send_reply('Z1')                
                
    def read_from_port(self):
        serial_buffer=[]
        while self.running:
            if (self.ser.inWaiting()>0):                                        #if incoming bytes are waiting to be read from the serial input buffer
                data_str = self.ser.read(self.ser.inWaiting())#.decode('ascii')  #read the bytes and convert from binary array to ASCII
                print "|"+data_str+"|"
                for i in data_str:
                    serial_buffer.append(i)
                while '\n' in serial_buffer:
                    #print "end found"
                    nind= serial_buffer.index('\n')
                    self.reply_buf.append(serial_buffer[0:nind])
#                    pen_data = ''.join(serial_buffer[0:nind])
                    for i in reversed(range(nind+1)):
                        serial_buffer.pop(i)
                    print serial_buffer
                    print self.reply_buf
            rospy.sleep(0.001) # Optional: sleep 10 ms (0.01 sec) once per loop to let other threads on your PC run 


    def send_reply(self, command):
        for i in command:
            self.ser.write(i)
            #rospy.sleep(0.001)
        self.ser.write('\n')
    
    def _on_shutdown(self):
        self.running=False

if __name__ == '__main__':
    rospy.init_node('penetrometer_simulator')
    server = PenetrometerSim(rospy.get_name())
