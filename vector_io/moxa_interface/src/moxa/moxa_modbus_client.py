from pyModbusTCP.client import ModbusClient
import rospy
from moxa_msgs.msg import Regs,Coils
import threading

class MoxaModbusClient(object):
    def __init__(self):
        self._lock = threading.Lock()
        self.init_success = True

        self._ip_address = rospy.get_param('~host_ip','10.66.171.30')
        self._prefix = rospy.get_param('~prefix','moxa_iologik')
        self._port_num = rospy.get_param('~host_port',502)
        self._timeout = rospy.get_param('~timeout',2)
        self._rate = rospy.get_param('~update_rate_hz',10.0)

        try:
            self._client = ModbusClient(host=self._ip_address, port=self._port_num, auto_open=True, timeout=self._timeout)
        except:
            rospy.logerr("Failed to connect to %s moxa ioLogik modbus server at IP address %s"%(self._prefix,self._ip_address))
            rospy.logerr("Received exception: %s"%self._client.last_except())             
            self.init_success = False
            self._client.close()
            return
        
        self.regs = Regs()
        self._pub = rospy.Publisher('/%s/feedback'%self._prefix,Regs,queue_size=10)
        self._sub = rospy.Subscriber('/%s/command'%self._prefix,Coils,self._update_cmd)
        self.t1 = rospy.Timer(rospy.Duration(1/self._rate), self._run)

    def Close(self):
        with self._lock:
            self.t1.shutdown()
            self._pub.unregister()
            self._sub.unregister()
            self._client.close()
        
    def _update_cmd(self,cmd):
        with self._lock:
            coils = [ord(i) for i in cmd.coils]
            if (6 != len(coils)):
                rospy.logerr("Too many coils specified in command for %s moxa ioLogik modbus server at IP address %s"%(self._prefix,self._ip_address))
                return
            
            if not self._client.write_multiple_coils(0, coils):
                rospy.logerr("Command failed for %s moxa ioLogik modbus server at IP address %s"%(self._prefix,self._ip_address))
    
    def _run(self,event):
        
        with self._lock:

            relay = self._client.read_holding_registers(32, 1)
            inputs = self._client.read_input_registers(48, 1)
            
            self.regs.outputs = [(relay[0] >> i)&1 for i in range(6)]
            self.regs.inputs =  [(inputs[0] >> i)&1 for i in range(6)]
            
            if not self.regs.outputs or not self.regs.inputs:
                rospy.logerr("Read failed for %s moxa ioLogik modbus server at IP address %s"%(self._prefix,self._ip_address))
                return            
            
            self.regs.header.stamp = rospy.get_rostime()
            self.regs.header.frame_id = ''
            self.regs.header.seq +=1
            
            self._pub.publish(self.regs)
