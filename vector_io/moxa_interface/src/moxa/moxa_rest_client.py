import requests
import threading
import json
import rospy
from moxa_msgs.msg import Regs,Coils

class MoxaRestClient(object):
    def __init__(self):
        
    
        self._host_ip = rospy.get_param('~host_ip','10.66.171.30')
        self._prefix = rospy.get_param('~prefix','moxa_iologik')
        self._rate = rospy.get_param('~update_rate_hz',10.0)
        
        self._lock = threading.Lock()
        
        self.di_url = 'http://%s/api/slot/0/io/di'%self._host_ip
        self.relay_url = 'http://%s/api/slot/0/io/relay'%self._host_ip
        self.info_url = 'http://%s/api/slot/0/sysInfo/device'%self._host_ip
        self.head = {'Content-Type': 'application/json',
                     'Accept':'vdn.dac.v1'}
        
        ret = requests.get(self.info_url,headers=self.head)
        
        if (200 != ret.status_code):
            rospy.logerr("Failed to connect to %s moxa ioLogik server at IP address %s"%(self._prefix,self._host_ip))
            rospy.logerr("Received RESTFul exception: %d"%ret)
            self.init_success = False
            return
        
        self.regs = Regs()
        self._pub = rospy.Publisher('/%s/feedback'%self._prefix,Regs,queue_size=10)
        self._sub = rospy.Subscriber('/%s/command'%self._prefix,Coils,self._update_cmd)
        self.t1 = rospy.Timer(rospy.Duration(1/self._rate), self._run)
        self.init_success = True

    def Close(self):
        with self._lock:
            self.t1.shutdown()
            self._pub.unregister()
            self._sub.unregister()
        
    def _update_cmd(self,cmd):
        with self._lock:
            coils = cmd.coils

            if (6 != len(coils)):
                rospy.logerr("Too many coils specified in command for %s moxa ioLogik modbus server at IP address %s"%(self._prefix,self._ip_address))
                return
            
            ret = requests.get(self.relay_url,headers=self.head)
            tmp = ret.json()
            
            for x in range(len(tmp[u'io'][u'relay'])):
                tmp[u'io'][u'relay'][x][u'relayStatus'] = coils[x]
            
            payld = json.dumps(tmp)
            
            ret = requests.put(self.relay_url,headers=self.head,data=payld)
            if 200 != ret.status_code:
                rospy.logerr("Command failed for %s moxa ioLogik modbus server at IP address %s"%(self._prefix,self._ip_address))

    def _run(self,event):
        
        with self._lock:
            ret = requests.get(self.di_url,headers=self.head)
            if(200 == ret.status_code):
                tmp = ret.json()[u'io'][u'di']
                self.regs.inputs = [tmp[x][u'diStatus'] for x in range(len(tmp))]
            else:
                self.regs.inputs = None

            ret = requests.get(self.relay_url,headers=self.head)
            if(200 == ret.status_code):
                tmp = ret.json()[u'io'][u'relay']
                self.regs.outputs = [tmp[x][u'relayStatus'] for x in range(len(tmp))]
            else:
                self.regs.outputs = None

            if not self.regs.outputs or not self.regs.inputs:
                rospy.logerr("Read failed for %s moxa ioLogik server at IP address %s"%(self._prefix,self._host_ip))
                return            
            
            self.regs.header.stamp = rospy.get_rostime()
            self.regs.header.frame_id = ''
            self.regs.header.seq +=1
            
            self._pub.publish(self.regs)