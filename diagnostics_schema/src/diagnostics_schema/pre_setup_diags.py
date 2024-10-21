#!/usr/bin/env python3

import rospy
from abc import ABC
from abc import abstractmethod
import subprocess
import os
import ipaddress

from colorama import init as colorama_init
from colorama import Fore
from colorama import Style

colorama_init()

import cv2

CRITICAL_REQUIREMENT = 'Critical: '
REQUIREMENT          = 'Failed: '
OPTIONAL_REQUIREMENT = 'Optional: '



class ATest(ABC):
    def __init__(self, criticality  = REQUIREMENT):
        super().__init__()
        self.criticality = criticality

    @abstractmethod
    def run(self):
        pass
    
    @abstractmethod
    def troubleshootingmsg(self):
        return []

    @abstractmethod
    def testname(self):
        return ""

class CheckOwnHost(ATest):
    def __init__(self, own_ip="192.168.1.100", criticality = 'Critical: '):
        super().__init__(criticality)
        self.is_hotspot = False
        try:
            if os.environ['USE_HOTSPOT'] == "true":
                own_ip = "192.168.1.1"
                self.is_hotspot = True
        except:
            rospy.logwarn("USE_HOTSPOT variable not set. I will assume I am not a hotspot!")
        ipaddress.ip_address(own_ip)
        self.own_ip = own_ip

    def run(self):
        self.hostreturn = subprocess.run(["hostname","-I"], capture_output=True, text = True)
        try:
            self.hostreturn.check_returncode()
            if self.own_ip in self.hostreturn.stdout:
                return 'OK'
            else:
                return self.criticality+self.hostreturn.stderr
        except:
            return self.criticality+self.hostreturn.stderr


    def troubleshootingmsg(self):
        if self.is_hotspot:
            return [f"??? This pc is set as a hotspot, however its own ip is not set to 192.168.1.1, which is weird. If you know what you are doing and you changed the network configurations, please fix this node {__file__} as well! "]
        return [f"Check if this pc's cable is connected to the router via ethernet cable", "Make sure that the cable is connected and both ends and is in the correct port on the router.", "Check if router is on.", "Check if router's power supply is connected.","Check if this system is somehow running on AP mode"]

    def testname(self):
        return f"Check if this pc has ip [{Style.BRIGHT}{self.own_ip}{Style.NORMAL}]."


class PingHost(ATest):
    def __init__(self, hostname, hostip, criticality = REQUIREMENT):
        super().__init__(criticality)
        ipaddress.ip_address(hostip)
        self.host = hostname
        self.hostip = hostip

    def run(self):
        self.hostreturn = subprocess.run(["ping","-W","1","-c","1",self.hostip], capture_output=True, text = True)
        try:
            self.hostreturn.check_returncode()
            return 'OK'
        except:
            return self.criticality+self.hostreturn.stderr


    def troubleshootingmsg(self):
        return [f"Make sure host [{self.host}] is on", f"Make sure that the IP address of the host [{self.host}] is set to {self.hostip}", f"Make sure that {self.host} is connected to the network:\n\t-if {self.host} is using wifi connection, check if it is set to the correct AP,\n\t-if {self.host} is linked via ethernet cable, make sure that the cable is connected and both ends and is in the correct port on the router."]

    def testname(self):
        return f"Check if host [{Style.BRIGHT}{self.host}{Style.NORMAL}] with ip [{Style.BRIGHT}{self.hostip}{Style.NORMAL}] is alive"


class Sound(ATest):
    def __init__(self, soundfilename, criticality= REQUIREMENT):
        super().__init__(criticality)
        if not os.path.exists(soundfilename):
            raise Exception("invalid sound filename")
        self.soundfile = soundfilename

    def run(self):
        self.soundreturn = subprocess.run(["aplay",self.soundfile], capture_output=True,text = True)
        try:
            self.soundreturn.check_returncode()
            return 'OK'
        except:
            return self.criticality+self.soundreturn.stderr

    def troubleshootingmsg(self):
        return ["You probably want to check the sharing options of the docker, there is likely some mistake there.","Maybe the file doesn't exist?"]
    def testname(self):
        return f"Check if alsa can play sound file: {self.soundfile}"

class Video(ATest):
    def __init__(self, videodevname, criticality = REQUIREMENT):
        super().__init__(criticality)
        if "/dev/video" not in videodevname:
            raise Exception(f"invalid device name {videodevname}")
        self.videodev = videodevname
        try:
            self.dev = int(self.videodev.split("/dev/video")[1])
        except:
            raise Exception("invalid device name {videodevname}")

    def run(self):
        if not os.path.exists(self.videodev):
            return f"{self.criticality} device does not exist {self.videodev}"
        cam = cv2.VideoCapture(self.dev)
        try:
            ret, frame = cam.read()

            if not ret:
                return f'{self.criticality} Could not read from capture device {self.dev}'
            cam.release()

            return 'OK'
        except:
                return f'{self.criticality} Could not read from capture device {self.dev}'

    def troubleshootingmsg(self):
        return ["You need to connect the external USB camera before starting the docker.","Maybe you don't want the camera?"]
    def testname(self):
        return f"Check if opencv can read from device: [{Style.BRIGHT}{self.videodev}{Style.NORMAL}]"

def do(tests):
    fail_bin = []
    for test in tests:
        ret = test.run() 
        if REQUIREMENT in ret:
            rospy.logerr(f"\t[{Style.BRIGHT}{Style.NORMAL}] {test.testname()}")
            fail_bin.append(test.testname())
            msg =" ".join(ret.split(REQUIREMENT)[1:]) 
            if msg:
                rospy.logerr("\t"+msg)
            for sugg in test.troubleshootingmsg():
                rospy.logwarn(f"\t\t[{Style.BRIGHT}{Style.NORMAL}] {sugg}")
        if OPTIONAL_REQUIREMENT in ret:
            rospy.logwarn(f"\t[{Style.BRIGHT}{Style.NORMAL}] {test.testname()}")
            fail_bin.append(test.testname())
            msg =" ".join(ret.split(REQUIREMENT)[1:]) 
            if msg:
                rospy.logerr("\t"+msg)
            for sugg in test.troubleshootingmsg():
                rospy.logwarn(f"\t\t[{Style.BRIGHT}{Style.NORMAL}] {sugg}")
        elif CRITICAL_REQUIREMENT in ret:
            rospy.logfatal(f"\t{Style.BRIGHT}{Style.NORMAL} {test.testname()}")
            msg =" ".join(ret.split(CRITICAL_REQUIREMENT)[1:]) 
            if msg:
                rospy.logerr("\t"+msg)
            for sugg in test.troubleshootingmsg():
                rospy.logwarn(f"\t\t[{Style.BRIGHT}{Style.NORMAL}] {sugg}")
            exit(1)
        else:
            rospy.loginfo(f"\t[{Style.BRIGHT}{Fore.GREEN}{Fore.WHITE}{Style.NORMAL}] "+test.testname())
    if len(fail_bin) > 0:
        rospy.logwarn("You have some warnings you may want to solve before testing, but system is usable.")

    rospy.spin()
