#!/usr/bin/env python3
import sys
import cv2
import os
from subprocess import call
import subprocess
import threading

def set_manual(camera_id):
    subprocess.Popen(['v4l2-ctl', '-d', '/dev/video'+str(camera_id),'-c', 'auto_exposure=1'],stdout=subprocess.PIPE)
    subprocess.Popen(['v4l2-ctl', '-d', '/dev/video'+str(camera_id),'-c', 'gain_automatic=1'],stdout=subprocess.PIPE)

def set_gain(camera_id,val):
    subprocess.Popen(['v4l2-ctl', '-d', '/dev/video'+str(camera_id),\
                      '-c', 'gain='+str(val)],stdout=subprocess.PIPE)
    print('The gain for the camera'+str(camera_id)+' has been set to:'+str(val))
    return True

def set_exposure(camera_id,val):
    subprocess.Popen(['v4l2-ctl', '-d', '/dev/video'+str(camera_id),\
                      '-c', 'exposure='+str(val)],stdout=subprocess.PIPE)
    print('The exposure for the camera'+str(camera_id)+' has been set to:'+str(val))

def usb_ports2id(port_list):
    if isinstance(port_list,list)==False:
        port_list=[port_list]
        print(port_list)
    #Get the info for the video devices connected to the system
    tst=subprocess.Popen(['v4l2-ctl', '--list-devices'],stdout=subprocess.PIPE)
    comInfo = tst.stdout.read().decode("utf-8") 
    info = comInfo.split('USB Camera')[1:]
    #Find out what /dev/videox corresponds to the given port name in the parameters
    found={}
    for lst in info:
        for idx in port_list:
            if lst.find(idx) is not -1:
                found[idx]=int(lst[lst.find('video')+5])
    return found
