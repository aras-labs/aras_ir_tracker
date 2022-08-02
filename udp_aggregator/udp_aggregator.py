import numpy as np
import yaml
import pickle
from src.telemetry_utils import *
import argparse
import yaml
import os
import time
import signal
import sys
import threading

# A class to hold the received data and record them into a file when required
class recorderClass():
    def __init__(self, path, ports_list):
        self.path = path
        self.ports_list = ports_list
        self.data ={i:[] for i in ports_list}

    def update(self,port,data):
        self.data[port].append(data)

    def record_to_file(self):
        timetup = time.localtime()
        stamp=time.strftime('%Y-%m-%d-%H:%M:%S', timetup)
        with open(os.path.join(self.path, f'recorded-{stamp}.pckl'), 'wb') as f:
            pickle.dump(self.data,f)

# A Que class that stores a moving widow of recent received frames and return frames that are closest to a stamp 
class PacketQue():
    def __init__(self, stack_len = 100):
        self.packet_data = []
        self.packet_stamps = []
        self.stack_len = stack_len

    def push(self,stamp, packet):
        self.packet_data.append(packet)
        self.packet_stamps.append(stamp)
        
        if len(self.packet_data)>self.stack_len:
            #Remove the oldest packet if the lenght limit is reached
            del self.packet_data[0]   
            del self.packet_stamps[0]

    def returnClosestPacket(self, stamp, threshold = 0.15):
        idx = np.argmin( np.abs(np.array(self.packet_stamps)- stamp) )
        if np.abs(self.packet_stamps[idx]-stamp) < threshold:
            return [self.packet_stamps[idx], self.packet_data[idx]]
        else:
            return None

# This function generates a clean data structure that contains a unified timestamp plus marker dictionary
def packetGenerator(data):
    # data: [stamp, [port, [local_stamp, [markers1, markers2] ]], ... ]
    stamp = data[0]
    ports = [ (i, data[i][0]) for i in range(1, len(data))]

    if len(ports) == 0:
        return None

    data_per_port = [ (data[i][0] ,data[i][1][1] ) for i in range(1, len(data))] # [ (port, [markers1, markers2]), ...]
    
    marker_data = {}

    for port_data in data_per_port:
        port = port_data [0]
        cams_per_port = port_data[1]
        for i,markers in enumerate(cams_per_port):
            marker_data[f'{port}_{i}'] = markers
    
    print(marker_data)
    return [stamp, marker_data]

# This callback is called by the RX functions of the telemetry class on the reception of new UDP packets from the UDP marker publisher nodes
def callback(stamp,local_stamp, seq_num, port, markers_in_frames):
    packet_ques[port].push(local_stamp, markers_in_frames)

    #Now aggregate all ques and make a unified packet to transmit to the reconstruction node
    # ToDo: implement the transmit telemetry and aggregator
    aggregated_data = []
    for key in packet_ques.keys():
        nearest_packet = packet_ques[key].returnClosestPacket(local_stamp, threshold=0.1)
        if nearest_packet is not None:
            aggregated_data+=[port, nearest_packet]

    #Transmit the aggregated data to the reconstruction nodes through UDP
    data = [local_stamp, aggregated_data]
    packet = packetGenerator(data)
    data_out_telemetry.transmit_data(packet) 

    #Print received data if required
    if configs['print_markers']:
        print('-------------------------')
        print(packet)
        print('-------------------------')
        
    # If required, record the marker locations into the recorder class
    if configs['record']:
        recorder.update(port, [stamp,local_stamp, seq_num, markers_in_frames])

# A function that is called on the reception of Ctrl-C interrupt. It cleans up the system and stores the data
def termination_handling(signum,frame):
    print('\nTerminating')
    #Terminate the data reception threads by setting their running flags to false
    for telemetry_object in telemetry_objects:
        telemetry_object.RX_RUNNING = False
    #Store the data recorded into the recorder class into a file
    if configs['record']:
        recorder.record_to_file()
    sys.exit()

if __name__=='__main__':
    parser = argparse.ArgumentParser(description = 'A prgram to capture the UDP markers from multiple marker publisher nodes and aggregate them.')
    parser.add_argument('config_file', type=str, help='The path to the config yaml file')
    args = parser.parse_args()
    # load the configureation paramters from the YAML config file
    with open(args.config_file,'r') as f:
        configs = yaml.safe_load(f)

    #For each UDP camera node in the system, generate a telemetry object and a specific thread to handle it
    telemetry_objects=[]
    rx_threads = []
    packet_ques = {}

    # Make a telemetry object to output the aggregated markers
    data_out_telemetry=udp_telemetry()
    data_out_telemetry.OUTPUT_IP = configs['remote_ip']
    data_out_telemetry.OUTPUT_PORT = configs['remote_port']
    #Each UDP camera node is designated by a unique port number. For each prot, instantiate a telemetry object
    for port in configs['camera_ports']:
        packet_ques[port] = PacketQue(stack_len = 100)
        obj=udp_telemetry()
        obj.INPUT_IP = configs['local_ip']
        obj.INPUT_PORT = port
        telemetry_objects.append(obj)
        # To handle the data reception of each telemetry object create a thread and hand over the callback function 
        # that should be called on the reception of each packet
        rx_threads.append(threading.Thread(target=obj.start_rx, args =(callback,)))
    # Handle Ctrl-C interrupt correctly by calling a function to clean up the system and story the data
    signal.signal(signal.SIGINT, termination_handling)
    # If required, we can record the received data into a file at the end of the node execution
    if configs['record']:
        recorder = recorderClass(configs['path'], configs['camera_ports'])
    # Start the threads 
    for thread in rx_threads:
        thread.start()
