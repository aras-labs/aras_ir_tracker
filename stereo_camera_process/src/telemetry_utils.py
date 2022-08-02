import socket
import struct
import numpy as np
import time
import pickle

class udp_telemetry():
    def __init__(self):
        self.OUT_IP='192.168.1.3'
        self.OUT_PORT=5000
        self.INPUT_IP='127.0.0.1'
        self.INPUT_PORT=5000
        self.out_socket=socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
        self.RX_RUNNING = True
        self.transmit_counter = 0

    def transmit_data(self,data):
        byte_object = pickle.dumps(data)
        self.out_socket.sendto(byte_object, (self.OUT_IP, self.OUT_PORT))

    def transmit_matlab(self,data_list, MATLAB_IP, MATLAB_PORT):
        '''Transmit a list of double to the matlab simulink target'''
        msg_format=f'{len(data_list)}d'
        arguments =  [msg_format] + data_list
        data=struct.pack(*arguments)        
        self.out_socket.sendto(data, (MATLAB_IP, MATLAB_PORT))

    def start_rx(self,callback):
        self.in_sock = socket.socket(socket.AF_INET, # Internet
                     socket.SOCK_DGRAM) # UDP
        self.in_sock.bind((self.INPUT_IP, self.INPUT_PORT))

        while self.RX_RUNNING:
            data = self.in_sock.recvmsg(4096)
            data = pickle.loads(data[0])
            reception_stamp = time.time()
            callback(reception_stamp, data)

    def terminate(self):
        self.RX_RUNNING = False
        in_sock.close()
        out_sock.close()
