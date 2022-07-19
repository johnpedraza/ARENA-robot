"""
filter.py: filters various sensor ouptut

Created by John Pedraza on July 3rd, 2022.

Copyright (c) 2022, The CONIX Research Center
All rights reserved.

This source code is licensed under the BSD-3-Clause license found in the
LICENSE file in the root directory of this source tree.
"""

from json import JSONEncoder, dumps, loads
from datetime import datetime
from scipy.spatial.transform import Rotation as R
from arena import Device
from paho.mqtt.client import MQTTMessage
from arenarobot.service.processor import ArenaRobotServiceProcessor
from filterpy.kalman import predict, update, KalmanFilter
from filterpy.common import Q_discrete_white_noise
import numpy as np
import time
import pandas as pd
import atexit # is there an ARENA-robot builtin for this?

def printFormattedPose(pose):
    print(pose[0], '\n', pose[1], '\n', pose[2], '\n', pose[3])

# TODO: If filter service is started before apriltag detector service, this doesn't work
class ArenaRobotServiceProcessorFilter(ArenaRobotServiceProcessor):
    DEVICE_INSTANCE_PROCESSOR_TYPE = "filter"
    def __init__(self, 
                 apriltag_topic = None,
                 vio_topic = None,
                 optitrack_topic = None, 
                 record: bool = False, # whether or not to record pose data
                 recording_id: int = -1,
                 **kwargs):
        
        self.apriltag_topic = apriltag_topic
        self.vio_topic = vio_topic
        self.optitrack_topic = optitrack_topic

        self.record = record
        self.recording_id = recording_id
        self.df_apriltags = None
        self.df_vio = None
        self.df_optitrack = None

        self.fetched_once = False
        self.prev_time = time.time()       
 
        # filtering
        self.filter = None

        processor_type = (ArenaRobotServiceProcessorFilter
                          .DEVICE_INSTANCE_PROCESSOR_TYPE)
        super().__init__(
            device_instance_processor_type=processor_type,
            device_instance_prefix="filter_",
            **kwargs
            )

    def setup(self):
        if self.record:
            self.df_apriltags = pd.DataFrame(columns=['time', 'position', 'rotation'])
            self.df_optitrack = pd.DataFrame(columns=['time', 'position', 'rotation'])
            self.df_vio = pd.DataFrame(columns=['time', 'position', 'rotation']) 

        # initialize filter
        self.filter = KalmanFilter(dim_x=6, dim_z=6)
        
        # state estimate vector
        self.filter.x = np.array([0, 0, 0, 0, 0, 0], dtype=float)
        
        # measurement function
        self.filter.H = np.eye(6, dtype=float)
        
        # measurement covariance
        self.filter.R = 0.175
        
        # filter state covariance
        self.filter.P = np.diag([500.0, 100.0, 500.0, 100.0, 500.0, 100.0])

        super().setup()

    def fetch(self):
        # Ensure not run more than once (sets up callback)
        if self.fetched_once:
            print("Filter processor should have interval of -1!")
            return
        self.fetched_once = True

        def exit_handler():
            self.df_optitrack.to_csv('recording_optitrack.csv')
            self.df_apriltags.to_csv('recording_apriltags.csv')
            self.df_vio.to_csv('recording_vio.csv')
        atexit.register(exit_handler)

        '''
        Filter receives pose data from various sensors via MQTT and outputs 
        filtered pose.
        '''
        def filter_data(client, userdata, msg: MQTTMessage):
            """Filter sensor data."""
            currTime = time.time()
            dt = currTime - self.prev_time
            payload = self.decode_payload(msg)
            
            self.filter.Q = Q_discrete_white_noise(dim=2, dt=dt, var=0.175, block_size=3)

            if payload is None:
                return

            if payload["device_instance_processor_type"] == "apriltag_detector":
                if payload["msg"]["data"]["pose"] is None:
                    print('no apriltags in frame')
                else:
                    printFormattedPose(payload["msg"]["data"]["pose"])
                    
                    
            self.prev_time = currTime

        def process_apriltag(client, userdata, msg: MQTTMessage):
            payload = self.decode_payload(msg)
            print(f'Apriltag Payload: {payload}')
            if payload:
                new_df = pd.DataFrame([[payload['timestamp'],
                                        payload['msg']['data']['pose']['position'],
                                        payload['msg']['data']['pose']['rotation']]],
                                       columns=['time', 'position', 'rotation'])
                self.df_apriltags = pd.concat([self.df_apriltags, new_df])
        
        def process_vio(client, userdata, msg: MQTTMessage):
            payload = self.decode_payload(msg)
            if payload:
                print(f'VIO Payload: {payload}')
                pose_mtx = payload['msg']['data']['h_t265_t265body']
                position = (pose_mtx[0][3], pose_mtx[1][3], pose_mtx[2][3])
                r = R.from_matrix([[pose_mtx[0][0], pose_mtx[0][1], pose_mtx[0][2]],
                                   [pose_mtx[1][0], pose_mtx[1][1], pose_mtx[1][2]],
                                   [pose_mtx[2][0], pose_mtx[2][1], pose_mtx[2][2]]]).as_quat()
                rotation = (r[0], r[1], r[2], r[3])
                new_df = pd.DataFrame([[payload['timestamp'], 
                                        position, 
                                        rotation]], 
                                        columns=['time', 'position', 'rotation'])
                self.df_vio = pd.concat([self.df_vio, new_df])

        def process_optitrack(client, userdata, msg: MQTTMessage):
            payload = self.decode_payload(msg)
            if payload:
                print(f'Optitrack Payload: {payload}')
                new_df = pd.DataFrame([[payload['timestamp'], 
                                        payload['msg']['data']['pose']['position'], 
                                        payload['msg']['data']['pose']['rotation']]], 
                                        columns=['time', 'position', 'rotation'])
                self.df_optitrack = pd.concat([self.df_optitrack, new_df])
        
        if self.apriltag_topic:
            self.device.message_callback_add(self.apriltag_topic, process_apriltag)
        if self.vio_topic:
            self.device.message_callback_add(self.vio_topic, process_vio)
        if self.optitrack_topic:
            self.device.message_callback_add(self.optitrack_topic, process_optitrack)
