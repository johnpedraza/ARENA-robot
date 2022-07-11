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

def printFormattedPose(pose):
    print(pose[0], '\n', pose[1], '\n', pose[2], '\n', pose[3])

# TODO: If filter service is started before apriltag detector service, this doesn't work
class ArenaRobotServiceProcessorFilter(ArenaRobotServiceProcessor):
    DEVICE_INSTANCE_PROCESSOR_TYPE = "filter"
    def __init__(self, sensor_topic, **kwargs):
        
        self.sensor_topic = sensor_topic

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

        def print_t265(client, userdata, msg: MQTTMessage):
            payload = self.decode_payload(msg)
            if payload:
                print(printFormattedPose(payload['msg']['data']['v_aeroref_aerobody']))

        def print_optitrack(client, userdata, msg: MQTTMessage):
            payload = self.decode_payload(msg)
            if payload:
                print(payload)
 
        self.device.message_callback_add(self.sensor_topic, filter_data)
        self.device.message_callback_add('realm/d/jpedraza/john-pi/processors/pose_transformed', print_t265)
        self.device.message_callback_add('realm/d/jpedraza/john-pi/processors/pose_optitrack', print_optitrack)

