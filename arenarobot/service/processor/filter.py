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
from arena import Device, Scene, Box, Position
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

        self.position = None
        self.rotation = None
        self.prev_global_position = None
        self.prev_global_rotation = None
        self.prev_global_t265_position = None
        self.prev_global_t265_rotation = None

        self.global_known = False 
 
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

        self.position = self.prev_global_position = (0, 0, 0)
        self.rotation = self.prev_global_rotation = (0, 0, 0, 1)

        self.prev_global_t265_position = (0, 0, 0)
        self.prev_global_t265_rotation = (0, 0, 0, 1)

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

        def process_apriltag(client, userdata, msg: MQTTMessage):
            payload = self.decode_payload(msg)
            if payload:
                pos = payload['msg']['data']['pose']['position']
                rot = payload['msg']['data']['pose']['rotation']
                if pos and rot:
                    self.position = self.prev_global_position = (pos[0], pos[1], pos[2])
                    self.rotation = self.prev_global_rotation = (rot[0], rot[1], rot[2], rot[3])
                    self.global_known = True
                    print(f'Position = {self.position}')
                    print(f'Rotation = {self.rotation}')
                    out = {
                    "pose": {"position": self.position, "rotation": self.rotation}
                    }
                    serializable_out = loads(dumps(
                        out,
                        cls=TransformedApriltagDetectorJSONEncoder
                    ))

                    # this publishes to subtopic (self.topic)
                    self.publish({"data": serializable_out})
        
        def process_vio(client, userdata, msg: MQTTMessage):
            payload = self.decode_payload(msg)
            if payload:
                pose = payload['msg']['data']['h_t265_t265body']
                if self.global_known:
                    self.prev_global_t265_position = (pose[0][3], pose[1][3], pose[2][3])
                    r = R.from_matrix([[pose[0][0], pose[0][1], pose[0][2]],
                                       [pose[1][0], pose[1][1], pose[1][2]],
                                       [pose[2][0], pose[2][1], pose[2][2]]]).as_quat()
                    self.prev_global_t265_rotation = (r[0], r[1], r[2], r[3])
                    self.global_known = False
                self.position = (self.prev_global_position[0] + (pose[0][3] - self.prev_global_t265_position[0]),
                                 self.prev_global_position[1] + (pose[1][3] - self.prev_global_t265_position[1]),
                                 self.prev_global_position[2] + (pose[2][3] - self.prev_global_t265_position[2]))
                out = {
                "pose": {"position": self.position, "rotation": self.rotation}
                }
                serializable_out = loads(dumps(
                    out,
                    cls=TransformedApriltagDetectorJSONEncoder
                ))
                self.publish({"data": serializable_out})
                print(self.position)
        
        if self.apriltag_topic:
            self.device.message_callback_add(self.apriltag_topic, process_apriltag)
        if self.vio_topic:
            self.device.message_callback_add(self.vio_topic, process_vio)


class TransformedApriltagDetectorJSONEncoder(JSONEncoder):
    """JSON Encoder helper for Apriltag Detector transformed attributes."""

    def default(self, o):
        """JSON Encoder helper for Apriltag Detector pose attributes."""
        if isinstance(o, (np.ndarray)):
            return list(o)

        return JSONEncoder.default(self, o)