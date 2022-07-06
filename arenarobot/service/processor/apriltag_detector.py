"""
apriltag_detector.py: reads frames, detects apriltags, and returns pose.

Created by John Pedraza on 6/7/22.

Copyright (c) 2022, The CONIX Research Center
All rights reserved.

This source code is licensed under the BSD-3-Clause license found in the
LICENSE file in the root directory of this source tree.
"""

from json import JSONEncoder, dumps, loads
import numpy as np
import cv2
from paho.mqtt.client import MQTTMessage
from transformations import euler_matrix, quaternion_matrix
from dt_apriltags import Detector
from arenarobot.service.processor import ArenaRobotServiceProcessor
from scipy.spatial.transform import Rotation as Ro
import time

HORIZONTAL_RESOLUTION_INDEX = 3
VERTICAL_RESOLUTION_INDEX = 4
        
FLIP_MATRIX = np.array([[1,  0,  0, 0],
                        [0, -1,  0, 0],
                        [0,  0, -1, 0],
                        [0,  0,  0, 1]], dtype=float)


# pylint: disable=too-many-instance-attributes
class ArenaRobotServiceProcessorApriltagDetector(ArenaRobotServiceProcessor):
    """Apriltag Detector processor class for the ARENA."""

    DEVICE_INSTANCE_PROCESSOR_TYPE = "apriltag_detector"

    # pylint: disable=too-many-arguments
    # pylint: disable=too-many-statements
    def __init__(self,
                 camera_resolution: list[int],
                 camera_params: list[int],
                 dist_params: list[int],
                 apriltag_family: str,
                 apriltag_locations: dict,
                 video_file: str,
                 num_detector_threads: int = 4,
                 quad_decimate: float = 1.0,
                 quad_sigma: float = 0.0,
                 refine_edges: int = 1,
                 decode_sharpening: float = 0.25,
                 tag_size: float = 0.15,
                 **kwargs):
        """Initialize the apriltag detector processor class."""
        
        self.cap                = video_file            # video capture
        self.camera_resolution  = camera_resolution     # camera resolution [width, height] (pixels)
        self.params             = camera_params         # camera matrix parameters fx, fy, cx, cy
        self.dist_params        = np.array(dist_params) # distortion parameters
       
        self.apriltag_family      = apriltag_family 
        self.num_detector_threads = num_detector_threads
        self.quad_decimate       = quad_decimate
        self.quad_sigma          = quad_sigma
        self.refine_edges        = refine_edges
        self.decode_sharpening   = decode_sharpening
        self.tag_size            = tag_size
        
        self.apriltags          = apriltag_locations    # known apriltag poses (world coordinates in meters)
        self.at_detector = None
        self.mtx = None
        
        processor_type = (ArenaRobotServiceProcessorApriltagDetector
                          .DEVICE_INSTANCE_PROCESSOR_TYPE)

        super().__init__(
            device_instance_processor_type=processor_type,
            device_instance_prefix="apriltag_detector_",
            **kwargs
        )
    '''
    Set up Apriltag Detector object and define a bunch of stuff
    '''
    def setup(self):
        self.cap = cv2.VideoCapture(self.cap)

        # subsample to increasing processing speed
        # note: selected resolution affects camera parameters
        self.cap.set(HORIZONTAL_RESOLUTION_INDEX, self.camera_resolution[0])
        self.cap.set(VERTICAL_RESOLUTION_INDEX, self.camera_resolution[1])

        # camera matrix
        self.mtx = np.array([[self.params[0], 0.000000000000, self.params[2]],
                             [0.000000000000, self.params[1], self.params[3]],
                             [0.000000000000, 0.000000000000, 1.000000000000]])

        # apriltag detector
        self.at_detector = Detector(searchpath=['apriltags'],
                                    families=self.apriltag_family,
                                    nthreads=self.num_detector_threads,
                                    quad_decimate=self.quad_decimate, # set to 2.0 for faster detections
                                    quad_sigma=self.quad_sigma,
                                    refine_edges=self.refine_edges,
                                    decode_sharpening=self.decode_sharpening,
                                    debug=0) # debug=1 saves images, don't use

        super().setup()

    '''
    Capture a camera frame, detect apriltags, calculate pose, and
    publish pose information to MQTT topic: realm/d/<user>/<device>/processors/apriltag_pose
    TODO: figure out how this works with different users/devices (not just jpedraza/john-pi)
          - so the device for the topic is just specified when running arena-robot-service
    TODO: handle getting wrong representation (apriltag solver has two possible solutions)
    TODO: handle multiple apriltags in frame at same time
    TODO: cap.release()?
    TODO: add among us support
    '''
    def fetch(self):
        # get grayscale frame for apriltag detection
        ret, frame = self.cap.read()
        if not ret:
            print('read() failed')
            return

        gray_image = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        
        # undistort frame
        h, w = gray_image.shape[:2]
        new_camera_matrix, roi = cv2.getOptimalNewCameraMatrix(self.mtx, self.dist_params, (w,h), 1, (w,h))
        dst = cv2.undistort(gray_image, self.mtx, self.dist_params, None, new_camera_matrix)
        
        # detect apriltags
        tags = self.at_detector.detect(dst, estimate_tag_pose=True, camera_params=self.params, tag_size=self.tag_size)
        
        pose = None
        if len(tags) > 0:
            Rot = tags[0].pose_R
            T = tags[0].pose_t

            # TODO: find a nicer way to notate/document this
            # M = [Rot,     T.T]
            #     [0, 0, 0, 1  ]
            
            T = [t[0] for t in T] 
            T = np.array([T])
            M = np.concatenate((Rot, T.T), axis=1)
            bottom_row = np.array([0, 0, 0, 1], dtype=float)
            M = np.concatenate((M, [bottom_row]), axis=0)
            
            if str(tags[0].tag_id) in self.apriltags: 
                tag_M = self.apriltags[str(tags[0].tag_id)]
                M = FLIP_MATRIX @ M @ FLIP_MATRIX
                pose = tag_M @ np.linalg.inv(M)
            else:
                print('Unknown Apriltag')
            
        out = {
            "pose": pose,
        }
        # print(out)

        serializable_out = loads(dumps(
            out,
            cls=TransformedApriltagDetectorJSONEncoder
        ))

        # this publishes to subtopic (self.topic)
        self.publish({"data": serializable_out})

class TransformedApriltagDetectorJSONEncoder(JSONEncoder):
    """JSON Encoder helper for Apriltag Detector transformed attributes."""

    def default(self, o):
        """JSON Encoder helper function for Apriltag Detector pose attributes."""
        if isinstance(o, (np.ndarray)):
            return list(o)

        return JSONEncoder.default(self, o)
