"""
optitrack.py: receives pose data from Motive and publishes to MQTT

Created by John Pedraza on July 11th, 2022.

Copyright (c) 2022, The CONIX Research Center
All rights reserved.

This source code is licensed under the BSD-3-Clause license found in the
LICENSE file in the root directory of this source tree.
"""

from json import JSONEncoder, dumps, loads
from typing import List

import numpy as np

from NatNetClient import NatNetClient
from arenarobot.service.processor import ArenaRobotServiceProcessor


# pylint: disable=too-many-instance-attributes
class ArenaRobotServiceProcessorOptitrack(ArenaRobotServiceProcessor):
    """Optitrack processor class for the ARENA."""

    DEVICE_INSTANCE_PROCESSOR_TYPE = "optitrack"

    # pylint: disable=too-many-arguments
    # pylint: disable=too-many-statements
    def __init__(self,
                 **kwargs):
        """Initialize the optitrack processor class."""

        processor_type = (ArenaRobotServiceProcessorOptitrack
                          .DEVICE_INSTANCE_PROCESSOR_TYPE)

        super().__init__(
            device_instance_processor_type=processor_type,
            device_instance_prefix="optitrack_",
            **kwargs
        )

    def setup(self):
        super().setup()

    def fetch(self):
        """
        Fetch data from Motive,
        Publish pose information to MQTT topic:
            realm/d/<user>/<device>/processors/pose_optitrack
        """
        
        def receiveRigidBodyFrame(id, position, rotation):
            global last_optitrack_time
            global last_optitrack_pose

            if (id == 39):
                pose = position
                out = {
                    "pose": pose,
                }
                print(out)
                serializable_out = loads(dumps(
                    out,
                    cls=TransformedOptitrackJSONEncoder
                ))

                # this publishes to subtopic (self.topic)
                self.publish({"data": serializable_out})

        streamingClient = NatNetClient()
        streamingClient.newFrameListener = None
        streamingClient.rigidBodyListener = receiveRigidBodyFrame
        streamingClient.run()

class TransformedOptitrackJSONEncoder(JSONEncoder):
    """JSON Encoder helper for Optitrack transformed attributes."""

    def default(self, o):
        """JSON Encoder helper for Optitrack pose attributes."""
        if isinstance(o, (np.ndarray)):
            return list(o)

        return JSONEncoder.default(self, o)