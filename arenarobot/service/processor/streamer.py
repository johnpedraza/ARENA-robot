"""
streamer.py: streams object data into an ARENA scene

Created by John Pedraza on August 4th, 2022.

Copyright (c) 2022, The CONIX Research Center
All rights reserved.

This source code is licensed under the BSD-3-Clause license found in the
LICENSE file in the root directory of this source tree.
"""

from json import JSONEncoder, dumps, loads
from typing import List
from paho.mqtt.client import MQTTMessage
from arenarobot.service.processor import ArenaRobotServiceProcessor
from arena import Scene, Box

# pylint: disable=too-many-instance-attributes
class ArenaRobotServiceProcessorStreamer(ArenaRobotServiceProcessor):
    """Streamer processor class for the ARENA."""

    DEVICE_INSTANCE_PROCESSOR_TYPE = "streamer"

    # pylint: disable=too-many-arguments
    # pylint: disable=too-many-statements
    def __init__(self,
                 filter_topic = None,
                 **kwargs):
        """Initialize the streamer processor class."""

        processor_type = (ArenaRobotServiceProcessorStreamer
                          .DEVICE_INSTANCE_PROCESSOR_TYPE)

        self.filter_topic = filter_topic
        self.fetched_once = False

        super().__init__(
            device_instance_processor_type=processor_type,
            device_instance_prefix="streamer_",
            **kwargs
        )

    def setup(self):
        # make a box
        self.scene = Scene(host="mqtt.arenaxr.org", scene="origin_only")
        self.box = Box(object_id="myBox", position=(0,3,-3), scale=(0.05,0.05,0.05))

        # add the box to ARENA
        self.scene.add_object(self.box)

        super().setup()

    def fetch(self):
        """
        Publish pose information to MQTT topic:
            realm/d/<user>/<device>/processors/pose_optitrack
        """
        if self.fetched_once:
            print("Streamer processor should have interval of -1!")
            return
        self.fetched_once = True
        


        def stream_data(client, userdata, msg: MQTTMessage):
            payload = self.decode_payload(msg)
            position = payload['msg']['data']['pose']['position']
            rotation = payload['msg']['data']['pose']['rotation']
            
            if position and rotation:
                self.box.data.position.x = position[0]
                self.box.data.position.y = position[1]
                self.box.data.position.z = position[2]


            self.scene.update_object(self.box)

        if self.filter_topic:
            self.device.message_callback_add(self.filter_topic, stream_data)

class TransformedStreamerJSONEncoder(JSONEncoder):
    """JSON Encoder helper for Streamer transformed attributes."""

    def default(self, o):
        """JSON Encoder helper for Streamer pose attributes."""
        if isinstance(o, (np.ndarray)):
            return list(o)

        return JSONEncoder.default(self, o)