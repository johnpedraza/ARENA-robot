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
from arena import Scene, Box, Scale, Object, Text, GLTF, TextInput
import numpy as np
import time
import board
import adafruit_scd4x
import smbus

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
        
        # make a box
        self.scene = Scene(host="mqtt.arenaxr.org", scene="origin_only")
        self.box = Box(object_id="myBox", position=(0,3,-3), scale=(0.05,0.05,0.05))
        
        # initialize some variables
        self.hvac_hud_switch = False
        self.hvac_stats = None
        # self.plug_status, self.plug_stats = get_smartplug_stats()
        self.plug_hud_switch = False

        self.vav_stat_button = None
        self.stat_panel = None
        self.stat_text = None
        self.airflow_max = None
        self.plug_stat_panel = None
        self.plug_stat_text = None

        self.last_activitated = 0

        super().__init__(
            device_instance_processor_type=processor_type,
            device_instance_prefix="streamer_",
            **kwargs
            )

    def setup(self):
        # add the box to ARENA
        self.scene.add_object(self.box)
        
        self.vav_stat_button = Box(
            object_id="HUD_button", 
            position=(-0.834, 2.7, -1.153),
            scale=Scale(.15,.05,.15), 
            clickable=True, 
            evt_handler=stat_panel_trigger,
            color="#FF0000",
            persist=True)

        self.stat_panel = Object(**{
            "object_id": "stat_panel",
            "persist": True,
            "data": {
                "panel": {"height": 1.4, "width": 1.1, "depth":0.05},
                "position": (-0.834, 1.814, -1.153),
                "rotation": (0, 0, 0),
                "look-at": "#my-camera",
                "parent": "myBox",
                "visible": False
            }
        })
        
        self.stat_text = Text(**{
            "object_id":"stat_text","persist":True,"type":"object","action":"update",
            "data":
                {"object_type":"text",
                "position":{"x":0.135,"y":0.67,"z":0},"rotation":{"x":0,"y":0,"z":0},
                "font":"roboto","side":"front","text":self.hvac_stats,"width":1.3,"color":"#ffffff",
                "parent":"stat_panel",
                "baseline":"top","align":"left","xOffset":0.03,"zOffset":0.03}
            })


        self.plug_stat_panel = Object(**{
                "object_id": "plug_stat_panel",
                "persist": True,
                "data": {
                    "panel": {"height": 0.45, "width": 1, "depth":0.05},
                    "position": (0, 0.5, 0),
                    "rotation": (0, 0, 0),
                    "look-at": "#my-camera",
                    "visible": False,
                    "parent": "water_cooler_button"
                }
            })
        
        self.plug_stat_text = Text(**{
            "object_id":"plug_stat_text","persist":True,"type":"object","action":"update",
            "data":
                {"object_type":"text",
                "position":{"x":0.200,"y":0.18,"z":0},"rotation":{"x":0,"y":0,"z":0},
                "font":"roboto","side":"front","text":self.plug_stats,"width":1.3, "color":"#ffffff",
                "parent":"plug_stat_panel",
                "baseline":"top","align":"left","xOffset":0.03,"zOffset":0.03}
            })


        self.scene.add_object(self.vav_stat_button)
        self.scene.add_object(self.stat_panel)
        self.scene.add_object(self.stat_text)
        self.scene.add_object(self.airflow_input)
        self.scene.add_object(self.airflow_revert)
        self.scene.add_object(self.temp_input)
        self.scene.add_object(self.temp_revert)

        self.scene.add_object(self.water_cooler_button)
        self.scene.add_object(self.plug_stat_panel)
        self.scene.add_object(self.plug_stat_text)
        self.scene.add_object(self.plug_on_button)
        self.scene.add_object(self.plug_off_button)


        def stat_panel_trigger(scene, evt, msg):
            ## Get Event type
            if evt.type  == "mousedown":
                self.hvac_hud_switch = not self.hvac_hud_switch
                print(self.hvac_hud_switch)
                self.stat_panel.data.visible=self.hvac_hud_switch
                scene.update_object(self.stat_panel)
                # scene.update_object(airflow_max, **{"click-listener": False})
                
        # Airspeed
        self.Bus = smbus.SMBus(1)
        
        # Environmental Sensors
        self.i2c = board.I2C()
        self.scd4x = adafruit_scd4x.SCD4X(self.i2c)
        print("Serial number:", [hex(i) for i in self.scd4x.serial_number])

        self.scd4x.start_periodic_measurement()
        print("Waiting for first measurement....")

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
            
            self.hvac_stats = ''
            
            # Airspeed 
            try:
                data = self.Bus.read_i2c_block_data(0x28, 0x01, 5)
                data_high, data_low = data[1], data[2]

                data_high &= 7

                result = (data_high << 8) | data_low
                
                self.hvac_stats += result
                
                print(result)
            except:
                print('I/O Error')
            
            # Environmental Sensors
            if self.scd4x.data_ready:
                print("CO2: %d ppm" % self.scd4x.CO2)
                print("Temperature: %0.1f *C" % self.scd4x.temperature)
                print("Humidity: %0.1f %%" % self.scd4x.relative_humidity)
                print()

        if self.filter_topic:
            self.device.message_callback_add(self.filter_topic, stream_data)

class TransformedStreamerJSONEncoder(JSONEncoder):
    """JSON Encoder helper for Streamer transformed attributes."""

    def default(self, o):
        """JSON Encoder helper for Streamer pose attributes."""
        if isinstance(o, (np.ndarray)):
            return list(o)

        return JSONEncoder.default(self, o)