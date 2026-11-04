#!/usr/bin/python3
"""Existing receiver plus hand topic, one UDP socket; original ARM message preserved."""
import json
from pathlib import Path
import sys
ROOT=Path(__file__).resolve().parent

def main(args=None):
    from rm_teleop import pico_receiver_node as original
    from std_msgs.msg import String
    from rclpy.qos import qos_profile_sensor_data
    from pico_o7.o7_stage.hand_input import ForwardCore
    class HandReceiver(original.PicoReceiverNode):
        def __init__(self):
            super().__init__()
            self.hand_pub=self.create_publisher(String,'/pico/right_hand_raw',qos_profile_sensor_data)
            self.core=ForwardCore(self.core,lambda e:self.hand_pub.publish(String(data=json.dumps(e,allow_nan=False))))
    original.PicoReceiverNode=HandReceiver
    original.main(args=args)

if __name__=='__main__':main()
