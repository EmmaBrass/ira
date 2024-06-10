import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image
from cv_bridge import CvBridge, CvBridgeError

from std_msgs.msg import String
from std_msgs.msg import Int16MultiArray

import cv2, math, time, logging, pickle

from transitions import Machine
from face import Face
import random, time
import numpy as np
import face_recognition

# State machine for the interaction node - keeps track of what state the whole system is in.
# This is for IRA painting people's faces.

class InterationStateMachine():
    
    states = ['scanning', 'found_unknown', 'found_known', 'to_found_noone','say_painted_recently', 
              'too_far', 'interaction_unknown', 'interaction_known', 'interaction_known_recent', 
              'disappeared', 'gone', 'interaction_returned', 'painting', 'completed']
    
    transitions = [
        { 'trigger': 'to_scanning', 'source': 'scanning', 'dest': 'scanning'}, #done

        { 'trigger': 'to_found_unknown', 'source': 'scanning', 'dest': 'found_unknown'}, #done
        { 'trigger': 'to_found_known', 'source': 'scanning', 'dest': 'found_known'}, #done
        { 'trigger': 'to_found_noone', 'source': 'scanning', 'dest': 'found_noone'}, #done

        { 'trigger': 'to_scanning', 'source': 'found_noone', 'dest': 'scanning'}, #done

        { 'trigger': 'to_say_painted_recently', 'source': 'found_known', 'dest': 'say_painted_recently' }, #Done
        { 'trigger': 'to_interaction_known_recent', 'source': 'found_known', 'dest': 'interaction_known_recent' },
        { 'trigger': 'interaction_known', 'source': 'found_known', 'dest': 'interaction_known' }, #done
        { 'trigger': 'to_too_far', 'source': 'found_known', 'dest': 'too_far' }, #done
        { 'trigger': 'to_scanning', 'source': 'found_known', 'dest': 'scanning' }, #done

        { 'trigger': 'to_scanning', 'source': 'say_painted_recently', 'dest': 'scanning' }, #done

        { 'trigger': 'to_scanning', 'source': 'too_far', 'dest': 'scanning' }, #done

        { 'trigger': 'to_interaction_unknown', 'source': 'found_unknown', 'dest': 'interaction_unknown' }, #done
        { 'trigger': 'to_too_far', 'source': 'found_unknown', 'dest': 'too_far' }, #done
        { 'trigger': 'to_scanning', 'source': 'found_unknown', 'dest': 'scanning' }, #done

        { 'trigger': 'to_disappeared', 'source': 'interaction_unknown', 'dest': 'disappeared' }, #done
        { 'trigger': 'to_painting', 'source': 'interaction_unknown', 'dest': 'painting' }, #done

        { 'trigger': 'to_disappeared', 'source': 'interaction_known', 'dest': 'disappeared' }, #done
        { 'trigger': 'to_painting', 'source': 'interaction_known', 'dest': 'painting' }, #done

        { 'trigger': 'to_disappeared', 'source': 'interaction_known_recent', 'dest': 'disappeared' }, #done
        { 'trigger': 'to_painting', 'source': 'interaction_known_recent', 'dest': 'painting' }, #done

        { 'trigger': 'to_disappeared', 'source': 'disappeared', 'dest': 'disappeared' }, #done
        { 'trigger': 'to_interaction_returned', 'source': 'disappeared', 'dest': 'interaction_returned' }, #done
        { 'trigger': 'to_gone', 'source': 'disappeared', 'dest': 'gone' }, #done
        
        { 'trigger': 'to_scanning', 'source': 'gone', 'dest': 'scanning' }, #done

        { 'trigger': 'to_disappeared', 'source': 'interaction_returned', 'dest': 'disappeared' }, #done
        { 'trigger': 'to_painting', 'source': 'interaction_returned', 'dest': 'painting' }, #done

        { 'trigger': 'to_completed', 'source': 'painting', 'dest': 'completed' }, #done

        { 'trigger': 'to_scanning', 'source': 'completed', 'dest': 'scanning' }, #done
    ]

    def __init__(self):

        # Initialize the state machine
        self.machine = Machine(
            model=self, 
            states=InterationStateMachine.states, 
            transitions=InterationStateMachine.transitions, 
            initial='scanning'
        )
