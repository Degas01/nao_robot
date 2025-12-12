#!/usr/bin/env python
# -*- coding: utf-8 -*-

import time
import os
from naoqi import ALProxy

# motion = ALProxy("ALMotion", "<robot-ip>", 9559)
# posture = ALProxy("ALRobotPosture", "<robot-ip>", 9559)

def process_command(command):
    command = command.strip().lower()
    print(command)
    

    if "stand" in command:
        # posture.goToPosture("Stand", 0.5)
        print("Robot standing")
    elif "sit" in command:
        # posture.goToPosture("Sit", 0.5)
        print("Robot sitting")
    elif "hello" in command:
        # motion.setAngles("RShoulderPitch", -1.0, 0.2)
        print("Robot waving")
    elif "kick the ball" in command:
        print ("Let's kick that ball!!")

def read_commands():
    while True:
        try:
            # Read and clear file automically
            with open("commands.txt", "r+") as f:
                lines = f.readlines()
                f.seek(0)
                f.truncate()
                
            # Process commands
            for line in lines:
                process_command(line)
                
        except IOError:
            open("commands.txt", "w").close()
        time.sleep(1)

if __name__ == "__main__":
    read_commands()