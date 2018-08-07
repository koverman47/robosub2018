#!/usr/bin/env python

import json, math, numpy, rospy
from tasks import *
from robosub2018.msg import State, Detection, Arm

class Mission():

    def __init__(self, initial_mission = 'prequalification'):
        self.queue = [] # Priority Queue - Tasks to complete after current
        self.current = initial_mission # Active Task - Want the object
        self.completed = []
        
        tasks_file = open('tasks.json')
        tasks_string = tasks_file.read()
        self.tasks = json.loads(tasks_string)

    def get_next(self):
        self.completed.append(self.current.priority)
        self.current = heappop(self.queue)


    

