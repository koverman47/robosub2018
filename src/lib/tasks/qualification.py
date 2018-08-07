#!/usr/bin/env python

from task import Task


class Qualification(Task):


    def __init__(self, current, desired, color=None):
        Task.__init__(self, current, desired)
        self.subtasks.append('find', 'gothrough')
