#! /usr/bin/env python
import roslib;
import rospy
import actionlib

from scripts import Script

from rose_script_manager.msg import *

class ScriptManagerServer:
	def __init__(self):
		self.script = Script("script_manager")
		
		self.server = actionlib.SimpleActionServer('script_manager', execute_scriptAction, self.goalReceived, False)
		self.server.register_preempt_callback(self.preemptRequested)
		self.server.start()

	def goalReceived(self, goal):
		self.execute(goal)

	def preemptRequested(self):
		self.script.cancel()
		
	def execute(self, goal):
		script_id 	= goal.script_id
		self.script.running()

		# Execute script
		if getattr(self.script, script_id)([]) :
			print "script-manager : script succeeded"
			self.server.set_succeeded()
		else:
			print "script-manager : script failed"
			self.server.set_aborted()

if __name__ == '__main__':
	rospy.init_node('script_manager')
	server = ScriptManagerServer()
	rospy.spin()