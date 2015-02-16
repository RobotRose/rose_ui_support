#! /usr/bin/env python
import rospy
import actionlib

from publisher import *
from time import sleep

class Script:
	def __init__(self, node):
		self.publisher = Publisher(node)
		self.cancelled = True

	def cancel(self):
		self.publisher.cancel()
		self.cancelled = True

	def running(self):
		self.cancelled = False

	# ------------------- BASIC SCRIPTS ------------------------

	# input: plek : waypoints | items
	def move_to(self, params):
		scriptId = "move_to"
		print scriptId + "(plek : waypoints | items | persons)"

		# No parameters filled in: Ask itemCollector
		if not params:
			params = self.publisher.getItemsForParameters([ "plek : waypoints | items | persons" ], scriptId)

		if not params:
			return False
		# no correct result received
		for param in params:
			if not param:
				message = str("Script < " + scriptId + " > failed due to missing parameters")
				self.publisher.publishWarning(message)
				print "Missing parameters"
				return False

		if not self.cancelled:
			return self.publisher.executeScript(scriptId, params)
		else:
			return False
		
	# input: item : items
	def grab(self, params):
		scriptId = "grab"
		print scriptId + "(item : items)"
		
		# No parameters filled in: Ask itemCollector
		if not params:
			params = self.publisher.getItemsForParameters([ "item : items" ], scriptId)

		if not params:
			return False
			
		# no correct result received
		for param in params:
			if not param:
				message = str("Script < " + scriptId + " > failed due to missing parameters")
				self.publisher.publishWarning(message)	
				print "Missing parameters"
				return False

		if not self.cancelled:
			return self.publisher.executeScript(scriptId, params)
		else:
			return False

	def place(self, params):
		scriptId = "place"
		print scriptId + "(item : items)"
		
		# No parameters filled in: Ask itemCollector
		if not params:
			params = self.publisher.getItemsForParameters([ "item : items", "plek : waypoints" ], scriptId)

		if not params:
			return False
			
		# no correct result received
		for param in params:
			if not param:
				message = str("Script < " + scriptId + " > failed due to missing parameters")
				self.publisher.publishWarning(message)	
				print "Missing parameters"
				return False

		if not self.cancelled:
			return self.publisher.executeScript(scriptId, params)
		else:
			return False

	def strech(self, params):
		scriptId = "strech"
		
		if not params:
			params = self.publisher.getItemsForParameters([], scriptId)

		if not self.cancelled:
			return self.publisher.executeScript(scriptId, "")
		else:
			return False			

	def say(self, params):
		scriptId = "say"

		if not params:
			self.publisher.getItemsForParameters([], scriptId)

		if not params:
			params = [["resource0"]] # welcome to Rose BV

		if not self.cancelled:
			return self.publisher.executeScript(scriptId, params)
		else:
			return False

	def give(self, params):
		scriptId = "give"
		
		if not params:
			params = self.publisher.getItemsForParameters([], scriptId)

		if not self.cancelled:
			return self.publisher.executeScript(scriptId, "")
		else:
			return False

	def handover_to_rose(self, params):
		scriptId = "handover_to_rose"
		
		if not params:
			params = self.publisher.getItemsForParameters([], scriptId)

		if not self.cancelled:
			return self.publisher.executeScript(scriptId, "")
		else:
			return False

	def arm_driving_position(self, params):
		scriptId = "arm_driving_position"
		
		if not params:
			params = self.publisher.getItemsForParameters([], scriptId)

		if not self.cancelled:
			return self.publisher.executeScript(scriptId, "")
		else:
			return False

	def arm_grabbing_position(self, params):
		scriptId = "arm_grabbing_position"
		
		if not params:
			params = self.publisher.getItemsForParameters([], scriptId)

		if not self.cancelled:
			return self.publisher.executeScript(scriptId, "")
		else:
			return False

	def open_gripper(self, params):
		scriptId = "open_gripper"

		if not params:
			params = self.publisher.getItemsForParameters([], scriptId)

		if not self.cancelled:
			return self.publisher.executeScript(scriptId, "")
		else:
			return False

	def close_gripper(self, params):
		scriptId = "close_gripper"

		if not params:
			params = self.publisher.getItemsForParameters([], scriptId)

		if not self.cancelled:
			return self.publisher.executeScript(scriptId, "")
		else:
			return False

	def position_determinator(self, params):
		scriptId = "position_determinator"
		print "position_determinator with params " + str(params)

		if not params:
			return False

		if not self.cancelled:
			return self.publisher.executeScript(scriptId, params)
		else:
			return False

	# ------------------- COMPLEX SCRIPTS ------------------------
	def hand_over_item(self,params):
		scriptId = "hand_over_item"

		if not params:
			params = self.publisher.getItemsForParameters([], scriptId)
		
		if not self.say([["resource1"]]): # flesje los
			return False

		sleep(2.5)

		if not self.open_gripper("open"): 
			return False

		if not self.say([["resource2"]]): # astublieft
			return False

		if not self.arm_driving_position("dummy"):
			return False

		message = str("Script < " + scriptId + " > finished successfully")
		self.publisher.publishMessage(message)			

		return True

	def hand_over_item_complete(self,params):
		scriptId = "hand_over_item"

		if not params:
			params = self.publisher.getItemsForParameters([], scriptId)
		
		if not self.give("dummy"):
			return False

		if not self.say([["resource1"]]): # flesje los
			return False

		sleep(2.5)

		if not self.open_gripper("open"): 
			return False

		if not self.say([["resource2"]]): # astublieft
			return False

		if not self.arm_driving_position("dummy"):
			return False

		if not self.close_gripper("close"): 
			return False			

		message = str("Script < " + scriptId + " > finished successfully")
		self.publisher.publishMessage(message)			

		return True		

	def move_and_grab(self, params):
		scriptId = "move_and_grab"

		# No parameters filled in: Ask itemCollector
		if not params:
			params = self.publisher.getItemsForParameters([ "Items : items"], scriptId)

		# no correct result received
		if not params :
			return False
		
		if not self.move_to([params[0]]): # Ontbijt plek : waypoints
			return False

		if not self.grab([params[0]]): # Ontbijtspullen : items
			return False

		message = str("Script < " + scriptId + " > finished successfully")
		self.publisher.publishMessage(message)			

		return True

	def breakfast(self, params):
		scriptId = "breakfast"
		print scriptId + "(Ontbijtspullen : items, Ontbijt plek : waypoints)"

		# No parameters filled in: Ask itemCollector
		if not params:
			params = self.publisher.getItemsForParameters([ "Ontbijtspullen : items", "Ontbijt plek : waypoints | persons" ], scriptId)

		# no correct result received
		if not params :
			return False
		
		if not self.grab([params[0]]): # Ontbijtspullen : items
			return False

		if not self.move_to([params[1]]): # Ontbijt plek : waypoints
			return False

		message = str("Script < " + scriptId + " > finished successfully")
		self.publisher.publishMessage(message)			

		return True

	#test
	def longfunc(self, params):
		scriptId = "longfunc"
		print scriptId + "(stuff : items | waypoints | persons)"

		# No parameters filled in: Ask itemCollector
		if not params:
			params = self.publisher.getItemsForParameters([ "stuff : items | waypoints | persons" ], scriptId)

		# no correct result received
		if not params :
			return False

		i = 0

		while i < 10000:
			if not self.grab([params[0]]): # stuff
				return False

			if not self.move_to([params[0]]): # stuff
				return False

			i = i + 1

		message = str("Script < " + scriptId + " > finished successfully")
		self.publisher.publishMessage(message)			
		return True

	def welcome(self, params):
		scriptId = "welcome"

		print scriptId
		
		if not self.move_to([["waypoint2"]]): # Directiekamer
			return False

		if not self.say([[]]): # stuff
			return False

		if not self.move_to([["waypoint1"]]): # Base
			return False


		message = str("Script < " + scriptId + " > finished successfully")
		self.publisher.publishMessage(message)			
		return True

	def grab_demo(self, params):
		scriptId = "grab_demo"

		if not params:
			params = self.publisher.getItemsForParameters([], scriptId)

		print scriptId

		self.arm_driving_position("dummy")

		if not self.move_to([["waypoint3"]]): # Turn from table so we have place to stretch the arms
			return False
		
		self.strech("dummy")
		
		if not self.move_to([["waypoint5"]]): # Turn over the table with arms stretched
			return False

		if not self.grab([["item23"]]):
                        return False

		if not self.move_to([["waypoint3"]]): # Turn from table so we have place to stretch the arms
			return False
		
		self.arm_driving_position("dummy")
		return True

	def place_demo(self, params):
		scriptId = "place_demo"

		if not params:
			params = self.publisher.getItemsForParameters([], scriptId)

		if not self.move_to([["waypoint6"]]): # Turn from table so we have place to stretch the arms
			return False
		
 		self.strech("dummy")
		
		if not self.move_to([["waypoint7"]]):
			return False

		if not self.place([["item23"], ["waypoint1"]]):
                        return False
		
		if not self.move_to([["waypoint8"]]):
			return False

		self.arm_driving_position("dummy")

		return True

	def grab_and_place_demo(self, params):		
		scriptId = "grab_and_place_demo"

		if not params:
			params = self.publisher.getItemsForParameters([], scriptId)
		
		if not self.grab_demo(params):
			return False

		if not self.place_demo(params):
			return False

		if not self.move_to([["waypoint4"]]): #Rose home
			return False

		return True

	def grab_complex(self, params):		
		scriptId = "grab_complex"

		if not params:
			params = self.publisher.getItemsForParameters(["item : items"], scriptId)

		if not params:
			return False

		pos_det_params = params 
		pos_det_params += [["grab"]]+ [["waypoint10"]] #Waypoint10 is reserved waypoint

		if not self.position_determinator(pos_det_params):
			return False

		if not self.arm_grabbing_position("dummy"):
			return False

		if not self.move_to([["waypoint10"]]): 
			return False

		if not self.grab(params):
			return False

		return True

	def place_complex(self, params):		
		scriptId = "place_complex"

		if not params:
			params = self.publisher.getItemsForParameters(["item : items", "plek : waypoints"], scriptId)

		if not params:
			return False

		pos_det_params = params 
		pos_det_params = [params[1]] + [["place"]] + [["waypoint10"]]  #item10 is reserved item

		if not self.position_determinator(pos_det_params):
			return False

		if not self.arm_grabbing_position("dummy"):
			return False

		if not self.move_to([["waypoint10"]]): #waypoint 
			return False

		if not self.place(params):
			return False

		return True

	def navigate_to(self, params):
		scriptId = "navigate_to"

		if not params:
			params = self.publisher.getItemsForParameters(["plek : waypoints | items | persons"], scriptId)

		if not params:
			return False

		if not self.arm_driving_position("dummy"):
			return False

		if not self.move_to(params): #waypoint 
			return False

		return True