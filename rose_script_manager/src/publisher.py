#! /usr/bin/env python
import rospy
import actionlib

from rose_ui_item_collector.msg import *
from rose_operation_manager.msg import *
from roscomm.msg import *
from std_msgs.msg import String
from time import sleep

class Publisher:
	def __init__(self, node):
		self.node = node
		self.warning_pub = rospy.Publisher("/messages_window/warning", String, latch=True)
		self.action_pub = rospy.Publisher("/messages_window/action", String, latch=True)
		self.message_pub = rospy.Publisher("/messages_window/message", String, latch=True)

		self.current_action_pub = rospy.Publisher(node + "/current_action", String, latch=True)

		self.item_collector_client = actionlib.SimpleActionClient("item_collector", get_itemsAction)
		self.operation_manager_client = actionlib.SimpleActionClient("operation_manager", executeAction)
		self.received_parameters = []
		self.received_result = False

		self.item_collector_active = False
		self.operation_manager_active = False

	def cancel(self):
		if self.item_collector_active:
			self.item_collector_client.cancel_goal()

		if self.operation_manager_active:
			self.operation_manager_client.cancel_goal()

		message = str("Script cancelled")
		self.publishMessage(message)		

# --------------- CURRENT ACTION ------------------	
	def publishAction(self, action):
		# print "action publish"
		# self.action_pub.publish(String(action))
		print ""
	
	def publishMessage(self, action):
		# print "message publish"
		# self.message_pub.publish(String(action))	
		print ""

	def publishWarning(self, action):
		# print "warning publish"
		# self.warning_pub.publish(String(action))	
		print ""

	def getItemsForParameters(self, params, script):
		self.itemCollectorPublishGoal(params)

		message = str("Select parameters for script < " + script + " >")
		self.publishAction(message)

		self.item_collector_client.wait_for_result()
		
		params = self.received_parameters
		self.received_parameters = []

		print params

		return params

	def executeScript(self, script, item_ids):

		print "Received script < " + script + " > with parameters < " + ','.join(map(str,item_ids)) + " >"
		self.operationManagerPublishGoal(script, item_ids)

		message = str("Executing script < " + script + " > with parameters < " + ','.join(map(str,item_ids)) + " >")
		self.publishMessage(message)

		self.operation_manager_client.wait_for_result()

		if self.received_result:
			message = str("Script < " + script + " > finished successfully")
			self.publishMessage(message)
			self.received_result = False
			return True
		else:
			message = str("Script < " + script + " > failed")
			self.publishWarning(message)
			return False

# --------------- ITEM COLLECTOR ------------------

	def itemCollectorPublishGoal(self, params):
	
		self.item_collector_client.wait_for_server()
		
		goal = get_itemsActionGoal

		message = str("Sending list of options...")
		self.publishMessage(message) 	

		# Goal fields
		goal.names = []
		goal.types = []

		# Strip all parameters
		for param in params:
			items = param.split(":")
			goal.names = goal.names + [items[0].strip()]

			paramtypes = stringlist()
			types = items[1].split("|")

			for item_type in types:
				paramtypes.values = paramtypes.values + [item_type.strip()]

			goal.types = goal.types + [paramtypes]
		
		print "Sending goal"
	
		self.item_collector_client.send_goal(goal, 
			self.itemCollectorReceiveResult, 
			self.itemCollectorReceiveActive, 
			self.itemCollectorReceiveFeedback)
# 
	def itemCollectorReceiveResult(self, state, result):
		print "result received"
		self.received_parameters = []
		
		self.item_collector_active = False

		if ( state == actionlib.GoalStatus.SUCCEEDED ):

			for item_ids in result.item_ids:
					param = []
					for item in item_ids.values:
						param.append(item)

					self.received_parameters.append(param)

	def itemCollectorReceiveActive(self):
		print "active received"
		self.item_collector_active = True

	def itemCollectorReceiveFeedback(self, feedback):
		print "feedback received"

# ----------------- OEPRATION MANAGER -----------------------------

	def operationManagerPublishGoal(self, script_id, item_ids):

		goal = executeGoal
		
		goal.script_id = script_id
		goal.item_ids = []

		for item_id_list in item_ids:
			
			for item_id in item_id_list:
				item_list 			= stringlist()
				item_list.values 	= item_list.values + [item_id]
			
			goal.item_ids 	= goal.item_ids + [item_list]

		self.operation_manager_client.wait_for_server()

		self.received_result = False

		self.operation_manager_client.send_goal(goal, 
			self.operationManagerReceiveResult, 
			self.operationManagerReceiveActive, 
			self.operationManagerReceiveFeedback
		)

	def operationManagerReceiveResult(self, state, result):
		print "result"
		self.operation_manager_active = False

		if ( state == actionlib.GoalStatus.SUCCEEDED ):
			self.received_result = True

	def operationManagerReceiveActive(self):
		print "active"
		self.operation_manager_active = True

	def operationManagerReceiveFeedback(self, feedback):
		print "feedback"