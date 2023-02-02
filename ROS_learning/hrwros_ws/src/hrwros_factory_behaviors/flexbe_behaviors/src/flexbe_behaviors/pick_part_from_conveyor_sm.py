#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from hrwros_factory_states.srdf_state_to_moveit import SrdfStateToMoveit as hrwros_factory_states__SrdfStateToMoveit
from hrwros_factory_states.compute_grasp_state import ComputeGraspState
from hrwros_factory_states.vacuum_gripper_control_state import VacuumGripperControlState
from hrwros_factory_states.moveit_to_joints_dyn_state import MoveitToJointsDynState as hrwros_factory_states__MoveitToJointsDynState
from hrwros_factory_states.control_feeder_state import ControlFeederState
from hrwros_factory_states.detect_part_camera_state import DetectPartCameraState
from hrwros_factory_states.set_conveyor_power_state import SetConveyorPowerState
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Tue Aug 07 2018
@author: Carlos Hernandez
'''
class PickpartfromconveyorSM(Behavior):
	'''
	This is a behavior for the hrwros factory simulation that picks a part from the conveyor belt using robot1
	'''


	def __init__(self):
		super(PickpartfromconveyorSM, self).__init__()
		self.name = 'Pick part from conveyor'

		# parameters of this behavior

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]

		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		pick_group = 'robot1'
		names1 = ['robot1_shoulder_pan_joint', 'robot1_shoulder_lift_joint', 'robot1_elbow_joint', 'robot1_wrist_1_joint', 'robot1_wrist_2_joint', 'robot1_wrist_3_joint']
		gripper1 = "vacuum_gripper1_suction_cup"
		# x:1010 y:559, x:40 y:517
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])
		_state_machine.userdata.part_pose = []
		_state_machine.userdata.place_configuration = []
		_state_machine.userdata.conveyor_speed = 100
		_state_machine.userdata.joint_names = []
		_state_machine.userdata.pick_configuration = []

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]

		# [/MANUAL_CREATE]


		with _state_machine:
			# x:73 y:34
			OperatableStateMachine.add('Move Robot 1 Home',
										hrwros_factory_states__SrdfStateToMoveit(config_name='R1Home', move_group=pick_group, action_topic='/move_group', robot_name=""),
										transitions={'reached': 'Stars Conveyor Belt', 'planning_failed': 'failed', 'control_failed': 'failed', 'param_error': 'failed'},
										autonomy={'reached': Autonomy.Off, 'planning_failed': Autonomy.Off, 'control_failed': Autonomy.Off, 'param_error': Autonomy.Off},
										remapping={'config_name': 'config_name', 'move_group': 'move_group', 'robot_name': 'robot_name', 'action_topic': 'action_topic', 'joint_values': 'joint_values', 'joint_names': 'joint_names'})

			# x:673 y:227
			OperatableStateMachine.add('Compute pick',
										ComputeGraspState(group=pick_group, offset=0.0, joint_names=names1, tool_link=gripper1, rotation=3.1415),
										transitions={'continue': 'Activate gripper', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'pose': 'part_pose', 'joint_values': 'pick_configuration', 'joint_names': 'joint_names'})

			# x:661 y:330
			OperatableStateMachine.add('Activate gripper',
										VacuumGripperControlState(enable=True, service_name='/gripper1/control'),
										transitions={'continue': 'Move to pick', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off})

			# x:668 y:439
			OperatableStateMachine.add('Move to pick',
										hrwros_factory_states__MoveitToJointsDynState(move_group=pick_group, offset=0.0, tool_link=gripper1, action_topic='/move_group'),
										transitions={'reached': 'Move Robot 1 Home_2', 'planning_failed': 'failed', 'control_failed': 'failed'},
										autonomy={'reached': Autonomy.Off, 'planning_failed': Autonomy.Off, 'control_failed': Autonomy.Off},
										remapping={'joint_values': 'pick_configuration', 'joint_names': 'joint_names'})

			# x:668 y:554
			OperatableStateMachine.add('Move Robot 1 Home_2',
										hrwros_factory_states__SrdfStateToMoveit(config_name='R1Home', move_group=pick_group, action_topic='/move_group', robot_name=""),
										transitions={'reached': 'finished', 'planning_failed': 'failed', 'control_failed': 'failed', 'param_error': 'failed'},
										autonomy={'reached': Autonomy.Off, 'planning_failed': Autonomy.Off, 'control_failed': Autonomy.Off, 'param_error': Autonomy.Off},
										remapping={'config_name': 'config_name', 'move_group': 'move_group', 'robot_name': 'robot_name', 'action_topic': 'action_topic', 'joint_values': 'place_configuration', 'joint_names': 'joint_names'})

			# x:507 y:36
			OperatableStateMachine.add('Feed part',
										ControlFeederState(activation=True),
										transitions={'succeeded': 'Locate part', 'failed': 'failed'},
										autonomy={'succeeded': Autonomy.Off, 'failed': Autonomy.Off})

			# x:701 y:31
			OperatableStateMachine.add('Locate part',
										DetectPartCameraState(ref_frame='robot1_base', camera_topic='/hrwros/logical_camera_1', camera_frame='logical_camera_1_frame'),
										transitions={'continue': 'Stops Conveyor Belt', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'pose': 'part_pose'})

			# x:272 y:31
			OperatableStateMachine.add('Deactivate Gripper',
										VacuumGripperControlState(enable=False, service_name='/gripper1/control'),
										transitions={'continue': 'Feed part', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off})

			# x:93 y:111
			OperatableStateMachine.add('Stars Conveyor Belt',
										SetConveyorPowerState(stop=False),
										transitions={'succeeded': 'Deactivate Gripper', 'failed': 'failed'},
										autonomy={'succeeded': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'speed': 'conveyor_speed'})

			# x:665 y:136
			OperatableStateMachine.add('Stops Conveyor Belt',
										SetConveyorPowerState(stop=True),
										transitions={'succeeded': 'Compute pick', 'failed': 'failed'},
										autonomy={'succeeded': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'speed': 'conveyor_speed'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]

	# [/MANUAL_FUNC]
