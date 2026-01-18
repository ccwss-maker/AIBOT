#!/usr/bin/env python3

"""Interactive utility that publishes elevator docking commands to /goal_v3."""

import argparse

import rospy
from geometry_msgs.msg import PoseStamped
from robot_v3.msg import Goal_v3


def build_goal(command, args):
	goal = Goal_v3()
	goal.pose = PoseStamped()
	goal.pose.header.frame_id = args.frame_id
	goal.floor = args.floor
	goal.house = args.house
	goal.relocation = False
	goal.stop = False
	goal.planner = args.planner
	goal.controller = args.controller
	goal.elevator_docking = command
	return goal


def parse_cli():
	parser = argparse.ArgumentParser(description="Interactive elevator docking Goal_v3 publisher")
	parser.add_argument("--house", default="ntuitive_align",
						help="House/map namespace to include in Goal_v3")
	parser.add_argument("--floor", default="2m",
						help="Floor identifier for the goal message")
	parser.add_argument("--planner", default="",
						help="Optional planner override string")
	parser.add_argument("--controller", default="",
						help="Optional controller override string")
	parser.add_argument("--frame-id", dest="frame_id", default="map",
						help="Frame used for the placeholder pose header")
	return parser.parse_args(rospy.myargv()[1:])


def main():
	rospy.init_node('elevator_docking_pub_simulator', anonymous=True)
	args = parse_cli()

	pub = rospy.Publisher('/goal_v3', Goal_v3, queue_size=1)
	rospy.sleep(0.5)

	valid_commands = {"detection", "in", "out"}
	prompt = "Enter command (detection/in/out) or 'quit': "

	rospy.loginfo("Interactive elevator docking publisher ready. Type commands when prompted.")

	while not rospy.is_shutdown():
		try:
			user_input = input(prompt).strip()
		except (EOFError, KeyboardInterrupt):
			rospy.loginfo("Input interrupted, shutting down publisher")
			break

		if not user_input:
			continue

		lowered = user_input.lower()
		if lowered in {"quit", "exit"}:
			rospy.loginfo("Exit command received, stopping publisher")
			break

		if lowered not in valid_commands:
			rospy.logwarn("Unsupported command '%s'. Use detection/in/out or quit.", user_input)
			continue

		msg = build_goal(lowered, args)
		msg.pose.header.stamp = rospy.Time.now()
		pub.publish(msg)
		rospy.loginfo("Published Goal_v3 with elevator_docking=%s", lowered)


if __name__ == '__main__':
	try:
		main()
	except rospy.ROSInterruptException:
		pass
