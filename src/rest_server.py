#!/usr/bin/env python

# ROS
import argparse
import rospy
from actionlib_msgs.msg import GoalStatusArray

# Flask
from flask import Flask
from flask_restful import Resource, Api

# ROS
rospy.init_node("ros_rest_server", disable_signals=True)
rospy.Subscriber(
    "/move_base/status", GoalStatusArray, move_base_status_callback
)

# Flask
app = Flask("REST Server")
api = Api(app)
api.add_resource(MoveBaseStatus, "/api/robot/status")

if __name__ == "__main__":
    server_parser = argparse.ArgumentParser(description="REST Server")
    server_parser.add_argument(
        "port_number", nargs="?", default="7201", type=int, help="port number"
    )
    server_parser.add_argument(
        "ip_address",
        nargs="?",
        default="127.0.0.1",
        help="ip_address for server",
    )
    margs = server_parser.parse_args(rospy.myargv()[1:])
    port_number = margs.port_number
    ip_address = margs.ip_address
    app.run(ip_address, port=port_number)