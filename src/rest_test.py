#!/usr/bin/env python
import time
import json
import argparse
import requests


if __name__ == "__main__":
    client_parser = argparse.ArgumentParser(description="REST Client")
    client_parser.add_argument(
        "port_number", nargs="?", default="7201", type=int, help="port number"
    )
    client_parser.add_argument(
        "ip_address",
        nargs="?",
        default="127.0.0.1",
        help="ip_address for server",
    )
    margs = client_parser.parse_args()
    port_number = margs.port_number
    ip_address = margs.ip_address
    url = "http://" + ip_address + ":" + str(port_number) + "/api/robot/status"
    print("Requesting from " + url)