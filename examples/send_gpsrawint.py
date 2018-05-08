#!/usr/bin/python

# This file is part of the MAVLink Router project
#
# Copyright (C) 2017  Intel Corporation. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from __future__ import print_function
from __future__ import print_function
from threading import Thread
from time import sleep
from math import pi

import pymavlink.mavutil as mavutil
import sys
import time
import math

if len(sys.argv) != 4:
    print("Usage: %s <ip:udp_port> <system-id> <target-system-id>" %
          (sys.argv[0]))
    print(
        "Send mavlink pings, using given <system-id> and <target-system-id>, "
        "to specified interface")
    quit()

mav = mavutil.mavlink_connection(
    'udpout:' + sys.argv[1], source_system=int(sys.argv[2]))


def sendloop():
    radius = 0.002 # radius[m] of circular trajectory
    omega = 1.0 # rotational velocity [rad/s]

    i = 0 # sequence
    theta = 0
    trajcenter_lon = 8.5443524
    trajcenter_lat = 47.3982087

    mav_alt = 30 * 1000
    mav_fix_type = 3
    mav_eph = 65535
    mav_epv = 65535


    while (True):

        mav_v = radius * omega * 100
        mav_cog = (theta + 0.5 * pi) * 1E2
        mav_lat = (radius * math.cos(theta) + trajcenter_lat) * 1E7
        mav_lon = (radius * math.sin(theta) + trajcenter_lon) * 1E7
        mav_sat = 10

        mav.mav.gps_raw_int_send(time.time() * 1000000, mav_fix_type, mav_lat, mav_lon, mav_alt, mav_eph, mav_epv, mav_v, mav_cog, mav_sat)
        # mav.mav.scaled_imu_send(time.time() * 1000000, 0.001, 0.001, 0.001, 0.001, 0.001, 0.001, 0.001, 0.001, 0.001)
        mav.mav.ping_send(int(time.time() * 1000), i, int(sys.argv[3]), 1)

        i = i + 1
        theta = omega * i * 0.01

        sleep(1)

sendthread = Thread(target=sendloop)
sendthread.daemon = True
sendthread.start()

while (True):
    msg = mav.recv_match(blocking=True)
    print("Message from %d: %s" % (msg.get_srcSystem(), msg))