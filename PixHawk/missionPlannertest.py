from Antek import PixHawkService, MissionService
import time
from pymavlink import mavutil, mavextra
import time
from typing import Tuple, List, Dict, Optional
import numpy as np
import cv2
from math import radians

drone = PixHawkService()
mission = MissionService(drone=drone)

#pixel = mission.getPixel()
wps=[
{ "lat": 0, "lon": 0, "alt": 0},
{ "lat": 40.736072295748734, "lon": 30.07320899999999, "alt": 40},
{ "lat": 40.73608011659842, "lon": 30.07397666200842, "alt": 40},
{ "lat": 40.73608237027865, "lon": 30.07236471691425, "alt": 40},
{ "lat": 40.73570132609039, "lon": 30.0740379183394, "alt": 40}
]
if drone.set_waypoints(wps):
    print("wps setted correctly")
print(drone.sandbox())
"""
lat_uav, lon_uav = 40.736313, 30.073209        # [deg]
alt_uav = 100.0          # [m] nad ziemią
print("UAV:", lat_uav, lon_uav, "alt:", alt_uav)


roll  = radians(90)              # [rad]
pitch = radians(0)              # [rad]
yaw   = radians(0)

pixels= [(0,0), (0, 1295), (2303,0), (2303, 1295)]


for pixel in pixels:
    # rozdzielczość kamery
    
    width, height = 2304, 1296

    # intrinsic (z kalibracji)
    #K = np.array([
    #    [1.37057373e+03, 0.00000000e+00, 1.08538067e+03],
        #   [0.00000000e+00, 1.38363295e+03, 7.10619956e+02],
        #  [0.00000000e+00, 0.00000000e+00, 1.00000000e+00]
    #], dtype=np.float32)

    # obliczone przez chata
    K = np.array([
        [1773.9244383143994, 0.00000000e+00, 1152],
        [0.00000000e+00, 1733.1547280645818, 648],
        [0.00000000e+00, 0.00000000e+00, 1.00000000e+00]
    ], dtype=np.float32)

    dist = np.array([[-0.1359832, 0.36278877, 0.02114295, 0.02506856, -0.21990155]], dtype=np.float32)

    # ---- undistort piksel ----
    u,v = pixel
    undistorted = cv2.undistortPoints(
        np.array([[u, v]], dtype=np.float32),
        cameraMatrix=K,
        distCoeffs=dist
    )
    x_u, y_u = undistorted[0,0]
    # ---- promień w kamerze ----
    ray_cam = np.array([x_u, -y_u, 1.0])
    ray_cam /= np.linalg.norm(ray_cam)


    R_world_body = mission.rot_matrix(roll, pitch, yaw)
    ray_world = R_world_body @ ray_cam

    dz = ray_world[2]
    if abs(dz) < 1e-6 or dz <= 0:
        print("Ray is parallel to ground or above horizon, cannot compute intersection.")
    
    # ---- przecięcie z ziemią ----
    h_g = 0.0
    t = (alt_uav - h_g) / dz
    hit_local = ray_world * t   # [north, east, down]

    # ---- GPS offset ----
    lat_t, lon_t = mavextra.gps_offset(
        lat_uav, lon_uav,
        hit_local[0],  # north
        hit_local[1]   # east
    )
    
    mission.TARGETS.append((lat_t, lon_t))

aiming_Paths=[]
yaw = 90
for target in mission.TARGETS:
    path = mission.calc_drop_waypoints(target, yaw)
    aiming_wps = mission.prepare_wps(path, alt=40)
    yaw = -yaw


if not drone.append_waypoints(mission.Waypoints_to_sent):
    print("couldnt update mission")


#kontynujemy wykryanie celów
# ...
'''
yaw = 90 # tutaj ustawiamy z której strony nalatujemy nad cel, do ustalenia
aiming_Paths = []
for target in drone.TARGETS:
    path = mission.calc_drop_waypoints(target, yaw)
    aiming_wps = mission.prepare_wps(path, alt=40)
    aiming_Paths.append(path)
    yaw = -yaw
for new_wps in aiming_Paths:
    
    if not drone.append_waypoints(new_wps):
        print("couldnt update mission")

#TODO jak wysłać sygnał do szczerba o zrzuceniu rzutki?

"""