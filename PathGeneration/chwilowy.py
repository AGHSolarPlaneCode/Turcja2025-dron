import numpy as np
import math

def count_distance_from_left_pole(left_pole, left_top_corner, left_bottom_corner) -> tuple:
    left_pole_vector = np.array([left_pole['lat'], left_pole['lon']])
    left_top_vector = np.array([left_top_corner['lat'], left_top_corner['lon']])
    left_bottom_vector = np.array([left_bottom_corner['lat'], left_bottom_corner['lon']])

    top_distance = np.linalg.norm(left_pole_vector - left_top_vector)
    bottom_distance = np.linalg.norm(left_pole_vector - left_bottom_vector)

    return top_distance, bottom_distance

def count_distance_from_right_pole(right_pole, right_top_corner, right_bottom_corner) -> tuple:
    right_pole_vector = np.array([right_pole['lat'], right_pole['lon']])
    right_top_vector = np.array([right_top_corner['lat'], right_top_corner['lon']])
    right_bottom_vector = np.array([right_bottom_corner['lat'], right_bottom_corner['lon']])

    top_distance = np.linalg.norm(right_pole_vector - right_top_vector)
    bottom_distance = np.linalg.norm(right_pole_vector - right_bottom_vector)

    return top_distance, bottom_distance

def offset_latlon(lat, lon, north_m=0.0, east_m=0.0):
    """Przesuwa punkt o north_m, east_m w metrach."""
    dlat = north_m / 111320.0
    dlon = east_m / (111320.0 * math.cos(math.radians(lat)))
    return {"lat": lat + dlat, "lon": lon + dlon, "alt": 20}


def first_lap_path(left_top, right_top, right_bottom, left_bottom) -> list:
    """Zwraca 8 punktów trasy: 4 rogi (50 m od rogu) i 4 środki boków (50 m od środka)."""
    path = []

    path.append(offset_latlon(left_top["lat"], left_top["lon"], 50, 50))
    path.append(offset_latlon(right_top["lat"], right_top["lon"], 50, -50))
    path.append(offset_latlon(right_bottom["lat"], right_bottom["lon"], -50, -50))
    path.append(offset_latlon(left_bottom["lat"], left_bottom["lon"], -50, 50))

    top_mid = {
        "lat": (left_top["lat"] + right_top["lat"]) / 2,
        "lon": (left_top["lon"] + right_top["lon"]) / 2,
        "alt": 20,
    }
    path.append(offset_latlon(top_mid["lat"], top_mid["lon"], 50, 0))

    right_mid = {
        "lat": (right_top["lat"] + right_bottom["lat"]) / 2,
        "lon": (right_top["lon"] + right_bottom["lon"]) / 2,
        "alt": 20,
    }
    path.append(offset_latlon(right_mid["lat"], right_mid["lon"], 0, -50))

    bottom_mid = {
        "lat": (left_bottom["lat"] + right_bottom["lat"]) / 2,
        "lon": (left_bottom["lon"] + right_bottom["lon"]) / 2,
        "alt": 20,
    }
    path.append(offset_latlon(bottom_mid["lat"], bottom_mid["lon"], -50, 0))

    left_mid = {
        "lat": (left_top["lat"] + left_bottom["lat"]) / 2,
        "lon": (left_top["lon"] + left_bottom["lon"]) / 2,
        "alt": 20,
    }
    path.append(offset_latlon(left_mid["lat"], left_mid["lon"], 0, 50))

    return path

def generate_inside_path(old_waypoints: list, reference_poles: list, reference_poles_distance: list) -> list:
    camera_half_width = 19  # meters

    left_top_corner = old_waypoints[0]
    right_top_corner = old_waypoints[1]
    right_bottom_corner = old_waypoints[2]
    left_bottom_corner = old_waypoints[3]

    left 

    

    
def main():
    waypoints = [
        {"lat": -35.362138, "lon": 149.163302, "alt": 20},
        {"lat": -35.362138, "lon": 149.167158, "alt": 20},
        {"lat": -35.364384, "lon": 149.167158, "alt": 20},
        {"lat": -35.364384, "lon": 149.163302, "alt": 20}
    ]
    reference_poles = [
  {"lat": -35.362363, "lon": 149.165230, "alt": 20},
  {"lat": -35.364159, "lon": 149.165230, "alt": 20}
    ]   








