import numpy as np
import math
import matplotlib.pyplot as plt
import random

def plot_coordinates(waypoints, reference_poles, first_lap, inside_path):
    """Rysuje rogi, reference poles, first lap i inside path w różnych kolorach"""
    plt.figure(figsize=(8,6))

    # znajdz lewy dolny rog jako punkt odniesienia
    min_lat = min(p['lat'] for p in waypoints + reference_poles + first_lap + inside_path)
    min_lon = min(p['lon'] for p in waypoints + reference_poles + first_lap + inside_path)

    def to_xy(points):
        """Konwertuje listę punktów lat/lon na x/y w metrach względem lewego dolnego rogu do narysowania"""
        x_m = [(p['lon'] - min_lon) * 111320 * math.cos(math.radians(min_lat)) for p in points]
        y_m = [(p['lat'] - min_lat) * 111320 for p in points]
        return x_m, y_m

    # waypoints + first lap – niebieski
    x, y = to_xy(waypoints + first_lap)
    plt.scatter(x, y, color='blue', label='Waypoints + First Lap')

    # reference poles – zielony
    x, y = to_xy(reference_poles)
    plt.scatter(x, y, color='green', label='Reference Poles')

    # inside path – pomarańczowy
    x, y = to_xy(inside_path)
    plt.scatter(x, y, color='orange', label='Inside Path')

    plt.xlabel("Easting (m)")
    plt.ylabel("Northing (m)")
    plt.title("Coordinates plotted in meters")
    plt.grid(True)
    plt.axis('equal')
    plt.legend()
    plt.show()

def offset_latlon(lat, lon, north_m, east_m):
    """
    Zwraca nowe współrzędne lat/lon przesunięte o north_m na północ i east_m na wschód od (lat, lon).
    """
    # promień Ziemi w metrach
    R = 6378137
    dLat = north_m / R
    dLon = east_m / (R * math.cos(math.pi * lat / 180))

    new_lat = lat + dLat * 180 / math.pi
    new_lon = lon + dLon * 180 / math.pi
    return {"lat": new_lat, "lon": new_lon}

def first_lap_path(left_top, right_top, right_bottom, left_bottom):
    """8 punktów: 4 rogi przesunięte po przekątnej do środka prostokąta (50 m), 4 środki boków 50 m w kierunku środka boku"""
    path = []

    # Obliczamy środek prostokąta
    center_lat = (left_top["lat"] + right_top["lat"] + right_bottom["lat"] + left_bottom["lat"]) / 4
    center_lon = (left_top["lon"] + right_top["lon"] + right_bottom["lon"] + left_bottom["lon"]) / 4

    def diagonal_offset(p):
        dx = (center_lon - p["lon"]) * 111320 * math.cos(math.radians(p["lat"]))
        dy = (center_lat - p["lat"]) * 111320
        norm = math.hypot(dx, dy)
        scale = 50 / norm
        north_m = dy * scale
        east_m = dx * scale
        new_p = offset_latlon(p["lat"], p["lon"], north_m, east_m)
        new_p["alt"] = p.get("alt", 20)   # <-- dodaj alt
        return new_p

    # ROGI przesunięte po przekątnej
    path.append(diagonal_offset(left_top))
    path.append(diagonal_offset(right_top))
    path.append(diagonal_offset(right_bottom))
    path.append(diagonal_offset(left_bottom))

    # Środki boków
    top_mid = {"lat": (left_top["lat"] + right_top["lat"]) / 2,
               "lon": (left_top["lon"] + right_top["lon"]) / 2,
               "alt": 20}
    path.append(diagonal_offset(top_mid))

    right_mid = {"lat": (right_top["lat"] + right_bottom["lat"]) / 2,
                 "lon": (right_top["lon"] + right_bottom["lon"]) / 2,
                 "alt": 20}
    path.append(diagonal_offset(right_mid))

    bottom_mid = {"lat": (left_bottom["lat"] + right_bottom["lat"]) / 2,
                  "lon": (left_bottom["lon"] + right_bottom["lon"]) / 2,
                  "alt": 20}
    path.append(diagonal_offset(bottom_mid))

    left_mid = {"lat": (left_top["lat"] + left_bottom["lat"]) / 2,
                "lon": (left_top["lon"] + left_bottom["lon"]) / 2,
                "alt": 20}
    path.append(diagonal_offset(left_mid))

    return path


def latlon_to_xy(points, min_lat=None, min_lon=None):
    """
    Konwertuje listę punktów lat/lon na x/y w metrach względem lewego dolnego rogu.
    Jeśli min_lat i min_lon są podane, używa ich jako punktu odniesienia.
    Zwraca listę słowników: {"x": x_m, "y": y_m, "alt": alt}.
    """
    if not points:
        return []

    if min_lat is None:
        min_lat = min(p['lat'] for p in points)
    if min_lon is None:
        min_lon = min(p['lon'] for p in points)

    result = []
    for p in points:
        x_m = (p['lon'] - min_lon) * 111320 * math.cos(math.radians(min_lat))
        y_m = (p['lat'] - min_lat) * 111320
        result.append({"x": x_m, "y": y_m, "alt": p.get("alt", 20)})
    return result



def generate_inside_path(old_waypoints: list, reference_poles: list, buffer_m=5) -> list:
    """
    Generuje ścieżkę wewnętrzną - każdy waypoint przesuwany w stronę środka prostokąta,
    ale tak, aby nie wchodził w obszar reference poles. Buffer_m to minimalny odstęp w metrach od ref polls.
    """
    path_inside = []

    # Obliczamy środek prostokąta old_waypoints
    center_lat = sum(p["lat"] for p in old_waypoints) / len(old_waypoints)
    center_lon = sum(p["lon"] for p in old_waypoints) / len(old_waypoints)

    # granice reference poles (linia pomiędzy nimi)
    if reference_poles:
        min_lat_rp = min(p["lat"] for p in reference_poles)
        max_lat_rp = max(p["lat"] for p in reference_poles)
        min_lon_rp = min(p["lon"] for p in reference_poles)
        max_lon_rp = max(p["lon"] for p in reference_poles)
    else:
        min_lat_rp = max_lat_rp = min_lon_rp = max_lon_rp = None

    # przelicznik metr ↔ stopnie
    deg_per_m_lat = 1 / 110540
    deg_per_m_lon = 1 / (111320 * math.cos(math.radians(center_lat)))
    buffer_deg_lat = buffer_m * deg_per_m_lat
    buffer_deg_lon = buffer_m * deg_per_m_lon

    all_points = reference_poles + old_waypoints
    min_lat = min(p['lat'] for p in all_points)
    min_lon = min(p['lon'] for p in all_points)

    #print("Reference poles in meters:", latlon_to_xy(reference_poles, min_lat=min_lat, min_lon=min_lon))

    for wp in old_waypoints:
        # wektor do środka w metrach
        dx = (center_lon - wp["lon"]) * 111320 * math.cos(math.radians(wp["lat"]))
        dy = (center_lat - wp["lat"]) * 111320
        norm = math.hypot(dx, dy)
        scale = 38 / norm if norm != 0 else 0  # przesunięcie 38 m w stronę środka
        north_m = dy * scale
        east_m = dx * scale

        # nowy waypoint przesunięty w stronę środka
        new_wp = offset_latlon(wp["lat"], wp["lon"], north_m, east_m)
        new_wp["alt"] = wp.get("alt", 20)

        # sprawdzamy granice reference poles z buforem
        if reference_poles:
            # konwertujemy reference poles i new_wp na x/y
            # Wszystkie punkty do wyświetlenia razem
            #new_wp_xy = latlon_to_xy([new_wp], min_lat=min_lat, min_lon=min_lon)[0]
            #print("New waypoint x/y:", new_wp_xy)

            if min_lat_rp - buffer_deg_lat <= new_wp["lat"] - 30 <= max_lat_rp + buffer_deg_lat:
                # wypychamy punkt w pionie w stronę oryginalnego wp
                new_wp["lat"] = wp["lat"] - buffer_deg_lat
            if min_lon_rp - buffer_deg_lon <= new_wp["lon"] <= max_lon_rp + buffer_deg_lon:
                # wypychamy punkt w poziomie w stronę oryginalnego wp
                new_wp["lon"] = wp["lon"]

        path_inside.append(new_wp)

    return path_inside


def generate_single_reference_poles_125m(waypoints, spacing_m=200, margin_m=50):
    """
    Generuje jeden zestaw reference poles (2 punkty w poziomie) zgodnie z zasadami:
    - odległość między nimi w osi X = spacing_m
    - minimalna odległość od boków w osi X = margin_m
    - oba punkty mają tę samą latitude = środek wysokości prostokąta
    """
    left_lon = min(p['lon'] for p in waypoints)
    right_lon = max(p['lon'] for p in waypoints)
    top_lat = max(p['lat'] for p in waypoints)
    bottom_lat = min(p['lat'] for p in waypoints)

    lat_ref = (top_lat + bottom_lat) / 2
    deg_per_m_lon = 1 / (111320 * math.cos(math.radians(lat_ref)))
    margin_deg = margin_m * deg_per_m_lon
    spacing_deg = spacing_m * deg_per_m_lon

    # losowa pozycja X lewego punktu w obrębie prostokąta + margines + spacing
    lon_left = random.uniform(left_lon + margin_deg, right_lon - margin_deg - spacing_deg)
    lon_right = lon_left + spacing_deg

    # latitude = środek wysokości prostokąta
    lat = lat_ref

    return [
        {"lat": lat, "lon": lon_left, "alt": 20},
        {"lat": lat, "lon": lon_right, "alt": 20}
    ]

def generate_single_reference_poles(waypoints, spacing_m=200, margin_m=50):
    """
    Generuje jeden zestaw reference poles (2 punkty w poziomie) zgodnie z zasadami:
    - odległość między nimi w osi X = spacing_m
    - minimalna odległość od boków w osi X i Y = margin_m
    - oba punkty mają tę samą latitude (Y) = losowa w obrębie prostokąta z marginesem
    - losowa wysokość = alt, np. 10–30 m
    """
    left_lon = min(p['lon'] for p in waypoints)
    right_lon = max(p['lon'] for p in waypoints)
    top_lat = max(p['lat'] for p in waypoints)
    bottom_lat = min(p['lat'] for p in waypoints)

    # przelicznik stopnie ↔ metry
    lat_ref = (top_lat + bottom_lat) / 2
    deg_per_m_lon = 1 / (111320 * math.cos(math.radians(lat_ref)))
    deg_per_m_lat = 1 / 110540  # przybliżenie

    margin_deg_lon = margin_m * deg_per_m_lon
    margin_deg_lat = margin_m * deg_per_m_lat
    spacing_deg = spacing_m * deg_per_m_lon

    # losowa pozycja lewego punktu w obrębie prostokąta + margines + spacing
    lon_left = random.uniform(left_lon + margin_deg_lon, right_lon - margin_deg_lon - spacing_deg)
    lon_right = lon_left + spacing_deg

    # losowa latitude w granicach prostokąta z marginesem
    lat = random.uniform(bottom_lat + margin_deg_lat, top_lat - margin_deg_lat)

    # losowa wysokość (alt)
    alt = random.uniform(10, 30)

    return [
        {"lat": lat, "lon": lon_left, "alt": alt},
        {"lat": lat, "lon": lon_right, "alt": alt}
    ]


def save_to_txt(first_lap, inside_path, filename="PathGeneration/waypoints.txt"):
    """
    Saves first lap and inside path waypoints to a text file.
    Each waypoint is saved as: lat, lon, alt
    """
    with open(filename, 'w') as f:
        for wp in first_lap + inside_path:
            f.write(f"{wp['lat']:.8f}, {wp['lon']:.8f}, {wp['alt']}\n")


def main_test():
    waypoints = [
        {"lat": -35.362138, "lon": 149.163302, "alt": 20},
        {"lat": -35.362138, "lon": 149.167158, "alt": 20},
        {"lat": -35.364384, "lon": 149.167158, "alt": 20},
        {"lat": -35.364384, "lon": 149.163302, "alt": 20}
    ]
    reference_poles = generate_single_reference_poles_125m(waypoints, spacing_m=200, margin_m=50)

    # pierwsza lap
    first_lap = first_lap_path(waypoints[0], waypoints[1], waypoints[2], waypoints[3])

    # pierwsza inside path
    inside_path = generate_inside_path(first_lap, reference_poles)

    # rysujemy wszystkie punkty
    plot_coordinates(waypoints, reference_poles, first_lap, inside_path)
    
    # zapisujemy waypoints do pliku
    save_to_txt(first_lap, inside_path)

def main_use():
    # UZUPEŁNIĆ - 4 rogi prostokąta
    waypoints = [
        {"lat": -35.362138, "lon": 149.163302, "alt": 20},
        {"lat": -35.362138, "lon": 149.167158, "alt": 20},
        {"lat": -35.364384, "lon": 149.167158, "alt": 20},
        {"lat": -35.364384, "lon": 149.163302, "alt": 20}
    ]

    # UZUPEŁNIĆ reference poles: lat lon alt
    reference_poles = [
        {"lat": -35.363261, "lon": 149.163800, "alt": 18},  # lewy słupek
        {"lat": -35.363261, "lon": 149.166660, "alt": 18},  # prawy słupek
    ]

    # generujemy first lap
    first_lap = first_lap_path(waypoints[0], waypoints[1], waypoints[2], waypoints[3])

    # generujemy inside path
    inside_path = generate_inside_path(first_lap, reference_poles)

    # zapisujemy waypoints do pliku
    save_to_txt(first_lap, inside_path)

if __name__ == "__main__":
    #main_test()
    main_use()