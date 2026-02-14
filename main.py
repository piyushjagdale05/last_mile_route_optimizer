import pandas as pd
import numpy as np
import folium
from math import radians, sin, cos, sqrt, atan2
from ortools.constraint_solver import pywrapcp, routing_enums_pb2

# =================================================
# 1. HAVERSINE DISTANCE
# =================================================
def haversine(lat1, lon1, lat2, lon2):
    R = 6371
    dlat = radians(lat2-lat1)
    dlon = radians(lon2-lon1)

    a = sin(dlat/2)**2 + cos(radians(lat1)) * cos(radians(lat2)) * sin(dlon/2)**2
    c = 2 * atan2(sqrt(a), sqrt(1-a))

    return R * c


# =================================================
# 2. LOAD DATA
# =================================================
df = pd.read_csv("locations.csv")

coords = list(zip(df.lat, df.lon))
n = len(coords)

print("Total Stops:", n)


# =================================================
# 3. DISTANCE MATRIX (meters)
# =================================================
dist_matrix = np.zeros((n,n))

for i in range(n):
    for j in range(n):
        dist_matrix[i][j] = haversine(*coords[i], *coords[j]) * 1000

dist_matrix = dist_matrix.astype(int)


# =================================================
# 4. SETTINGS
# =================================================
num_vehicles = 3
vehicle_capacity = 3
speed = 40  # km/h


# =================================================
# 5. TRAVEL TIME MATRIX (minutes)
# =================================================
time_matrix = (dist_matrix/1000) / speed * 60
time_matrix = time_matrix.astype(int)


# =================================================
# 6. DEMANDS (capacity)
# =================================================
demands = [0] + [1]*(n-1)


# =================================================
# 7. TIME WINDOWS (minutes)
# format: (start, end)
# depot open whole day
# =================================================
time_windows = [
    (0, 600),   # depot
    (0, 200),
    (50, 300),
    (100, 400),
    (200, 500),
    (150, 450),
    (250, 600)
]


# =================================================
# 8. ORTOOLS MODEL
# =================================================
manager = pywrapcp.RoutingIndexManager(n, num_vehicles, 0)
routing = pywrapcp.RoutingModel(manager)


# ---------- Distance callback ----------
def distance_callback(from_index, to_index):
    return dist_matrix[
        manager.IndexToNode(from_index)
    ][
        manager.IndexToNode(to_index)
    ]

transit_callback = routing.RegisterTransitCallback(distance_callback)
routing.SetArcCostEvaluatorOfAllVehicles(transit_callback)


# =================================================
# CAPACITY CONSTRAINT
# =================================================
def demand_callback(from_index):
    return demands[manager.IndexToNode(from_index)]

demand_callback_index = routing.RegisterUnaryTransitCallback(demand_callback)

routing.AddDimensionWithVehicleCapacity(
    demand_callback_index,
    0,
    [vehicle_capacity]*num_vehicles,
    True,
    "Capacity"
)


# =================================================
# TIME WINDOW CONSTRAINT
# =================================================
def time_callback(from_index, to_index):
    return time_matrix[
        manager.IndexToNode(from_index)
    ][
        manager.IndexToNode(to_index)
    ]

time_callback_index = routing.RegisterTransitCallback(time_callback)

routing.AddDimension(
    time_callback_index,
    30,     # waiting allowed
    600,    # max time per vehicle
    False,
    "Time"
)

time_dimension = routing.GetDimensionOrDie("Time")

for i, window in enumerate(time_windows):
    index = manager.NodeToIndex(i)
    time_dimension.CumulVar(index).SetRange(window[0], window[1])


# =================================================
# SOLVE
# =================================================
search = pywrapcp.DefaultRoutingSearchParameters()
search.first_solution_strategy = routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC

solution = routing.SolveWithParameters(search)


# =================================================
# PRINT ROUTES
# =================================================
routes = []
total_distance = 0

for vehicle_id in range(num_vehicles):

    index = routing.Start(vehicle_id)
    route = []
    print(f"\nVehicle {vehicle_id+1}")

    while not routing.IsEnd(index):

        node = manager.IndexToNode(index)
        time_var = time_dimension.CumulVar(index)

        print(
            f"Stop {node} "
            f"Time({solution.Min(time_var)}-{solution.Max(time_var)})"
        )

        route.append(node)

        next_index = solution.Value(routing.NextVar(index))
        total_distance += routing.GetArcCostForVehicle(index, next_index, vehicle_id)
        index = next_index

    route.append(0)
    routes.append(route)

print("\nTotal Distance (m):", total_distance)


# =================================================
# MAP VISUALIZATION
# =================================================
m = folium.Map(location=[df.lat.mean(), df.lon.mean()], zoom_start=13)

colors = ["red", "blue", "green", "purple"]

for i,row in df.iterrows():
    folium.Marker([row.lat,row.lon], popup=f"{i}").add_to(m)

for idx, route in enumerate(routes):
    pts = [[coords[n][0], coords[n][1]] for n in route]
    folium.PolyLine(pts, color=colors[idx], weight=5).add_to(m)

m.save("routes_map.html")

print("Map saved → routes_map.html")




import requests

API_KEY = "YOUR_GOOGLE_API_KEY"

def get_traffic_matrix(origins, destinations):
    url = "https://routes.googleapis.com/directions/v2:computeRouteMatrix"
    body = {
        "origins": [{"location": {"latLng": {"latitude": o[0], "longitude": o[1]}}} for o in origins],
        "destinations": [{"location": {"latLng": {"latitude": d[0], "longitude": d[1]}}} for d in destinations],
        "travelMode": "DRIVE",
        "routingPreference": "TRAFFIC_AWARE_OPTIMAL",
        "departureTime": "now"
    }

    headers = {
        "Content-Type": "application/json",
        "X-Goog-Api-Key": API_KEY
    }

    resp = requests.post(url, json=body, headers=headers).json()
    matrix = []
    for row in resp["rows"]:
        row_times = []
        for element in row["elements"]:
            # Use duration in traffic if available
            val = element.get("durationInTraffic", element.get("duration", {}).get("value", 0))
            row_times.append(val)
        matrix.append(row_times)
    return matrix
