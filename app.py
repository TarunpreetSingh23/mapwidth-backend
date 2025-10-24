from flask import Flask, request, jsonify
from flask_cors import CORS
import networkx as nx

try:
    import osmnx as ox
    OSMNX_AVAILABLE = True
except Exception:
    OSMNX_AVAILABLE = False

app = Flask(__name__)
CORS(app)

# --- Defaults ---
DEFAULT_LANE_WIDTH = 3.0
DEFAULT_CENTER_LAT = 31.6339
DEFAULT_CENTER_LON = 74.8770
DEFAULT_DIST = 5000
G_CACHED = None
CACHED_BBOX = None
PENALTY_WEIGHT = 1e6  # very high travel time for narrow roads

# --- Average Speeds (km/h) ---
AVG_SPEEDS_KMH = {
    'motorway': 90,
    'trunk': 80,
    'primary': 60,
    'secondary': 45,
    'tertiary': 35,
    'residential': 25,
    'service': 15,
    'unclassified': 20,
    'default': 30
}


def load_default_graph():
    global G_CACHED, CACHED_BBOX
    if not OSMNX_AVAILABLE:
        print("OSMNX not available, skipping cache.")
        return
    try:
        G_CACHED = ox.graph_from_point(
            (DEFAULT_CENTER_LAT, DEFAULT_CENTER_LON),
            dist=DEFAULT_DIST,
            network_type='drive'
        )
        nodes = G_CACHED.nodes
        CACHED_BBOX = (
            min(nodes[n]['y'] for n in nodes),
            min(nodes[n]['x'] for n in nodes),
            max(nodes[n]['y'] for n in nodes),
            max(nodes[n]['x'] for n in nodes)
        )
        print("✅ Default graph loaded successfully.")
    except Exception as e:
        print(f"⚠️ Failed to load default graph: {e}")
        G_CACHED = None


def estimate_edge_width(data):
    width = None
    if 'width' in data:
        try:
            width = float(data['width'][0] if isinstance(data['width'], list) else data['width'])
        except:
            pass
    if width is None and 'lanes' in data:
        try:
            lanes = int(data['lanes'][0] if isinstance(data['lanes'], list) else data['lanes'])
            width = lanes * DEFAULT_LANE_WIDTH
        except:
            pass
    if width is None:
        hw = data.get('highway', 'residential')
        if isinstance(hw, list):
            hw = hw[0]
        heur = {'motorway': 11, 'trunk': 10, 'primary': 9, 'secondary': 8,
                'tertiary': 7, 'residential': 6, 'service': 4}
        width = heur.get(hw, 5.5)
    return width


@app.route("/route")
def get_route():
    if not OSMNX_AVAILABLE:
        return jsonify({"error": "OSMnx not installed"}), 501

    try:
        s_lat = float(request.args.get('start_lat'))
        s_lon = float(request.args.get('start_lon'))
        e_lat = float(request.args.get('end_lat'))
        e_lon = float(request.args.get('end_lon'))
        vehicle_width = float(request.args.get('vehicle_width', 3.0))
    except:
        return jsonify({"error": "Missing or invalid coordinates"}), 400

    north = max(s_lat, e_lat) + 0.005
    south = min(s_lat, e_lat) - 0.005
    east = max(s_lon, e_lon) + 0.005
    west = min(s_lon, e_lon) - 0.005

    G_base = None
    if G_CACHED and CACHED_BBOX:
        min_lat, min_lon, max_lat, max_lon = CACHED_BBOX
        if south >= min_lat and north <= max_lat and west >= min_lon and east <= max_lon:
            G_base = G_CACHED

    if G_base is None:
        try:
            G_base = ox.graph_from_bbox(north, south, east, west, network_type='drive')
        except Exception as e:
            return jsonify({"error": f"Could not build road network: {e}"}), 500

    G_processed = G_base.copy()

    # --- Add edge speeds safely ---
    for u, v, k, data in G_processed.edges(keys=True, data=True):
        hw = data.get('highway', 'default')
        if isinstance(hw, list):
            hw = hw[0]
        data['speed_kph'] = AVG_SPEEDS_KMH.get(hw, AVG_SPEEDS_KMH['default'])

    # --- Add travel times manually ---
    for u, v, k, data in G_processed.edges(keys=True, data=True):
        length_m = data.get('length', 1)
        speed_mps = data['speed_kph'] * 1000 / 3600
        data['travel_time'] = length_m / speed_mps  # seconds

    # --- Apply width penalties ---
    for u, v, k, data in G_processed.edges(keys=True, data=True):
        width = estimate_edge_width(data)
        if width < vehicle_width:
            data['travel_time'] += PENALTY_WEIGHT

    try:
        orig_node = ox.distance.nearest_nodes(G_processed, s_lon, s_lat)
        dest_node = ox.distance.nearest_nodes(G_processed, e_lon, e_lat)
        route = nx.shortest_path(G_processed, orig_node, dest_node, weight='travel_time')
    except nx.NetworkXNoPath:
        return jsonify({"error": "No route found"}), 404
    except Exception as e:
        return jsonify({"error": f"Routing error: {e}"}), 500

    coords = [(G_processed.nodes[n]['y'], G_processed.nodes[n]['x']) for n in route]

    total_distance = 0
    total_time_s = 0
    route_names = []

    for i in range(len(route) - 1):
        u, v = route[i], route[i + 1]
        d = G_processed.get_edge_data(u, v, 0)
        total_distance += d.get('length', 0)
        total_time_s += d.get('travel_time', 0)
        name = d.get('name') or d.get('ref') or d.get('highway')
        if isinstance(name, list):
            name = name[0]
        if not name:
            name = "Unnamed Road"
        if name not in route_names:
            route_names.append(name)

    response = {
        "route": coords,
        "route_names": route_names,
        "distance_km": round(total_distance / 1000, 2),
        "duration_min": round(total_time_s / 60, 1),  # convert seconds → minutes
        "vehicle_width": vehicle_width
    }

    return jsonify(response)


if __name__ == "__main__":
    load_default_graph()
    app.run(host="0.0.0.0", port=5000, debug=True)
