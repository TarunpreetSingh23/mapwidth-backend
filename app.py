from flask import Flask, request, jsonify
from flask_cors import CORS
try:
    import osmnx as ox
    import networkx as nx
    OSMNX_AVAILABLE = True
except Exception:
    OSMNX_AVAILABLE = False
import copy

app = Flask(__name__)
CORS(app)

# --- Defaults ---
DEFAULT_LANE_WIDTH = 3.0
DEFAULT_CENTER_LAT = 31.6339
DEFAULT_CENTER_LON = 74.8770
DEFAULT_DIST = 5000
G_CACHED = None
CACHED_BBOX = None

# --- Estimated Speeds (km/h) based on highway type for time calculation ---
# Used for average travel time estimation.
AVG_SPEEDS_KMH = {
    'motorway': 90,
    'trunk': 80,
    'primary': 60,
    'secondary': 45,
    'tertiary': 35,
    'residential': 25,
    'service': 15,
    'unclassified': 20,
    'default': 30  # Fallback speed
}


# --- Load default graph ---
def load_default_graph():
    global G_CACHED, CACHED_BBOX
    if not OSMNX_AVAILABLE:
        print("OSMNX not available, caching skipped.")
        return

    try:
        # NOTE: Using 'all' for network_type initially to get all edges, then filter by drive later if needed
        # We'll stick to 'drive' as in the original code.
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
        print("Default graph loaded successfully.")
    except Exception as e:
        print(f"Failed to load default graph: {e}. Routing will use on-the-fly calls.")
        G_CACHED = None


# --- Estimate edge width (Unchanged) ---
def estimate_edge_width(data):
    width = None
    # Width tag
    if 'width' in data:
        try:
            width = float(data['width'][0] if isinstance(data['width'], list) else data['width'])
        except: pass
    # Lanes
    if width is None and 'lanes' in data:
        try:
            lanes = int(data['lanes'][0] if isinstance(data['lanes'], list) else data['lanes'])
            width = lanes * DEFAULT_LANE_WIDTH
        except: pass
    # Fallback by highway type
    if width is None:
        hw = data.get('highway', 'residential')
        if isinstance(hw, list):
            hw = hw[0]
        heur = {'motorway':11,'trunk':10,'primary':9,'secondary':8,'tertiary':7,'residential':6,'service':4}
        width = heur.get(hw, 5.5)
    return width

# --- Route endpoint ---
if OSMNX_AVAILABLE:
    @app.route("/route")
    def get_route():
        try:
            s_lat = float(request.args.get('start_lat'))
            s_lon = float(request.args.get('start_lon'))
            e_lat = float(request.args.get('end_lat'))
            e_lon = float(request.args.get('end_lon'))
        except:
            return jsonify({"error": "Missing or invalid coordinates"}), 400

        try:
            vehicle_width = float(request.args.get('vehicle_width', 3.0))
        except:
            return jsonify({"error": "Invalid vehicle_width value"}), 400

        north = max(s_lat, e_lat) + 0.005
        south = min(s_lat, e_lat) - 0.005
        east = max(s_lon, e_lon) + 0.005
        west = min(s_lon, e_lon) - 0.005

        # Determine which graph to use (cached or on-the-fly)
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

        # --- Prepare G_request (Width-Restricted Graph) ---
        G_request = copy.deepcopy(G_base)
        edges_to_remove = []

        # Add necessary metrics (width_est, speed, time) and identify edges to remove
        for u, v, k, data in G_request.edges(keys=True, data=True):
            width = estimate_edge_width(data)
            data['width_est'] = width
            data['weight'] = data.get('length', 1) 

            # Estimate speed for time calculation
            hw = data.get('highway')
            if isinstance(hw, list):
                hw = hw[0]
            speed_kmh = AVG_SPEEDS_KMH.get(hw, AVG_SPEEDS_KMH['default'])
            data['speed_kmh'] = speed_kmh
            data['travel_time_min'] = (data.get('length', 1) / (speed_kmh * 1000 / 60))

            if width < vehicle_width:
                edges_to_remove.append((u, v, k))
        
        # Remove width-restricted edges
        for edge in edges_to_remove:
            G_request.remove_edge(*edge)

        # --- Compute route ---
        orig = ox.distance.nearest_nodes(G_request, s_lon, s_lat)
        dest = ox.distance.nearest_nodes(G_request, e_lon, e_lat)
        
        # The graph used for routing, starts with the width-restricted one
        G_route = G_request
        is_fallback = False

        try:
            # 1. Attempt route calculation on the width-restricted graph (G_request)
            route = nx.shortest_path(G_route, orig, dest, weight='weight')

        except nx.NetworkXNoPath:
            # 2. FALLBACK: Route failed. Try on the original, unrestricted graph (G_base).
            
            # Re-find nearest nodes on the original graph G_base
            orig_fallback = ox.distance.nearest_nodes(G_base, s_lon, s_lat)
            dest_fallback = ox.distance.nearest_nodes(G_base, e_lon, e_lat)
            
            G_route = G_base # Switch to the unrestricted graph
            is_fallback = True

            try:
                # Need to ensure 'weight' (length) exists on G_base if it wasn't processed above.
                # Since G_request was a deepcopy of G_base, we assume G_base has 'length'/'weight'.
                # But to be safe, we assign the weight attribute if it's missing.
                for _, _, data in G_route.edges(data=True):
                    if 'weight' not in data:
                        data['weight'] = data.get('length', 1)

                route = nx.shortest_path(G_route, orig_fallback, dest_fallback, weight='weight')
                
            except nx.NetworkXNoPath:
                # 3. Final failure: No path even on the unrestricted graph.
                return jsonify({"error": "No route available, even without width restrictions."}), 404
            except Exception as e:
                return jsonify({"error": f"Fallback routing error: {e}"}), 500
        
        # --- Process successful route ---
        coords = [(G_route.nodes[n]['y'], G_route.nodes[n]['x']) for n in route]

        total_distance_m = 0
        total_duration_min = 0
        route_names = []
        
        for i in range(len(route) - 1):
            u, v = route[i], route[i + 1]
            data = G_route.get_edge_data(u, v, 0) 
            
            if not data:
                continue
            
            # --- Ensure metrics exist for the final graph (important for fallback) ---
            if 'travel_time_min' not in data:
                hw = data.get('highway')
                if isinstance(hw, list):
                    hw = hw[0]
                speed_kmh = AVG_SPEEDS_KMH.get(hw, AVG_SPEEDS_KMH['default'])
                data['travel_time_min'] = (data.get('length', 1) / (speed_kmh * 1000 / 60))


            # Sum distance (in meters) and time (in minutes)
            total_distance_m += data.get('length', 0)
            total_duration_min += data.get('travel_time_min', 0)

            # Collect route names
            name = data.get('name') or data.get('ref') or data.get('highway')
            if isinstance(name, list):
                name = name[0]
            if not name:
                name = f"Unnamed Road ({round(data.get('length',0))} m)"
            if name not in route_names:
                route_names.append(name)
        
        total_distance_km = total_distance_m / 1000

        response = {
            "route": coords,
            "route_names": route_names,
            "num_nodes": len(coords),
            "vehicle_width": vehicle_width,
            "distance_km": round(total_distance_km, 2),
            "duration_min": round(total_duration_min)
        }
        
        if is_fallback:
            response["error"] = "Warning: The width-restricted route failed. Displaying an alternate route (all roads included)."

        return jsonify(response)

    @app.route("/route")
    def route_not_available():
        return jsonify({"error": "osmnx or networkx not available"}), 501

if __name__ == "__main__":
    load_default_graph()
    app.run(host="0.0.0.0", port=5000, debug=True)