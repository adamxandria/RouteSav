import os
import sys
import plotly.graph_objects as go
import osmnx as ox
from PyQt5 import QtWidgets, QtWebEngineWidgets, QtCore
from PyQt5.QtWidgets import QLabel, QComboBox
from plotly import offline
import heapq
from multiprocessing import Process
import pytz
from datetime import datetime
import requests
import json

headers = {
    'AccountKey': '6UiItqJPT8WWLFB4FnIMCg==',
    'accept': 'application/json'
}

MODE = "drive"
NORTH = 1.3701654712520062
SOUTH = 1.2946774207297054
EAST = 103.9907219819978
WEST = 103.87206966924965
PERIMETER = 0.001
ox.settings.log_console = True
ox.settings.use_cache = True
# Define the nodes for which ERP charges apply
ERP_NODES = {5917182554: "EC1", 5918708421: "PE4"}  # Replace with your specific node IDs


def fetch_all(url):
    results = []
    while True:
        new_results = requests.get(
            url,
            headers=headers,
            params={'$skip': len(results)}
        ).json()['value']
        if new_results == []:
            break
        else:
            results += new_results
    return results

def dijkstra_shortest_path(graph, source, target, toll, incident_nodes, nodes_to_avoid, speed, road_type, vehicle_type):
    # Initialize distances to infinity for all nodes except the source node
    distances = {node: float('inf') for node in graph}
    distances[source] = 0
    # dict to keep track of prev nodes in shortest path
    previous_nodes = {node: None for node in graph}
    avg_speed_limit = 50.0
    # incident node for testing, minwah, dont avoid toll
    # incident_nodes.append(7153273679)
    # incident_nodes.append(5918708421)
    # queue to store nodes based on dist
    priority_queue = [(0, source)]
    while priority_queue:
        speed_weight_factor = 1 / avg_speed_limit
        erp_weight_factor = 1
        incident_weight_factor = 1

        # get node with smallest dist from queue
        current_distance, current_node = heapq.heappop(priority_queue)

        # if reach target node, exit
        if current_node == target:
            break

        # skip iteration if current dist > dist to current node
        if current_distance > distances[current_node]:
            continue

        # explore neighbours of current node
        for neighbor, weight in graph[current_node].items():
            if neighbor in nodes_to_avoid:
                continue
            if toll:
                if neighbor in ERP_NODES.keys():  # Check if neighbor is an ERP node
                    continue
            if neighbor in ERP_NODES.keys():  # Check if neighbor is an ERP node
                rate = erp_rate(ERP_NODES[neighbor])
                if rate != 0:
                    erp_weight_factor = rate
            if neighbor in incident_nodes:
                incident_weight_factor = 5.25

            # Calculate the dist to the neighbor node, including fuel consumption
            if 'maxspeed' in weight[0]:
                max_speed = float(weight[0]['maxspeed'])
                speed_weight_factor = 1 / max_speed  # Higher maximum speed results in a lower weight factor

            distance = current_distance + weight[0]['length'] * speed_weight_factor * erp_weight_factor * incident_weight_factor

            # Calculate the fuel consumption for the edge
            edge_speed = max_speed if 'maxspeed' in weight[0] else avg_speed_limit
            fuel_consumption = calculate_fuel_consumption(weight[0]['length'], edge_speed, road_type, vehicle_type)

            # Update dist and prev node if new distance shorter
            if distance < distances[neighbor]:
                distances[neighbor] = distance
                previous_nodes[neighbor] = current_node
                heapq.heappush(priority_queue, (distance, neighbor))

    if distances[target] == float('inf'):
        return None  # No path found

    # Reconstruct the path from the target node to the source node
    current_path = []
    current_node = target
    while current_node is not None:
        current_path.append(current_node)
        current_node = previous_nodes[current_node]
    # reverse path from src to target
    current_path.reverse()

    if len(current_path) > 1:
        # Check if the node splits into two paths
        # prev_node = current_path[-2]
        # next_node = current_path[1]
        for i in range(len(current_path) - 1):
            node = current_path[i]
            next_node = current_path[i + 1]
            # num_of_ways = len(graph[node].items())
            items = list(graph[node].items())
            if len(items) > 1:
                for item, edge in items:
                    if item == next_node and 'motorway' in edge[0]['highway']:
                        nodes_to_avoid.append(item)

    return current_path, nodes_to_avoid



def create_graph():
    """use osmnx to pull map data and process to graph"""
    if os.path.exists('preprocessed_graph.graphml'):
        os.remove('preprocessed_graph.graphml')
    graph = ox.graph_from_bbox(NORTH + PERIMETER, SOUTH - PERIMETER, EAST + PERIMETER, WEST - PERIMETER,
                               network_type=MODE, simplify=False)

    ox.save_graphml(graph, 'preprocessed_graph.graphml')


def calculate_total_distance(graph, path):
    total_distance = 0
    for i in range(len(path) - 1):
        node1 = path[i]
        node2 = path[i + 1]
        edge_data = graph[node1][node2][0]
        edge_length = edge_data['length'] / 1000
        total_distance += edge_length
    return "{:.2f}".format(total_distance)


def calculate_fuel_consumption(distance, speed, road_type, vehicle_type):
    # Speed factor
    speed_threshold = 80.0  # Adjust as per your requirements
    if speed > speed_threshold:
        fuel_consumption_rate = 0.12  # High speed fuel consumption rate
    else:
        fuel_consumption_rate = 0.08  # Normal speed fuel consumption rate

    # Road conditions factor
    road_type_factors = {
        'motorway': 0.9,
        'city': 1.0,
        'off-road': 1.2
    }
    road_factor = road_type_factors.get(road_type, 1.0)

    # Vehicle type factor
    vehicle_type_factors = {
        'sedan': 1.0,
        'suv': 1.2,
        'truck': 1.5
    }
    vehicle_factor = vehicle_type_factors.get(vehicle_type, 1.0)

    # Calculate fuel consumption based on distance, speed, road conditions, and vehicle type
    fuel_consumption = distance * fuel_consumption_rate * road_factor * vehicle_factor
    return fuel_consumption



def calculate_cumulative_time(graph, path, incident_nodes):
    cumulative_time = 0
    num_incident_nodes_passed = 0
    for i in range(len(path) - 1):
        if path[i] in incident_nodes:
            num_incident_nodes_passed += 1
        node1 = path[i]
        node2 = path[i + 1]
        edge_data = graph[node1][node2][0]
        edge_length = edge_data['length'] / 1000
        if 'maxspeed' in edge_data:
            edge_speed = float(edge_data['maxspeed'])  # Speed specific to the edge
        else:
            edge_speed = 50.0
        edge_time = edge_length / edge_speed * 60
        cumulative_time += edge_time
    # 30% allowance to consider traffic and slower driving, as not possible to drive at max speed all the way
    # additional 3 min for every incident node passed
    return round(cumulative_time * 1.3 + (num_incident_nodes_passed * 3))


def erp_rate(zoneid):
    # return erp rate
    erp_rates = fetch_all("http://datamall2.mytransport.sg/ltaodataservice/ERPRates")
    vehicle_type = "Big Bus"
    day_type = 'none'
    cost = 0
    # Get the current date
    today = datetime.now().date()
    # Check if today is a weekday (Monday to Friday)
    if today.weekday() < 5:
        day_type = "Weekdays"
    elif today.weekday() == 5:
        day_type = "Saturday"
    # Set the time zone to Singapore
    sg_timezone = pytz.timezone('Asia/Singapore')
    # Get the current time in Singapore
    current_time = datetime.now(sg_timezone).time()
    for erp in erp_rates:
        if zoneid in erp['ZoneID']:
            if vehicle_type in erp['VehicleType']:
                if day_type in erp['DayType']:
                    # Convert the start and end time strings to time objects
                    start_time = datetime.strptime(erp['StartTime'], '%H:%M').time()
                    end_time = datetime.strptime(erp['EndTime'], '%H:%M').time()
                    if start_time <= current_time <= end_time:
                        cost = float(erp['ChargeAmount'])
                        break
    return cost


def calculate_total_cost(route):
    # calculate total cost trip (ie. ERP)
    cost = 0
    zoneids = []
    for node in ERP_NODES.keys():
        if node in route:
            zoneids.append(ERP_NODES[node])

    for zoneid in zoneids:
        cost += erp_rate(zoneid)
    return cost

def generating_path(origin_point, target_point, toll, speed, road_type, vehicle_type):
    """load processed graph and use to calculate optimal route"""
    # create_graph_process.join()
    paths = []
    routes = []
    nodes_to_avoid = []

    # Load the pre-processed graph
    graph = ox.load_graphml('preprocessed_graph.graphml')

    # Get the nearest node in the OSMNX graph for the origin point
    origin_node = ox.distance.nearest_nodes(graph, origin_point[1], origin_point[0])

    # Get the nearest node in the OSMNX graph for the target point
    target_node = ox.distance.nearest_nodes(graph, target_point[1], target_point[0])

    incident_nodes = get_incidents(graph)
    # Get the optimal path via Dijkstra's algorithm
    path, nodes_to_avoid = dijkstra_shortest_path(graph, origin_node, target_node, toll, incident_nodes, nodes_to_avoid, speed, road_type, vehicle_type)
    paths.append(path)
    path, nodes_to_avoid = dijkstra_shortest_path(graph, origin_node, target_node, toll, incident_nodes, nodes_to_avoid, speed, road_type, vehicle_type)
    paths.append(path)

    for path in paths:
        total_distance = calculate_total_distance(graph, path)
        cumulative_time = calculate_cumulative_time(graph, path, incident_nodes)
        total_cost = calculate_total_cost(path)
        fuel_consumption = calculate_fuel_consumption(float(total_distance), float(cumulative_time), road_type, vehicle_type)

        lat = []
        long = []

        for i in path:
            point = graph.nodes[i]
            long.append(point['x'])
            lat.append(point['y'])
        routes.append((long, lat, total_distance, cumulative_time, total_cost,fuel_consumption))

    # swap the first in array to be the top priority
    time1 = float(routes[0][3])
    time2 = float(routes[1][3])
    distance1 = float(routes[0][2])
    distance2 = float(routes[1][2])
    erp1 = float(routes[0][4])
    erp2 = float(routes[1][4])
    # swap to print the red line above the grey
    if time1 < time2:
        routes[0], routes[1] = routes[1], routes[0]
    elif time1 == time2 and distance1 < distance2:
        routes[0], routes[1] = routes[1], routes[0]
    elif time1 == time2 and distance1 == distance2 and erp1 < erp2:
        routes[0], routes[1] = routes[1], routes[0]
    # Return the route route
    return routes


def get_incidents(graph):
    # Define the API endpoint URL
    url = 'http://datamall2.mytransport.sg/ltaodataservice/TrafficIncidents'

    # get incident data from LTA
    incidents = fetch_all(url)
    # array to store incident nodes
    incident_nodes = []
    # Process the traffic incident data
    for incident in incidents:
        # Find the nearest node from OSMnx graph
        nearest_node = ox.distance.nearest_nodes(graph, incident['Longitude'], incident['Latitude'])
        incident_nodes.append(nearest_node)
    return incident_nodes

def plot_map(origin_point, target_point, routes):
    """plot route onto map"""
    print(origin_point)
    print(target_point)
    # Create a plotly map and add the origin point to the map
    print("Plotting map...")
    fig = go.Figure(go.Scattermapbox(
        name="Origin",
        mode="markers",
        lon=[origin_point[1]],
        lat=[origin_point[0]],
        showlegend=False,
        marker={'size': 16, 'color': "#333333"}
    )
    )

    for index, route in enumerate(routes):
        # long, lat, _, _, _ = route  # Adjusted the unpacking to match 5 values
        # Plot the optimal paths to the map
        long, lat, total_dist, cumulative_time, total_cost, fuel_consumption = route

        print("Generating paths.....")
        if index == 0:
            fig.add_trace(go.Scattermapbox(
                name="Path",
                mode="lines",
                lon=long,
                lat=lat,
                marker={'size': 10},
                showlegend=False,
                line=dict(width=4.5, color='#CCCCCC'))
            )

        else:
            fig.add_trace(go.Scattermapbox(
                name="Path",
                mode="lines",
                lon=long,
                lat=lat,
                marker={'size': 10},
                showlegend=False,
                line=dict(width=4.5, color='#ff0000'))
            )

    # Plot the target geocoordinates to the map
    print("Generating target...")
    fig.add_trace(go.Scattermapbox(
        name="Destination",
        mode="markers",
        showlegend=False,
        lon=[target_point[1]],
        lat=[target_point[0]],
        marker={'size': 16, 'color': '#ff0000'}))

    # Style the map layout
    fig.update_layout(
        mapbox_style="streets",
        mapbox_accesstoken="pk.eyJ1IjoiYWRhbXhhbmRyaWEiLCJhIjoiY2xqanRhbHpkMGFzbDNsbXU5bGxvaG9kcyJ9.0kKSEs9qPLBECjCqbNZ68A",
        legend=dict(yanchor="top", y=1, xanchor="left", x=0.83),  # x 0.9
        title="<span style='font-size: 32px;'><b>RouteSAV</b></span>",
        font_family="Times New Roman",
        font_color="#333333",
        title_font_size=32,
        font_size=18,
        width=1920,
        height=1080,
    )

    # Set the center of the map
    lat_center = lat[int(len(lat) / 2)] - 0.008
    long_center = long[int(len(long) / 2)] + 0.05

    # Add the center to the map layout
    fig.update_layout(margin={"r": 0, "t": 0, "l": 0, "b": 0},
                      title=dict(yanchor="top", y=.97, xanchor="left", x=0.03),  # x 0.75
                      mapbox={
                          'center': {'lat': lat_center,
                                     'lon': long_center},
                          'zoom': 12}
                      )
    # Save the figure as an HTML file
    offline.plot(fig, filename='plot.html', auto_open=False)

class Window(QtWidgets.QMainWindow):
    """Main window GUI"""

    def __init__(self):
        super().__init__()
        self.infolay = None
        self.vlay = None
        self.view = None
        self.toll_dropdown = None
        self.label_cost = None
        self.label_distance = None
        self.label_time = None
        self.destination_dropdown = None
        self.source_dropdown = None
        self.initWindow()

    def initWindow(self):
        self.setWindowTitle(self.tr("RouteSAV"))
        self.setFixedSize(1500, 800)
        self.buttonUI()
        self.display_map('default.html')

    def buttonUI(self):
        """create and display all button and widgets"""
        # Create the "Source" title label
        source_label = QLabel("Starting Point", self)

        # Create the source dropdown
        self.source_dropdown = QComboBox(self)
        self.source_dropdown.addItem("Changi Airport Terminal 3", [1.350401, 103.9850091])
        self.source_dropdown.addItem("ibis budget Singapore Pearl", [1.3116102100626978, 103.87927240561176])
        self.source_dropdown.addItem("Min Wah Hotel", [1.3121928244165013, 103.88242907961897])
        self.source_dropdown.addItem("Amrise Hotel", [1.3139710326135319, 103.87786884865685])

        # Create the "Destination" title label
        destination_label = QLabel("Destination", self)

        # Create the destination dropdown
        self.destination_dropdown = QComboBox(self)
        self.destination_dropdown.addItem("Changi Airport Terminal 3", [1.350401, 103.9850091])
        self.destination_dropdown.addItem("ibis budget Singapore Pearl", [1.3116102100626978, 103.87927240561176])
        self.destination_dropdown.addItem("Min Wah Hotel", [1.3121928244165013, 103.88242907961897])
        self.destination_dropdown.addItem("Amrise Hotel", [1.3139710326135319, 103.87786884865685])

        # Create the "Avoid Toll" title label
        toll_label = QLabel("Avoid Toll", self)

        # Create the "avoid toll" dropdown
        self.toll_dropdown = QComboBox(self)
        self.toll_dropdown.addItem("Yes", True)
        self.toll_dropdown.addItem("No", False)

        findPathButton = QtWidgets.QPushButton(self.tr("Find path"))
        findPathButton.setFixedSize(120, 50)

        self.view = QtWebEngineWidgets.QWebEngineView()
        self.view.setContentsMargins(25, 25, 25, 25)

        central_widget = QtWidgets.QWidget()
        self.setCentralWidget(central_widget)
        lay = QtWidgets.QHBoxLayout(central_widget)

        button_container = QtWidgets.QWidget()
        self.vlay = QtWidgets.QVBoxLayout(button_container)
        self.vlay.addStretch()
        self.vlay.addWidget(source_label)
        self.vlay.addWidget(self.source_dropdown)
        self.vlay.addWidget(destination_label)
        self.vlay.addWidget(self.destination_dropdown)
        self.vlay.addWidget(toll_label)
        self.vlay.addWidget(self.toll_dropdown)
        hlay = QtWidgets.QHBoxLayout()
        hlay.addWidget(findPathButton)
        self.vlay.addLayout(hlay)
        self.infolay = QtWidgets.QVBoxLayout(self)
        self.vlay.addLayout(self.infolay)
        self.vlay.addStretch()
        lay.addWidget(button_container)
        lay.addWidget(self.view, stretch=1)

        # Connect the findPathButton to rout_path function
        findPathButton.clicked.connect(self.route_path)

    def display_map(self, filename):
        """display map"""
        # Get the current directory
        current_directory = os.path.dirname(os.path.abspath(__file__))
        # Specify the full path to the HTML file
        html_file = os.path.join(current_directory, filename)
        self.view.load(QtCore.QUrl.fromLocalFile(html_file))

    def route_path(self):
        source = self.source_dropdown.itemData(self.source_dropdown.currentIndex())
        destination = self.destination_dropdown.itemData(self.destination_dropdown.currentIndex())
        toll = self.toll_dropdown.itemData(self.toll_dropdown.currentIndex())
        speed = 60.0  # Example speed value (adjust as needed)
        road_type = 'city'  # Example road type (adjust as needed)
        vehicle_type = 'sedan'  # Example vehicle type (adjust as needed)

        # Set the origin and target geocoordinate from which the paths are calculated
        origin_point = (source[0], source[1])
        target_point = (destination[0], destination[1])

        # this is to generate route WITH toll
        routes = generating_path(origin_point, target_point, toll, speed, road_type, vehicle_type)
        plot_map(origin_point, target_point, routes)

        # remove existing widget
        while self.infolay.count():
            item = self.infolay.takeAt(0)
            widget = item.widget()
            if widget:
                widget.deleteLater()

        # swap the sequence back
        routes[0], routes[1] = routes[1], routes[0]

        for index, route in enumerate(routes):
            if index == 0:
                self.red_label = QLabel("<b>Optimal Route in RED:</b>", self)
                self.infolay.addWidget(self.red_label)
            else:
                self.grey_label = QLabel("<b>Alternate Route in GREY:</b>", self)
                self.infolay.addWidget(self.grey_label)

            _, _, total_dist, cumulative_time, total_cost, fuel_consumption = route
            self.label_time = QLabel(f"Estimated Time: {cumulative_time} min")
            self.label_distance = QLabel(f"Estimated Distance: {total_dist} km")
            self.label_cost = QLabel(f"Estimated Cost: ${total_cost}")
            self.label_fuel = QLabel(f"Estimated Fuel Consumption: {fuel_consumption} liters")
            self.infolay.addWidget(self.label_time)
            self.infolay.addWidget(self.label_distance)
            self.infolay.addWidget(self.label_cost)
            self.infolay.addWidget(self.label_fuel)

        self.display_map('plot.html')

    # def route_path(self):
    #     source = self.source_dropdown.itemData(self.source_dropdown.currentIndex())
    #     destination = self.destination_dropdown.itemData(self.destination_dropdown.currentIndex())
    #     toll = self.toll_dropdown.itemData(self.toll_dropdown.currentIndex())
    #     # Set the origin and target geocoordinate from which the paths are calculated
    #     origin_point = (source[0], source[1])
    #     target_point = (destination[0], destination[1])
    #
    #     # this is to generate route WITH toll
    #     routes = generating_path(origin_point, target_point, toll)
    #     plot_map(origin_point, target_point, routes)
    #
    #     # remove existing widget
    #     while self.infolay.count():
    #         item = self.infolay.takeAt(0)
    #         widget = item.widget()
    #         if widget:
    #             widget.deleteLater()
    #
    #     # swap the sequence back
    #     routes[0], routes[1] = routes[1], routes[0]
    #
    #     for index, route in enumerate(routes):
    #         if index == 0:
    #             self.red_label = QLabel("Optimal Route in RED:", self)
    #             self.infolay.addWidget(self.red_label)
    #         else:
    #             self.grey_label = QLabel("Alternate Route in GREY:", self)
    #             self.infolay.addWidget(self.grey_label)
    #
    #         _, _, total_dist, cumulative_time, total_cost, fuel_consumption = route
    #         self.label_time = QLabel(f"Estimated Time: {cumulative_time} min")
    #         self.label_distance = QLabel(f"Estimated Distance: {total_dist} km")
    #         self.label_cost = QLabel(f"Estimated Cost: ${total_cost}")
    #         self.label_fuel = QLabel(f"Estimated Fuel Consumption: {fuel_consumption} liters")
    #         self.infolay.addWidget(self.label_time)
    #         self.infolay.addWidget(self.label_distance)
    #         self.infolay.addWidget(self.label_cost)
    #         self.infolay.addWidget(self.label_fuel)
    #
    #     self.display_map('plot.html')


if __name__ == "__main__":
    # create_graph_process = Process(target=create_graph)
    # create_graph_process.start()
    app = QtWidgets.QApplication(sys.argv)
    mainWin = Window()
    mainWin.show()
    sys.exit(app.exec_())
