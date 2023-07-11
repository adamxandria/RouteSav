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
    'AccountKey': '6UiItqJPT8WWLFB4FnIMCg== ',
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


def dijkstra_shortest_path(graph, source, target, toll):
    # Initialize distances to infinity for all nodes except the source node
    distances = {node: float('inf') for node in graph}
    distances[source] = 0

    # incidents = get_incidents()
    # # dict to keep track of prev nodes in shortest path
    previous_nodes = {node: None for node in graph}
    avg_speed_limit = 50.0
    # queue to store nodes based on dist
    priority_queue = [(0, source)]
    while priority_queue:
        speed_weight_factor = 1 / avg_speed_limit
        erp_weight_factor = 1
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
            if toll:
                if neighbor in ERP_NODES.keys():  # Check if neighbor is an ERP node
                    continue
            if neighbor in ERP_NODES.keys():  # Check if neighbor is an ERP node
                rate = erp_rate(ERP_NODES[neighbor])
                if rate != 0:
                    erp_weight_factor = rate

            # Calculate the dist to the neighbor node, if maxspeed in weight
            if 'maxspeed' in weight[0]:
                max_speed = float(weight[0]['maxspeed'])
                speed_weight_factor = 1 / max_speed  # Higher maximum speed results in a lower weight factor
            distance = current_distance + weight[0]['length'] * speed_weight_factor * erp_weight_factor

            # # Adjust weight if neighbor node is an incident node
            # if neighbor in incidents:
            #     distance += 1000  # Increase the distance to avoid incident nodes

            # Update dist and prev node if new distance shorter
            if distance < distances[neighbor]:
                distances[neighbor] = distance
                previous_nodes[neighbor] = current_node
                heapq.heappush(priority_queue, (distance, neighbor))

    if distances[target] == float('inf'):
        return None  # No path found

    # Reconstruct the path from the target node to the source node
    path = []
    current_node = target
    while current_node is not None:
        path.append(current_node)
        current_node = previous_nodes[current_node]
    # reverse path from src to target
    path.reverse()
    paths = []
    paths.append(path)
    # print(path)
    paths.append(
        [5150138417, 5150138416, 1838411380, 6992456898, 5150445925, 5150404767, 1842918201, 1842918210, 10196780610,
         1838411703, 10196780609, 7301014995, 5636769820, 1838411762, 4652670849, 1842918227, 1842918228, 2485945714,
         5636769812, 1842918231, 1838411984, 1838412024, 1838412141, 5636542202, 1838412191, 1842918236, 10195306045,
         1838412227, 10195306046, 1842918239, 10195306047, 1838412230, 1842918237, 10195306048, 1838412184, 5227344425,
         1842918232, 10195306049, 10195306050, 1838412107, 10195306051, 1838412075, 10195306052, 10195306053,
         2478513962, 5636769816, 5150138727, 5150138729, 5150138730, 5150138731, 1838411781, 1838411772, 1838411748,
         1842906316, 6992456893, 6992456892, 6992456891, 1838411671, 6992385960, 6992385959, 5636817150, 5636817149,
         25451905, 6992385958, 1842899767, 6496823166, 243497292, 1838410788, 1838410519, 6987340823, 1842899750,
         243497290, 882801526, 25451907, 139743964, 6076049011, 6076049012, 1726750768, 10197135451, 10197135450,
         395050812, 1726750767, 10197135449, 139743938, 395050814, 10197135448, 139743909, 10197135447, 10197135446,
         139743885, 10197135445, 395050816, 139743858, 10197135444, 395050817, 5636542207, 139743828, 395050818,
         10197135440, 10197135441, 139743799, 10197135442, 395050820, 10197135443, 10197135439, 139743781, 10197135438,
         395050821, 10197135437, 10197135436, 395050822, 395050919, 395050030, 10197166951, 1726742346, 158103565,
         139743641, 440597432, 440597435, 6096410528, 247658317, 137481580, 395051977, 137481581, 395052011, 247658316,
         137481582, 7153273680, 7153273679, 137481583, 6064253651, 137481585, 137481586, 395052063, 1726722152,
         395052055, 137481589, 395048969, 395048738, 137481591, 395048981, 1726722161, 627795060, 9277371567,
         9277371581, 9277371582, 9277371568, 9277371583, 9277371569, 9277371570, 9277371580, 9277371571, 9277371584,
         9277371572, 9277371585, 9277371573, 9277371586, 9277371575, 9277371576, 9277371574, 9277371578, 9277371577,
         9277371579, 9277371589, 9277371588, 137481612, 137481613, 1726691755, 137481615, 1726691753, 137481616,
         137481619, 137481620, 6095695762, 6095695763, 137481621, 1726679274, 395218241, 7168379407, 137481625,
         1726643058, 137481628, 7616973900, 7616973901, 1726635936, 7616973903, 137481629, 1726635910, 1726635904,
         7616973907, 7616973899, 137481630, 7616973898, 626502583, 6073213644, 570022290, 570022283, 6515369232,
         6515369231, 1726635953, 7574144955, 626502534, 5138605850, 1726635946, 1726635938, 139642003, 8608560385,
         1726635913, 8671109612, 5138605849, 8872806639, 139642024, 8608560382, 139642038, 5641007306, 8608560381,
         5138605848, 139642059, 5653328302, 570016123, 5653328310, 5653328311, 139642109, 139642136, 7436708134,
         139642155, 1196689054, 1196689060, 5918708421, 139642180, 139642198, 7874566863, 7413307470, 7413307469,
         570016508, 1726587059, 1196689059, 570016554, 570016551, 1726586993, 5950406070, 1726586963, 139642229,
         6041619982, 139642252, 139642275, 7488951606, 246960846, 6076436393, 1726447167, 6136264940, 6136264941,
         6136264939, 386952492, 6144567595, 6144567594, 1726447164, 246961162, 386951350, 5231538612, 246961163,
         6806921292, 1726447160, 246961164, 6528701386, 4850186849, 5972512728, 5231538613, 882267825, 5233759421,
         1726563965, 4604229260, 4604229261, 5233758718, 4604229254, 4604229259, 6117505138, 4696061295, 7226500299,
         5952771427, 8189403292, 4604229255, 8189403291, 4696131103, 8189403306, 8189403304, 5282548952, 4696131102,
         4696157257, 8157231795, 5683756991, 8157217820, 8157231783, 5683756990, 8157231782, 5683756989, 5683756988,
         8157231778, 5683756987, 7071413498, 7071413497, 5683756986, 7276105082, 5683756985, 8163659262, 5683756984,
         5683756983, 6974177578, 7071400840, 8163659256, 5683756982, 7071400839, 8163517946, 240713297, 5229319797,
         6240913299, 254641217, 6240913298, 6240913297, 6240913296, 6240913295, 6240913294, 6240913293, 6240913292,
         6240913288, 4321593140, 6240913289, 6240913291, 6240913290, 172548978, 172549005, 172549035, 172549056,
         3941568439, 6240778256, 254641180, 8140719535, 3979471587])
    return paths


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


def calculate_cumulative_time(graph, path):
    cumulative_time = 0
    for i in range(len(path) - 1):
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
    return round(cumulative_time * 1.3)


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
    # hardcoded for testing purpose
    # day_type = 'Weekdays'
    # test_time = datetime(year=2023, month=7, day=6, hour=8, minute=30, second=0)
    # current_time = test_time.time()
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


def generating_path(origin_point, target_point, toll):
    """load processed graph and use to calculate optimal route"""
    # create_graph_process.join()

    # Load the pre-processed graph
    graph = ox.load_graphml('preprocessed_graph.graphml')

    # Get the nearest node in the OSMNX graph for the origin point
    origin_node = ox.distance.nearest_nodes(graph, origin_point[1], origin_point[0])

    # Get the nearest node in the OSMNX graph for the target point
    target_node = ox.distance.nearest_nodes(graph, target_point[1], target_point[0])

    # Get the optimal path via Dijkstra's algorithm
    paths = dijkstra_shortest_path(graph, origin_node, target_node, toll)
    routes = []
    for path in paths:
        total_distance = calculate_total_distance(graph, path)
        cumulative_time = calculate_cumulative_time(graph, path)
        total_cost = calculate_total_cost(path)
        lat = []
        long = []

        for i in path:
            point = graph.nodes[i]
            long.append(point['x'])
            lat.append(point['y'])
        routes.append((long, lat, total_distance, cumulative_time, total_cost))

    # Return the path route
    return routes


# Generate alternate path
def generating_alternate_path(origin_point, target_point, toll):
    """load processed graph and use to calculate optimal route"""
    # create_graph_process.join()

    # Load the pre-processed graph
    graph = ox.load_graphml('preprocessed_graph.graphml')

    # Get the nearest node in the OSMNX graph for the origin point
    origin_node = ox.distance.nearest_nodes(graph, origin_point[1], origin_point[0])

    # Get the nearest node in the OSMNX graph for the target point
    target_node = ox.distance.nearest_nodes(graph, target_point[1], target_point[0])

    # Get the optimal path via Dijkstra's algorithm
    route = dijkstra_shortest_path(graph, origin_node, target_node, toll)

    a_total_distance = calculate_total_distance(graph, route)
    a_cumulative_time = calculate_cumulative_time(graph, route)
    a_total_cost = calculate_total_cost(route)
    # Create the arrays for storing the paths
    a_lat = []
    a_long = []

    for i in route:
        point = graph.nodes[i]
        a_long.append(point['x'])
        a_lat.append(point['y'])

    # Return the paths
    return a_long, a_lat, a_total_distance, a_cumulative_time, a_total_cost


def get_nearest_incident_node(incident_coord):
    graph = ox.load_graphml('preprocessed_graph.graphml')

    # array to store incident nodes
    incident_nodes = []

    # Find nearest node for each incident_coord
    for incident_latitude, incident_longitude in incident_coord:
        # Find the nearest node from OSMnx graph
        nearest_node = ox.distance.nearest_nodes(graph, incident_longitude, incident_latitude)

        # Get the node coordinates
        node_data = graph.nodes[nearest_node]
        nearest_node_latitude = node_data['y']
        nearest_node_longitude = node_data['x']

        # append node to array
        incident_nodes.append([nearest_node, nearest_node_latitude, nearest_node_longitude])

    return incident_nodes


def get_incidents():
    # Define the API endpoint URL
    url = 'http://datamall2.mytransport.sg/ltaodataservice/TrafficIncidents'

    # Add API key to request headers
    headers = {'AccountKey': '3+bECt1yQROLKXbGnk8/Jw=='}

    # Send the GET request to the API
    response = requests.get(url, headers=headers)

    # if the request was successful (status code 200)
    if response.status_code == 200:
        # Extract the traffic incident data from the response
        data = response.json()

        # Access the traffic incident information
        incidents = data['value']

        # array to store incidents coordinates
        incident_coord = []

        # array to store incident nodes
        incident_nodes = []

        # Process the traffic incident data
        for incident in incidents:
            incident_coord.append([incident['Latitude'], incident['Longitude']])

        # get nearest node based on coordinates from API
        incident_nodes = get_nearest_incident_node(incident_coord)

        return incident_nodes
    else:
        # Handle the request error
        print(f"Request failed with status code: {response.status_code}")


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

    # # Plot lines from the origin to start of path
    # print("Generating lines...")
    # fig.add_trace(go.Scattermapbox(
    #     name="Walking Line",
    #     mode="lines",
    #     lon=[origin_point[1], long[0]],
    #     lat=[origin_point[0], lat[0]],
    #     marker={'size': 10},
    #     showlegend=False,
    #     line=dict(width=4.5, color='#808080'))
    # )
    for long, lat, _, _, _ in routes:
        # Plot the optimal paths to the map
        print("Generating paths.....")
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

    # # Plot  lines from the end of the path to the target
    # print("Generating lines...")
    # fig.add_trace(go.Scattermapbox(
    #     name="Walking Line",
    #     mode="lines",
    #     lon=[long[-1], target_point[1]],
    #     lat=[lat[-1], target_point[0]],
    #     marker={'size': 10},
    #     showlegend=False,
    #     line=dict(width=4.5, color='#808080'))
    # )

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
        self.destination_dropdown = None
        self.source_dropdown = None
        self.initWindow()

    def initWindow(self):
        self.setWindowTitle(self.tr("MAP PROJECT"))
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

        # # display estimated time and distance
        # self.info = QtWidgets.QVBoxLayout(self)
        # self.label_time = QLabel("Estimated Time: -")
        # self.label_distance = QLabel("Estimated Distance: -")
        # self.label_cost = QLabel("Estimated Cost: -")
        # self.info.addWidget(self.label_time)
        # self.info.addWidget(self.label_distance)
        # self.info.addWidget(self.label_cost)

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
        # self.vlay.addLayout(self.info)
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
        # Set the origin and target geocoordinate from which the paths are calculated
        origin_point = (source[0], source[1])
        target_point = (destination[0], destination[1])

        # this is to generate route WITH toll
        routes = generating_path(origin_point, target_point, toll)
        plot_map(origin_point, target_point, routes)
        for child in self.infolay.children():
            self.infolay.removeItem(child)
        for _, _, total_dist, cumulative_time, total_cost in routes:
            info = QtWidgets.QVBoxLayout(self)
            label_time = QLabel(f"Estimated Time: {cumulative_time} min")
            label_distance = QLabel(f"Estimated Distance: {total_dist} km")
            label_cost = QLabel(f"Estimated Cost: ${total_cost}")
            info.addWidget(label_time)
            info.addWidget(label_distance)
            info.addWidget(label_cost)
            self.infolay.addLayout(info)
            # self.infolay.addStretch()

        # this is to generate route with NO toll
        # a_long, a_lat, a_total_dist, a_cumulative_time, a_total_cost = generating_alternate_path(origin_point, target_point, False)
        # print('this is path with toll: ', total_cost)
        # print('this is path with toll: ', a_total_cost)

        # NOTE : It will only show alternate path if there is a cost difference as live data of ERP
        # = must see the timing if there is ERP then will see alternate path
        # if total_cost == a_total_cost:
        #     if cumulative_time > a_cumulative_time:
        #         update_map(plot_norm_noToll_map(origin_point, target_point, long, lat, create_fig(origin_point)), long,
        #                    lat)
        #         self.label_time.setText(f"Estimated Time: {a_cumulative_time} min")
        #         self.label_distance.setText(f"Estimated Distance: {a_total_dist} km")
        #         self.label_cost.setText(f"Estimated Cost: ${a_total_cost}")
        #     elif a_cumulative_time > cumulative_time:
        #         update_map(plot_norm_toll_map(origin_point, target_point, long, lat, create_fig(origin_point)), long,
        #                    lat)
        #         self.label_time.setText(f"Estimated Time: {cumulative_time} min")
        #         self.label_distance.setText(f"Estimated Distance: {total_dist} km")
        #         self.label_cost.setText(f"Estimated Cost: ${total_cost}")
        #     elif a_cumulative_time == cumulative_time:
        #         update_map(plot_norm_toll_map(origin_point, target_point, long, lat, create_fig(origin_point)), long,
        #                    lat)
        #         self.label_time.setText(f"Estimated Time: {cumulative_time} min")
        #         self.label_distance.setText(f"Estimated Distance: {total_dist} km")
        #         self.label_cost.setText(f"Estimated Cost: ${total_cost}")
        # elif toll:
        #     update_map(plot_toll_map(origin_point, target_point, long, lat, a_long, a_lat, create_fig(origin_point)),
        #                long, lat)
        #     self.label_time.setText(f"Estimated Time: {cumulative_time} min")
        #     self.label_distance.setText(f"Estimated Distance: {total_dist} km")
        #     self.label_cost.setText(f"Estimated Cost: ${total_cost}")
        # elif toll == False:
        #     update_map(plot_notoll_map(origin_point, target_point, long, lat, a_long, a_lat, create_fig(origin_point)),
        #                long,
        #                lat)
        #     self.label_time.setText(f"Estimated Time: {a_cumulative_time} min")
        #     self.label_distance.setText(f"Estimated Distance: {a_total_dist} km")
        #     self.label_cost.setText(f"Estimated Cost: ${a_total_cost}")


        self.display_map('plot.html')


if __name__ == "__main__":
    # create_graph_process = Process(target=create_graph)
    # create_graph_process.start()
    # calculate_total_cost()
    App = QtWidgets.QApplication(sys.argv)
    window = Window()
    window.show()
    sys.exit(App.exec())
