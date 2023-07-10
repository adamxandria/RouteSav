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
    distances = {node: float('inf') for node in graph}
    distances[source] = 0
    previous_nodes = {node: None for node in graph}
    avg_speed_limit = 50.0
    priority_queue = [(0, source)]
    while priority_queue:
        speed_weight_factor = 1 / avg_speed_limit
        erp_weight_factor = 1
        current_distance, current_node = heapq.heappop(priority_queue)

        if current_node == target:
            break

        if current_distance > distances[current_node]:
            continue

        for neighbor, weight in graph[current_node].items():
            if toll:
                if neighbor in ERP_NODES.keys():  # Check if neighbor is an ERP node
                    continue
            if neighbor in ERP_NODES.keys():  # Check if neighbor is an ERP node
                rate = erp_rate(ERP_NODES[neighbor])
                if rate != 0:
                    erp_weight_factor = rate
            if 'maxspeed' in weight[0]:
                max_speed = float(weight[0]['maxspeed'])
                speed_weight_factor = 1 / max_speed  # Higher maximum speed results in a lower weight factor
            distance = current_distance + weight[0]['length'] * speed_weight_factor * erp_weight_factor
            if distance < distances[neighbor]:
                distances[neighbor] = distance
                previous_nodes[neighbor] = current_node
                heapq.heappush(priority_queue, (distance, neighbor))

    if distances[target] == float('inf'):
        return None  # No path found

    path = []
    current_node = target
    while current_node is not None:
        path.append(current_node)
        current_node = previous_nodes[current_node]
    path.reverse()

    return path


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
    route = dijkstra_shortest_path(graph, origin_node, target_node, toll)

    total_distance = calculate_total_distance(graph, route)
    cumulative_time = calculate_cumulative_time(graph, route)
    total_cost = calculate_total_cost(route)
    # Create the arrays for storing the paths
    lat = []
    long = []

    for i in route:
        point = graph.nodes[i]
        long.append(point['x'])
        lat.append(point['y'])

    # Return the path route
    return long, lat, total_distance, cumulative_time, total_cost

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









def create_fig(origin_point):
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
    return fig


def plot_toll_map(origin_point, target_point, long, lat, a_long, a_lat, fig):
    """plot route onto map"""
    print(origin_point)
    print(target_point)
    print(long)
    print(lat)

    # Plot  lines from the end of the path to the target
    print("Generating lines...")
    fig.add_trace(go.Scattermapbox(
        name="Walking Line",
        mode="lines",
        lon=[origin_point[1], long[0]],
        lat=[origin_point[0], lat[0]],
        marker={'size': 10},
        showlegend=False,
        line=dict(width=4.5, color='#808080'))
    )

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

    # erp path
    # For plotting non_erp route        -----
    print(origin_point)
    print(target_point)
    print(a_long)
    print(a_lat)
    
    fig.add_trace(go.Scattermapbox(
        name="Walking Line",
        mode="lines",
        lon=[origin_point[1], a_long[0]],
        lat=[origin_point[0], a_lat[0]],
        marker={'size': 10},
        showlegend=False,
        line=dict(width=4.5, color='#CCCCCC'))
    )

    # Plot the optimal paths to the map
    print("Generating paths.....")
    fig.add_trace(go.Scattermapbox(
        name="Path",
        mode="lines",
        lon=a_long,
        lat=a_lat,
        marker={'size': 10},
        showlegend=False,
        line=dict(width=4.5, color='#CCCCCC'))
    )

    # Plot the target geocoordinates to the map
    print("Generating target...")
    fig.add_trace(go.Scattermapbox(
        name="Destination",
        mode="markers",
        showlegend=False,
        lon=[target_point[1]],
        lat=[target_point[0]],
        marker={'size': 16, 'color': '#CCCCCC'}))

    return fig


def plot_notoll_map(origin_point, target_point, long, lat, a_long, a_lat, fig):
    """plot route onto map"""
    print(origin_point)
    print(target_point)
    print(a_long)
    print(a_lat)

    # Plot for non-erp path
    # Plot lines from the end of the path to the target
    print("Generating lines...")
    fig.add_trace(go.Scattermapbox(
        name="Walking Line",
        mode="lines",
        lon=[origin_point[1], a_long[0]],
        lat=[origin_point[0], a_lat[0]],
        marker={'size': 10},
        showlegend=False,
        line=dict(width=4.5, color='#808080'))
    )

    # Plot the optimal paths to the map
    print("Generating paths.....")
    fig.add_trace(go.Scattermapbox(
        name="Path",
        mode="lines",
        lon=a_long,
        lat=a_lat,
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


    
    print(origin_point)
    print(target_point)
    print(long)
    print(lat)

    # For plotting erp route -----
    # Plot lines from the end of the path to the target
    fig.add_trace(go.Scattermapbox(
        name="Walking Line",
        mode="lines",
        lon=[origin_point[1], long[0]],
        lat=[origin_point[0], lat[0]],
        marker={'size': 10},
        showlegend=False,
        line=dict(width=4.5, color='#CCCCCC'))
    )

    # Plot the optimal paths to the map
    print("Generating paths.....")
    fig.add_trace(go.Scattermapbox(
        name="Path",
        mode="lines",
        lon=long,
        lat=lat,
        marker={'size': 10},
        showlegend=False,
        line=dict(width=4.5, color='#CCCCCC'))
    )

    # Plot the target geocoordinates to the map
    print("Generating target...")
    fig.add_trace(go.Scattermapbox(
        name="Destination",
        mode="markers",
        showlegend=False,
        lon=[target_point[1]],
        lat=[target_point[0]],
        marker={'size': 16, 'color': '#CCCCCC'}))
    
    return fig

def plot_normal_map(origin_point, target_point, long, lat,fig):
    """plot route onto map"""
    print(origin_point)
    print(target_point)
    print(long)
    print(lat)

    # Plot for non-erp path
    # Plot lines from the end of the path to the target
    print("Generating lines...")
    fig.add_trace(go.Scattermapbox(
        name="Walking Line",
        mode="lines",
        lon=[origin_point[1], long[0]],
        lat=[origin_point[0], lat[0]],
        marker={'size': 10},
        showlegend=False,
        line=dict(width=4.5, color='#808080'))
    )

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
    
    return fig

def update_map(fig, long, lat):
    # Style the map layout
    print('update map')
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

        # display estimated time and distance
        self.info = QtWidgets.QVBoxLayout(self)
        self.label_time = QLabel("Estimated Time: -")
        self.label_distance = QLabel("Estimated Distance: -")
        self.label_cost = QLabel("Estimated Cost: -")
        self.info.addWidget(self.label_time)
        self.info.addWidget(self.label_distance)
        self.info.addWidget(self.label_cost)

        self.view = QtWebEngineWidgets.QWebEngineView()
        self.view.setContentsMargins(25, 25, 25, 25)

        central_widget = QtWidgets.QWidget()
        self.setCentralWidget(central_widget)
        lay = QtWidgets.QHBoxLayout(central_widget)

        button_container = QtWidgets.QWidget()
        vlay = QtWidgets.QVBoxLayout(button_container)
        vlay.addStretch()
        vlay.addWidget(source_label)
        vlay.addWidget(self.source_dropdown)
        vlay.addWidget(destination_label)
        vlay.addWidget(self.destination_dropdown)
        vlay.addWidget(toll_label)
        vlay.addWidget(self.toll_dropdown)
        hlay = QtWidgets.QHBoxLayout()
        hlay.addWidget(findPathButton)
        vlay.addLayout(hlay)
        vlay.addLayout(self.info)
        vlay.addStretch()
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
        long, lat, total_dist, cumulative_time, total_cost = generating_path(origin_point, target_point, True)
        # this is to generate route with NO toll
        a_long, a_lat, a_total_dist, a_cumulative_time, a_total_cost = generating_alternate_path(origin_point, target_point, False)
        print('this is path with toll: ', total_cost)
        print('this is path with toll: ', a_total_cost)

        # NOTE : It will only show alternate path if there is a cost difference as live data of ERP 
        # = must see the timing if there is ERP then will see alternate path
        if total_cost == a_total_cost:
            update_map(plot_normal_map(origin_point, target_point, long, lat,create_fig(origin_point)), long, lat)
            self.label_time.setText(f"Estimated Time: {cumulative_time} min")
            self.label_distance.setText(f"Estimated Distance: {total_dist} km")
            self.label_cost.setText(f"Estimated Cost: ${total_cost}")
        elif toll:
            update_map(plot_toll_map(origin_point, target_point, long, lat, a_long, a_lat, create_fig(origin_point)), long, lat)
            self.label_time.setText(f"Estimated Time: {cumulative_time} min")
            self.label_distance.setText(f"Estimated Distance: {total_dist} km")
            self.label_cost.setText(f"Estimated Cost: ${total_cost}")
        elif toll == False:
            update_map(plot_notoll_map(origin_point, target_point, long, lat, a_long, a_lat, create_fig(origin_point)), long, lat)
            self.label_time.setText(f"Estimated Time: {a_cumulative_time} min")
            self.label_distance.setText(f"Estimated Distance: {a_total_dist} km")
            self.label_cost.setText(f"Estimated Cost: ${a_total_cost}")

        self.display_map('plot.html')


if __name__ == "__main__":
    # create_graph_process = Process(target=create_graph)
    # create_graph_process.start()
    # calculate_total_cost()
    App = QtWidgets.QApplication(sys.argv)
    window = Window()
    window.show()
    sys.exit(App.exec())


# put the grey and red drawing in one function thennnnnnnnnn do if else  to choose which seq