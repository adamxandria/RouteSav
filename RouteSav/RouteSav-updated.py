import os
import sys
import plotly.graph_objects as go
import osmnx as ox
from PyQt5 import QtWidgets, QtWebEngineWidgets, QtCore
from PyQt5.QtWidgets import QLabel, QComboBox
from plotly import offline
import heapq
from multiprocessing import Process

MODE = "drive"
NORTH = 1.3701654712520062
SOUTH = 1.2946774207297054
EAST = 103.9907219819978
WEST = 103.87206966924965
PERIMETER = 0.001
ox.settings.log_console = True
ox.settings.use_cache = True


def dijkstra_shortest_path(graph, source, target):
    distances = {node: float('inf') for node in graph}
    distances[source] = 0
    previous_nodes = {node: None for node in graph}

    # Find the source and target
    # print('Source: ', source)
    # print('Target: ',target)

    priority_queue = [(0, source)]
    while priority_queue:
        current_distance, current_node = heapq.heappop(priority_queue)

        # print('current node is : ', current_node)

        if current_node == target:
            break

        if current_distance > distances[current_node]:
            continue

        for neighbor, weight in graph[current_node].items():
            if 'maxspeed' in weight[0]:
                distance = current_distance + weight[0]['length'] + float(weight[0]['maxspeed'])
            else:
                distance = current_distance + weight[0]['length']
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
    return round(cumulative_time * 1.3) # 30% allowance to consider traffic and slower driving, as not possible to drive at max speed all the way


def generating_path(origin_point, target_point):
    """load processed graph and use to calculate optimal route"""
    # create_graph_process.join()
    # Load the pre-processed graph
    graph = ox.load_graphml('preprocessed_graph.graphml')
    # Get the nearest node in the OSMNX graph for the origin point
    origin_node = ox.distance.nearest_nodes(graph, origin_point[1], origin_point[0])

    # Get the nearest node in the OSMNX graph for the target point
    target_node = ox.distance.nearest_nodes(graph, target_point[1], target_point[0])

    # Get the optimal path via dijkstra
    route = dijkstra_shortest_path(graph, origin_node, target_node)
    total_distance = calculate_total_distance(graph, route)
    print(total_distance)
    cumulative_time = calculate_cumulative_time(graph, route)
    print("Cumulative time from A to B:", cumulative_time)
    # Create the arrays for storing the paths
    lat = []
    long = []

    for i in route:
        point = graph.nodes[i]
        long.append(point['x'])
        lat.append(point['y'])

    # Return the paths
    return long, lat, total_distance, cumulative_time


def plot_map(origin_point, target_point, long, lat):
    """plot route onto map"""
    print(origin_point)
    print(target_point)
    print(long)
    print(lat)
    # Create a plotly map and add the origin point to the map
    print("Plotting map...")
    fig = go.Figure(go.Scattermapbox(
        name="Origin",
        mode="markers",
        lon=[long[0]],
        lat=[lat[0]],
        showlegend=False,
        marker={'size': 16, 'color': "#333333"}
    )
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
        lon=[long[-1]],
        lat=[lat[-1]],
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
                          'zoom': 12.5}
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
        self.source_dropdown.addItem("ibis budget Singapore Pearl", [1.3117510023367127, 103.87940230507937])
        self.source_dropdown.addItem("Min Wah Hotel", [1.312324970031862, 103.8824107783044])
        self.source_dropdown.addItem("Amrise Hotel", [1.3139710326135319, 103.87786884865685])

        # Create the "Destination" title label
        destination_label = QLabel("Destination", self)

        # Create the destination dropdown
        self.destination_dropdown = QComboBox(self)
        self.destination_dropdown.addItem("Changi Airport Terminal 3", [1.350401, 103.9850091])
        self.destination_dropdown.addItem("ibis budget Singapore Pearl", [1.3117510023367127, 103.87940230507937])
        self.destination_dropdown.addItem("Min Wah Hotel", [1.312324970031862, 103.8824107783044])
        self.destination_dropdown.addItem("Amrise Hotel", [1.3139710326135319, 103.87786884865685])
        findPathButton = QtWidgets.QPushButton(self.tr("Find path"))
        findPathButton.setFixedSize(120, 50)

        #display estimated time and distance
        self.info = QtWidgets.QVBoxLayout(self)
        self.label_time = QLabel("Estimated Time: -")
        self.label_distance = QLabel("Estimated Distance: -")
        self.info.addWidget(self.label_time)
        self.info.addWidget(self.label_distance)

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
        # Set the origin and target geocoordinate from which the paths are calculated
        origin_point = (source[0], source[1])
        target_point = (destination[0], destination[1])

        long, lat, total_dist, cumulative_time = generating_path(origin_point, target_point)

        plot_map(origin_point, target_point, long, lat)
        self.label_time.setText(f"Estimated Time: {cumulative_time} min")
        self.label_distance.setText(f"Estimated Distance: {total_dist} km")
        self.display_map('plot.html')


if __name__ == "__main__":
    create_graph_process = Process(target=create_graph)
    create_graph_process.start()
    App = QtWidgets.QApplication(sys.argv)
    window = Window()
    window.show()
    sys.exit(App.exec())
