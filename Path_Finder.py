import networkx as nx
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.image as mpimg
from path_follower import AircraftSimulation

class TaxiwayGraph:
    def __init__(self, vertices, edges):
        self.edges = edges
        self.vertices = vertices
        self.G = nx.Graph()
        self.G.add_nodes_from(vertices)
        for u, v, weight in edges:
            self.G.add_edge(u, v, weight=weight)
            # self.G.add_edge(u, v, tdg=tdg, name=f"Taxiway {u}-{v}")

    def directed_expanded_graph(self):
        D = nx.DiGraph()
        # Add "v in" vertices to D for each vertex v in G
        for v in self.G.nodes():
            D.add_node(f"{v} in")

        # Add "u from v" vertices to D for each connected vertex pair (u, v) in G
        for u in self.G.nodes():
            for v in self.G.neighbors(u):
                D.add_node(f"{u} from {v}")

        # Add directed edges to D based on the original edges in G
        for u, v, attr in self.G.edges(data=True):
            taxiway_name = attr.get('name', '')

            for w in self.G.neighbors(u):
                if w != v:
                    D.add_edge(f"{u} in", f"{v} from {u}", name=taxiway_name)

            for w in self.G.neighbors(v):
                if w != u:
                    D.add_edge(f"{v} in", f"{u} from {v}", name=taxiway_name)
        return D

    def directed_graph(self, D):
        plt.figure(figsize=(12, 8))
        pos = nx.spring_layout(D, seed=0)
        nx.draw(D, pos, with_labels=True, node_size=5000, node_color='lightblue', edge_color='gray', font_size=10)
        edge_labels = nx.get_edge_attributes(D, 'name')
        nx.draw_networkx_edge_labels(D, pos, edge_labels=edge_labels)
        plt.title("Directed Expanded Graph")
        plt.show()
    
    def undirected_graph(self, vertices, edges, pos, image):
        G = nx.Graph()
        G.add_nodes_from(vertices)
        G.add_weighted_edges_from(edges)

        # Draw the graph
        elarge = [(u, v) for (u, v, d) in G.edges(data=True) if d["weight"] > 5]
        esmall = [(u, v) for (u, v, d) in G.edges(data=True) if d["weight"] <= 5]

        # nodes and edges
        plt.figure(figsize=(12, 8))
        plt.imshow(image)
        plt.gca().invert_yaxis()
        nx.draw_networkx_nodes(G, pos, node_size=200)
        nx.draw_networkx_edges(G, pos, edgelist=elarge, width=2)
        nx.draw_networkx_edges(G, pos, edgelist=esmall, width=2, alpha=0.5, edge_color="b", style="solid")
        nx.draw_networkx_labels(G, pos, font_size=5, font_family="sans-serif")   # node labels

        # edge_labels = {(u, v): f"{d['weight']}" for u, v, d in G.edges(data=True)}  # edge weight labels
        # nx.draw_networkx_edge_labels(G, pos, edge_labels=edge_labels)
        plt.title("Undirected Graph")
        plt.show()

    def find_shortest_path_flight(self, start_vertex, end_vertex, pos):
        distance = nx.dijkstra_path(self.G, start_vertex, end_vertex, weight='weight')
        control_points = []
        for i in range(len(distance)):
            control_points.append(pos[distance[i]])
        return distance, control_points

    def create_and_visualize_graph_2(self, vertices, edges, start, finish, shortest_path_flight, pos, image):
        G = nx.DiGraph()
        G.add_nodes_from(vertices)
        G.add_weighted_edges_from(edges)

        elarge = [(u, v) for (u, v, d) in G.edges(data=True) if d["weight"] > 5]
        esmall = [(u, v) for (u, v, d) in G.edges(data=True) if d["weight"] <= 5]

        plt.figure(figsize=(12, 8))
        plt.imshow(image)
        plt.gca().invert_yaxis()
        nx.draw_networkx_nodes(G, pos, node_size=200)
        nx.draw_networkx_edges(G, pos, edgelist=elarge, width=2)
        nx.draw_networkx_edges(G, pos, edgelist=esmall, width=2, alpha=0.5, edge_color="b", style="solid")
        nx.draw_networkx_labels(G, pos, font_size=5, font_family="sans-serif")

        # edge_labels = {(u, v): f"{d['weight']}" for u, v, d in G.edges(data=True)}
        # nx.draw_networkx_edge_labels(G, pos, edge_labels=edge_labels)

        # Draw the shortest path nodes and edges
        path_nodes = shortest_path_flight
        path_edges = list(zip(shortest_path_flight, shortest_path_flight[1:]))

        nx.draw_networkx_nodes(G, pos, nodelist=path_nodes, node_color='r')
        nx.draw_networkx_edges(G, pos, edgelist=path_edges, width=2, edge_color="r", style="solid")

        plt.title(f"Shortest Path from {start} to {finish}")
        plt.show()

    def cardinal_spline(self, points, num_points=100, tension=0.5):
        def tj(ti, pi, pj):
            return ((1 - tension) / 2) * (pj - pi)

        def interpolate(p0, p1, p2, p3, t):
            t2 = t * t
            t3 = t2 * t
            return ((2 * t3 - 3 * t2 + 1) * p1 + (t3 - 2 * t2 + t) * tj(0, p0, p2) + (-2 * t3 + 3 * t2) * p2 + (t3 - t2) * tj(0, p1, p3))

        def interpolate_derivative(p0, p1, p2, p3, t):
            t2 = t * t
            return ((6 * t2 - 6 * t) * p1 + (3 * t2 - 4 * t + 1) * tj(0, p0, p2) +(-6 * t2 + 6 * t) * p2 + (3 * t2 - 2 * t) * tj(0, p1, p3))

        n = len(control_points)
        points = []
        derivatives = []

        for i in range(n - 1):
            p0 = control_points[i - 1] if i > 0 else control_points[i]
            p1 = control_points[i]
            p2 = control_points[i + 1]
            p3 = control_points[i + 2] if i < n - 2 else control_points[i + 1]

            for t in np.linspace(0, 1, num_points, endpoint=False):
                x = interpolate(p0[0], p1[0], p2[0], p3[0], t)
                y = interpolate(p0[1], p1[1], p2[1], p3[1], t)
                dx = interpolate_derivative(p0[0], p1[0], p2[0], p3[0], t)
                dy = interpolate_derivative(p0[1], p1[1], p2[1], p3[1], t)
                points.append((x, y))
                derivatives.append((dx, dy))

        points.append(control_points[-1])
        derivatives.append((0, 0))  # Derivative at the last point can be approximated as zero
        return np.array(points), np.array(derivatives)
    
    def visualize_cardinal_spline(self, points, tension=0.5, image=None):
        spline_curve, derivative_points = self.cardinal_spline(points, tension=tension)
        plt.figure(figsize=(12, 8))
        plt.imshow(image)
        plt.gca().invert_yaxis() 
        # Plot control points
        plt.plot(points[:, 0], points[:, 1], 'ro-', label='Waypoints')
        # Plot spline curve
        plt.plot(spline_curve[:, 0], spline_curve[:, 1], 'b-', label='Cardinal Spline')
        plt.legend()
        plt.title(f'Cardinal Spline Interpolation with Tension = {tension}')
        plt.xlabel('X')
        plt.ylabel('Y')
        plt.show()


# Define vertices and edges for the first graph
vertices = ['Gate 1', 'Gate 2', 'Gate 3', 'Gate 4', 'Gate 5', 'Gate 6', 'Gate 7', 'Gate 8',
            'R10L', 'R10S', 'R28L', 'R28S', 'R5L', 'R5S', 'R23', 
            'CC4', 'CC3', 'CC2', 'CC1', 'BB5', 'BB4', 'ED', 'DB', 'CB', 'BB3', 'AB2', 'AA', 'LK']

edges = [('CC4', 'CC3', 1), ('CC4', 'R10L', 1), ('CC3', 'R10S', 1), ('R10L', 'R10S', 1), ('CC3', 'CC2', 2),
         ('CC2', 'CC1', 2), ('Gate 1', 'CC1', 2), ('CC1', 'CB', 2), ('BB4', 'BB5', 1), ('BB5', 'R5L', 1), 
         ('BB4', 'CB', 3), ('BB4', 'R5S', 1), ('R5L', 'R5S', 1), ('CB', 'BB3', 1), ('CB', 'DB', 1), ('AA', 'R28L', 1),
         ('AA', 'R28S', 1), ('R28L', 'R28S', 1), ('AA', 'AB2', 3), ('Gate 6', 'AB2', 1), ('Gate 7', 'AB2', 1), 
         ('Gate 8', 'R23', 1), ('R23', 'BB3', 5), ('Gate 5', 'DB', 2), ('Gate 2', 'ED', 1), ('Gate 3', 'ED', 1), 
         ('Gate 4', 'ED', 1), ('ED', 'LK', 1), ('LK', 'DB', 1), ('BB3', 'AB2', 3), ('R23', 'AB2', 2)]

pos = {'Gate 1': (1093, 579), 'Gate 2': (1234, 562), 'Gate 3': (1250, 661), 'Gate 4': (1340, 647), 'Gate 5': (1394, 638), 'Gate 6': (1510, 706), 'Gate 7': (1573, 701), 'Gate 8': (1646, 699),
        'CC4': (95, 670), 'CC3': (165, 668), 'CC2': (656, 589), 'CC1': (1090, 524), 'ED': (1304, 613), 'DB': (1393, 564), 'CB': (1320, 481), 'AB2': (1583, 611), 'BB5': (888, 99), 'BB4': (946, 149), 
        'BB3': (1370, 437), 'AA': (1717, 440), 'R10L': (71, 589), 'R10S': (155, 577), 'R28L': (1722, 323), 'R28S': (1652, 338), 'R5L': (927, 50), 'R5S': (989, 105), 'R23': (1734, 742), 'LK': (1355, 567)}

image_raw = mpimg.imread('map.png') # Load the airport map image
image = np.flipud(image_raw)


# start_gate = str(input("Enter the starting gate: "))
# end_runway = str(input("Enter the runway: "))

start_gate = 'Gate 1'
end_runway = 'R10L'

# Create the TaxiwayGraph object
taxiway_graph = TaxiwayGraph(vertices, edges)
D = taxiway_graph.directed_expanded_graph()
# taxiway_graph.directed_graph(D)
# taxiway_graph.undirected_graph(vertices, edges, pos, image)

# Find the shortest path from the starting gate to the runway
shortest_path_flight, control_points = taxiway_graph.find_shortest_path_flight(start_gate, end_runway ,pos)
# print(f"The shortest path from {start_gate} to {end_runway} is: {shortest_path_flight}")
# taxiway_graph.create_and_visualize_graph_2(vertices, edges, start_gate, end_runway, shortest_path_flight, pos, image)
# print(f"The control points are: {control_points}")

tension = 0.25 # Adjust this parameter to control the tension of the spline
detail_points, derivative_points = taxiway_graph.cardinal_spline(control_points, tension=tension)
# taxiway_graph.visualize_cardinal_spline(detail_points, tension=tension, image=image)
# print(detail_points)
# print(derivative_points)

simulation = AircraftSimulation(detail_points)
simulation.run_simulation()
simulation.plot_results_animated()
