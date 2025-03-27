import networkx as nx
import random

# Parameters
num_vertices = 20000
num_edges = random.randint(80000, 100000)  # Between 3x and 4x the number of vertices

# Create an empty graph
G = nx.Graph()

# Add vertices
G.add_nodes_from(range(num_vertices))

# Add random weighted edges
edges_added = set()
while len(edges_added) < num_edges:
    u, v = random.sample(range(num_vertices), 2)  # Pick two distinct nodes
    if (u, v) not in edges_added and (v, u) not in edges_added:
        weight = random.randint(1, 100)  # Assign a random weight between 1 and 100
        G.add_edge(u, v, weight=weight)
        edges_added.add((u, v))

# Ensure the graph is connected (if needed)
if not nx.is_connected(G):
    # Connect components using a minimum spanning tree approach
    components = list(nx.connected_components(G))
    for i in range(len(components) - 1):
        node1 = random.choice(list(components[i]))
        node2 = random.choice(list(components[i + 1]))
        G.add_edge(node1, node2, weight=random.randint(1, 100))

# Graph statistics
num_components = nx.number_connected_components(G)
avg_degree = sum(dict(G.degree()).values()) / num_vertices

# Display results
graph_stats = {
    "Number of vertices": num_vertices,
    "Number of edges": G.number_of_edges(),
    "Number of connected components": num_components,
    "Average degree": avg_degree
}

print(graph_stats)

# Define the output file path
output_file = "random_graph.txt"

# Write the graph to a text file in the format: node1 node2 edgeweight
with open(output_file, "w") as f:
    for u, v, data in G.edges(data=True):
        f.write(f"{u} {v} {data['weight']}\n")

# Confirm the file has been created
output_file