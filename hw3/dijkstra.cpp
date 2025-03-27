#include <iostream>
#include <vector>
#include <queue>
#include <unordered_map>
#include <fstream>
#include <sstream>
#include <limits>

using namespace std;

// Class representing the Graph
class Graph {
private:
    unordered_map<int, vector<pair<int, double>>> adjList; // Adjacency list

public:
    // Add an edge to the graph
    void addEdge(int u, int v, double weight) {
        adjList[u].push_back({v, weight});
    }

    // Function to read graph from a file
    void loadGraphFromFile(const string& filename) {
        ifstream file(filename);
        if (!file) {
            cerr << "Error: Could not open file!" << endl;
            return;
        }

        string line;
        getline(file, line); // Skip the header line
        while (getline(file, line)) {
            stringstream ss(line);
            int u, v;
            double weight;
            ss >> u >> v >> weight;
            addEdge(u, v, weight);
        }
    }

    // Dijkstra's algorithm to find shortest paths
    void dijkstra(int src) {
        unordered_map<int, double> dist;
        priority_queue<pair<double, int>, vector<pair<double, int>>, greater<>> pq;

        // Initialize distances to infinity
        for (const auto& node : adjList) {
            dist[node.first] = numeric_limits<double>::infinity();
        }
        
        dist[src] = 0;
        pq.push({0, src});

        while (!pq.empty()) {
            int u = pq.top().second;
            double d = pq.top().first;
            pq.pop();

            if (d > dist[u]) continue; // Skip outdated entries

            for (auto [v, weight] : adjList[u]) {
                if (dist[u] + weight < dist[v]) {
                    dist[v] = dist[u] + weight;
                    pq.push({dist[v], v});
                }
            }
        }

        // Display the shortest paths from source
        cout << "Shortest distances from node " << src << ":\n";
        for (const auto& [node, distance] : dist) {
            cout << "Node " << node << " : " << (distance == numeric_limits<double>::infinity() ? "INF" : to_string(distance)) << "\n";
        }
    }
};

int main() {
    Graph cityGraph;
    cityGraph.loadGraphFromFile("airline_routes.txt");

    int source;
    cout << "Enter the source node: ";
    cin >> source;
    cityGraph.dijkstra(source);
    
    return 0;
}
