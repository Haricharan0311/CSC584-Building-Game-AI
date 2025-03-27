#include <iostream>
#include <vector>
#include <queue>
#include <unordered_map>
#include <fstream>
#include <sstream>
#include <limits>
#include <chrono>  

using namespace std;
using namespace std::chrono;  


class Graph {
private:
    unordered_map<int, vector<pair<int, double>>> adjList; 
public:
    void addEdge(int u, int v, double weight) {
        adjList[u].push_back({v, weight});
    }

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

    // Dijkstra's algorithm to find shortest paths and compute metrics
    void dijkstra(int src, int goal) {
        unordered_map<int, double> dist;
        priority_queue<pair<double, int>, vector<pair<double, int>>, greater<>> pq;
        
        int max_fringe = 0; 
        int total_fill = 0;  
        int nodes_processed = 0;

        // Start timing the algorithm
        auto start_time = high_resolution_clock::now();

        for (const auto& node : adjList) {
            dist[node.first] = numeric_limits<double>::infinity();
        }
        
        dist[src] = 0;
        pq.push({0, src});

        while (!pq.empty()) {
            int u = pq.top().second;
            double d = pq.top().first;
            pq.pop();
            nodes_processed++;
            if (d > dist[u]) continue; 

            // Track max fringe size
            if (pq.size() > max_fringe) {
                max_fringe = pq.size();
            }

            for (auto [v, weight] : adjList[u]) {
                if (dist[u] + weight < dist[v]) {
                    dist[v] = dist[u] + weight;
                    pq.push({dist[v], v});
                    total_fill++; 
                }
            }
        }

        auto end_time = high_resolution_clock::now();
        auto duration = duration_cast<milliseconds>(end_time - start_time);

        double sum_distances = 0.0;
        int reachable_nodes = 0;

        cout << "Shortest distances from node " << src << ":\n";
        for (const auto& [node, distance] : dist) {
            if (node != src && distance != numeric_limits<double>::infinity()) {
                sum_distances += distance;
                reachable_nodes++;
            }
            if (node == goal) {
                cout << "Shortest distance from node " << src << " to node " << goal << " is " << distance << "\n";
            }
            //cout << "Node " << node << " : " << (distance == numeric_limits<double>::infinity() ? "INF" : to_string(distance)) << "\n";
        }

        if (reachable_nodes > 0) {
            double avg_distance = sum_distances / reachable_nodes;
            cout << "\nAverage shortest path distance from node " << src << " to all reachable nodes: " << avg_distance << "\n";
        } else {
            cout << "\nNo reachable nodes from source " << src << ".\n";
        }

        cout << "\nAlgorithm Metrics:\n";
        cout << "Maximum Fringe Size: " << max_fringe << "\n";
        cout << "Total Fill (distance updates): " << total_fill << "\n";
        cout << "Total Execution Time: " << duration.count() << " ms\n";  // Output execution time in milliseconds
        cout << "Nodes Processed: " << nodes_processed << "\n";
    }
};

int main(int argc, char* argv[]) {
    if (argc < 2) {
        cerr << "Usage: " << argv[0] << " <graph_file>" << endl;
        return 1;
    }
    
    string graphFile = argv[1];
    Graph cityGraph;
    cityGraph.loadGraphFromFile(graphFile);

    int source;
    int goal;
    cout << "Enter the source node: ";
    cin >> source;
    cout << "Enter the goal node: ";
    cin >> goal;
    cityGraph.dijkstra(source, goal);
    
    return 0;
}
