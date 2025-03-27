#include <iostream>
#include <vector>
#include <queue>
#include <unordered_map>
#include <unordered_set>
#include <fstream>
#include <sstream>
#include <limits>
#include <cstdlib>
#include <algorithm>
#include <chrono>

using namespace std;
using namespace std::chrono;

class Graph {
private:
    unordered_map<int, vector<pair<int, double>>> adjList;
    unordered_map<int, unordered_map<int, double>> allPairsShortestPaths;
    unordered_map<int, double> minOutgoingEdge;
    vector<int> landmarks;

public:
    void addEdge(int u, int v, double weight) {
        adjList[u].push_back({v, weight});
        if (minOutgoingEdge.find(u) == minOutgoingEdge.end()) {
            minOutgoingEdge[u] = weight;
        } else {
            minOutgoingEdge[u] = min(minOutgoingEdge[u], weight);
        }
    }

    void loadGraphFromFile(const string &filename) {
        ifstream file(filename);
        if (!file) {
            cerr << "Error: Could not open file!" << endl;
            return;
        }

        string line;
        getline(file, line);
        while (getline(file, line)) {
            stringstream ss(line);
            int u, v;
            double weight;
            ss >> u >> v >> weight;
            addEdge(u, v, weight);
        }
    }

    unordered_map<int, double> dijkstra(int src) {
        unordered_map<int, double> dist;
        for (const auto &node : adjList)
            dist[node.first] = numeric_limits<double>::infinity();
        
        priority_queue<pair<double, int>, vector<pair<double, int>>, greater<>> pq;
        dist[src] = 0;
        pq.push({0, src});

        while (!pq.empty()) {
            int u = pq.top().second;
            double d = pq.top().first;
            pq.pop();

            if (d > dist[u]) continue;

            for (auto &edge : adjList[u]) {
                int v = edge.first;
                double weight = edge.second;
                if (dist[u] + weight < dist[v]) {
                    dist[v] = dist[u] + weight;
                    pq.push({dist[v], v});
                }
            }
        }
        return dist;
    }

    void preprocessLandmarks(int numLandmarks) {
        srand(time(0));
        vector<int> nodes;
        for (const auto &node : adjList) nodes.push_back(node.first);
        
        for (int i = 0; i < numLandmarks; i++) {
            int landmark = nodes[rand() % nodes.size()];
            landmarks.push_back(landmark);
            allPairsShortestPaths[landmark] = dijkstra(landmark);
        }
    }

    void aStar(int start, int goal, int heuristicType) {
        unordered_map<int, double> gScore;
        unordered_map<int, double> fScore;
        priority_queue<pair<double, int>, vector<pair<double, int>>, greater<>> pq;
        unordered_set<int> visited;
        unordered_set<int> inFringe;
        vector<int> expansionOrder;

        int overestimationCount = 0;
        double totalOverestimation = 0.0;

        for (const auto &node : adjList)
            gScore[node.first] = fScore[node.first] = numeric_limits<double>::infinity();
        
        gScore[start] = 0;
        fScore[start] = heuristic(start, goal, heuristicType, overestimationCount, totalOverestimation);
        pq.push({fScore[start], start});
        inFringe.insert(start);

        unordered_map<int, int> cameFrom;

        int nodesProcessed = 0;
        int maxFringeSize = 0;
        int fillCount = 0;
        auto startTime = high_resolution_clock::now();

        while (!pq.empty()) {
            int current = pq.top().second;
            pq.pop();
            inFringe.erase(current);
            nodesProcessed++;
            visited.insert(current);
            fillCount = visited.size();
            maxFringeSize = max(maxFringeSize, static_cast<int>(pq.size()));

            expansionOrder.push_back(current);

            if (current == goal) {
                auto endTime = high_resolution_clock::now();
                auto duration = duration_cast<milliseconds>(endTime - startTime);

                vector<int> path;
                for (int at = goal; at != start; at = cameFrom[at]) {
                    path.push_back(at);
                }
                path.push_back(start);
                reverse(path.begin(), path.end());

                // Compute focus ratio
                unordered_set<int> pathSet(path.begin(), path.end());
                int onPath = 0;
                for (int n : expansionOrder) {
                    if (pathSet.count(n)) onPath++;
                }
                double focusRatio = (double)onPath / expansionOrder.size();

                cout << "\nAlgorithm runtime: " << duration.count() << " ms\n";
                cout << "Nodes processed: " << nodesProcessed << "\n";
                cout << "Maximum fringe size: " << maxFringeSize << "\n";
                cout << "Fill count (unique nodes visited): " << fillCount << "\n";
                cout << "Focus ratio (path nodes / expanded nodes): " << focusRatio << "\n";
                cout << "Overestimation count: " << overestimationCount << "\n";
                if (overestimationCount > 0)
                    cout << "Average overestimation: " << (totalOverestimation / overestimationCount) << "\n";

                cout << "Shortest path: ";
                for (int node : path) cout << node << " -> ";
                cout << "GOAL\n";
                return;
            }

            for (auto &[neighbor, weight] : adjList[current]) {
                double tentativeGScore = gScore[current] + weight;
                if (tentativeGScore < gScore[neighbor]) {
                    cameFrom[neighbor] = current;
                    gScore[neighbor] = tentativeGScore;
                    fScore[neighbor] = gScore[neighbor] + heuristic(neighbor, goal, heuristicType, overestimationCount, totalOverestimation);
                    pq.push({fScore[neighbor], neighbor});
                    inFringe.insert(neighbor);
                }
            }
        }
        cout << "No path found." << endl;
    }

    double heuristic(int node, int goal, int heuristicType, int &overestimationCount, double &totalOverestimation) {
        if (node == goal) return 0;

        double h = 0.0;
        switch (heuristicType) {
            case 2:
                h = landmarkHeuristic(node, goal);
                break;
            case 3:
                h = minOutgoingEdge.count(node) ? minOutgoingEdge[node] : 0;
                break;
            default:
                h = 0;
                break;
        }

        double actual = dijkstra(node)[goal];
        if (h > actual) {
            overestimationCount++;
            totalOverestimation += (h - actual);
            cout << "Heuristic overestimates actual cost! Not admissible for node " << node
                 << " with h = " << h << " and actual = " << actual << "\n";
        }

        return h;
    }

    double landmarkHeuristic(int node, int goal) {
        double maxDiff = 0;
        for (const auto &landmark : landmarks) {
            double diff = abs(allPairsShortestPaths[landmark][goal] - allPairsShortestPaths[landmark][node]);
            maxDiff = max(maxDiff, diff);
        }
        return maxDiff;
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
    cityGraph.preprocessLandmarks(3);

    int start, goal;
    cout << "Enter start node: ";
    cin >> start;
    cout << "Enter goal node: ";
    cin >> goal;

    cout << "\nUsing Landmark-Based Heuristic:\n";
    cityGraph.aStar(start, goal, 2);

    cout << "\nUsing Outgoing Edge Minimum Heuristic:\n";
    cityGraph.aStar(start, goal, 3);

    return 0;
}
