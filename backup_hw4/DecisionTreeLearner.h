#ifndef DECISION_TREE_LEARNER_H
#define DECISION_TREE_LEARNER_H

#include <vector>
#include <string>
#include <fstream>
#include <sstream>
#include <cmath>
#include <map>
#include <memory>
#include <algorithm>
#include <iostream>
#include <iomanip>
#include <numeric>  // for std::accumulate
#include <set>      // for std::set

struct DataPoint {
    float room;
    float distToPlayer;
    bool lineOfSight;
    float distToWall;
    float timeInAction;
    bool sameRoom;
    float distToPatrol;
    float angleToPlayer;
    float playerSpeed;
    std::string action;
};

class DTNode {
public:
    virtual ~DTNode() = default;
    virtual std::string predict(const DataPoint& data) const = 0;
    virtual const DTNode* getLeft() const { return nullptr; }
    virtual const DTNode* getRight() const { return nullptr; }
};

class DecisionNode : public DTNode {
public:
    int featureIndex;
    float threshold;
    std::unique_ptr<DTNode> leftChild;
    std::unique_ptr<DTNode> rightChild;

    DecisionNode(int feature, float thresh) 
        : featureIndex(feature), threshold(thresh) {}

    std::string predict(const DataPoint& data) const override {
        float value = getValue(data, featureIndex);
        if (value <= threshold) {
            return leftChild->predict(data);
        } else {
            return rightChild->predict(data);
        }
    }

    const DTNode* getLeft() const override { return leftChild.get(); }
    const DTNode* getRight() const override { return rightChild.get(); }

private:
    float getValue(const DataPoint& data, int index) const {
        switch(index) {
            case 0: return data.room;
            case 1: return data.distToPlayer;
            case 2: return data.lineOfSight ? 1.0f : 0.0f;
            case 3: return data.distToWall;
            case 4: return data.timeInAction;
            case 5: return data.sameRoom ? 1.0f : 0.0f;
            case 6: return data.distToPatrol;
            case 7: return data.angleToPlayer;
            case 8: return data.playerSpeed;
            default: return 0.0f;
        }
    }
};

class LeafNode : public DTNode {
public:
    std::string action;
    LeafNode(const std::string& act) : action(act) {}
    
    std::string predict(const DataPoint& data) const override {
        return action;
    }
};

class DecisionTreeLearner {
private:
    std::unique_ptr<DTNode> root;
    const std::vector<std::string> featureNames = {
        "Room", "DistToPlayer", "LineOfSight", "DistToWall",
        "TimeInAction", "SameRoom", "DistToPatrol", "AngleToPlayer",
        "PlayerSpeed"
    };

    std::vector<DataPoint> loadData(const std::string& filename) {
        std::vector<DataPoint> data;
        std::ifstream file(filename);
        std::string line;
        
        // Skip header
        std::getline(file, line);
        
        while (std::getline(file, line)) {
            std::stringstream ss(line);
            std::string value;
            DataPoint point;
            
            std::getline(ss, value, ',');
            point.room = std::stof(value);
            
            std::getline(ss, value, ',');
            point.distToPlayer = std::stof(value);
            
            std::getline(ss, value, ',');
            point.lineOfSight = (value == "1");
            
            std::getline(ss, value, ',');
            point.distToWall = std::stof(value);
            
            std::getline(ss, value, ',');
            point.timeInAction = std::stof(value);
            
            std::getline(ss, value, ',');
            point.sameRoom = (value == "1");
            
            std::getline(ss, value, ',');
            point.distToPatrol = std::stof(value);
            
            std::getline(ss, value, ',');
            point.angleToPlayer = std::stof(value);
            
            std::getline(ss, value, ',');
            point.playerSpeed = std::stof(value);
            
            std::getline(ss, value, ',');
            point.action = value;
            
            data.push_back(point);
        }
        
        return data;
    }

    float calculateEntropy(const std::vector<DataPoint>& data) {
        std::map<std::string, int> actionCounts;
        for (const auto& point : data) {
            actionCounts[point.action]++;
        }
        
        float entropy = 0.0f;
        float total = data.size();
        for (const auto& pair : actionCounts) {
            float probability = pair.second / total;
            entropy -= probability * std::log2(probability);
        }
        return entropy;
    }

    std::pair<float, float> findBestSplit(const std::vector<DataPoint>& data, int featureIndex) {
        float bestGain = -1;
        float bestThreshold = 0;
        float initialEntropy = calculateEntropy(data);
        
        std::vector<float> values;
        for (const auto& point : data) {
            switch(featureIndex) {
                case 0: values.push_back(point.room); break;
                case 1: values.push_back(point.distToPlayer); break;
                case 2: values.push_back(point.lineOfSight ? 1.0f : 0.0f); break;
                case 3: values.push_back(point.distToWall); break;
                case 4: values.push_back(point.timeInAction); break;
                case 5: values.push_back(point.sameRoom ? 1.0f : 0.0f); break;
                case 6: values.push_back(point.distToPatrol); break;
                case 7: values.push_back(point.angleToPlayer); break;
                case 8: values.push_back(point.playerSpeed); break;
            }
        }
        
        std::sort(values.begin(), values.end());
        
        // Try different thresholds
        for (size_t i = 0; i < values.size() - 1; i++) {
            float threshold = (values[i] + values[i + 1]) / 2.0f;
            
            std::vector<DataPoint> leftSplit, rightSplit;
            for (const auto& point : data) {
                float value;
                switch(featureIndex) {
                    case 0: value = point.room; break;
                    case 1: value = point.distToPlayer; break;
                    case 2: value = point.lineOfSight ? 1.0f : 0.0f; break;
                    case 3: value = point.distToWall; break;
                    case 4: value = point.timeInAction; break;
                    case 5: value = point.sameRoom ? 1.0f : 0.0f; break;
                    case 6: value = point.distToPatrol; break;
                    case 7: value = point.angleToPlayer; break;
                    case 8: value = point.playerSpeed; break;
                    default: value = 0.0f;
                }
                
                if (value <= threshold) {
                    leftSplit.push_back(point);
                } else {
                    rightSplit.push_back(point);
                }
            }
            
            float leftEntropy = calculateEntropy(leftSplit);
            float rightEntropy = calculateEntropy(rightSplit);
            float gain = initialEntropy - 
                        (leftSplit.size() * leftEntropy + rightSplit.size() * rightEntropy) / data.size();
            
            if (gain > bestGain) {
                bestGain = gain;
                bestThreshold = threshold;
            }
        }
        
        return {bestGain, bestThreshold};
    }

    std::unique_ptr<DTNode> buildTree(const std::vector<DataPoint>& data, int depth = 0) {
        if (data.empty()) {
            return std::make_unique<LeafNode>("PATROL");  // Default action
        }
        
        // Check if all examples have same action
        bool allSame = true;
        std::string firstAction = data[0].action;
        for (const auto& point : data) {
            if (point.action != firstAction) {
                allSame = false;
                break;
            }
        }
        
        if (allSame || depth >= 8) {  // Increased from 5 to 8
            return std::make_unique<LeafNode>(data[0].action);
        }
        
        // Find best split
        float bestGain = -1;
        int bestFeature = -1;
        float bestThreshold = 0;
        
        for (int i = 0; i < featureNames.size(); i++) {
            auto [gain, threshold] = findBestSplit(data, i);
            if (gain > bestGain) {
                bestGain = gain;
                bestFeature = i;
                bestThreshold = threshold;
            }
        }
        
        if (bestGain <= 0.001f) {  // Changed from 0 to 0.001f
            return std::make_unique<LeafNode>(data[0].action);
        }
        
        // Split data
        std::vector<DataPoint> leftData, rightData;
        for (const auto& point : data) {
            float value;
            switch(bestFeature) {
                case 0: value = point.room; break;
                case 1: value = point.distToPlayer; break;
                case 2: value = point.lineOfSight ? 1.0f : 0.0f; break;
                case 3: value = point.distToWall; break;
                case 4: value = point.timeInAction; break;
                case 5: value = point.sameRoom ? 1.0f : 0.0f; break;
                case 6: value = point.distToPatrol; break;
                case 7: value = point.angleToPlayer; break;
                case 8: value = point.playerSpeed; break;
                default: value = 0.0f;
            }
            
            if (value <= bestThreshold) {
                leftData.push_back(point);
            } else {
                rightData.push_back(point);
            }
        }
        
        auto node = std::make_unique<DecisionNode>(bestFeature, bestThreshold);
        node->leftChild = buildTree(leftData, depth + 1);
        node->rightChild = buildTree(rightData, depth + 1);
        
        return node;
    }

public:
    void train(const std::string& filename) {
        auto data = loadData(filename);
        root = buildTree(data);
    }

    std::string predict(const DataPoint& data) const {
        if (!root) return "PATROL";  // Default action if tree is not trained
        return root->predict(data);
    }

    void saveToFile(const std::string& filename) const {
        // Implementation for saving the tree structure
        // This will be useful for debugging and visualization
    }

    const DTNode* getRoot() const {
        return root.get();
    }

    void analyzeTrainingData(const std::string& filename) {
        std::ifstream file(filename);
        std::string line;
        std::getline(file, line); // Skip header

        int totalSamples = 0;
        std::map<std::string, int> actionCounts;
        std::map<std::string, std::vector<float>> featureStats;
        
        // Initialize feature names
        std::vector<std::string> features = {"Room", "DistToPlayer", "LineOfSight", 
                                           "DistToWall", "TimeInAction", "SameRoom",
                                           "DistToPatrol", "AngleToPlayer", "PlayerSpeed"};
        
        while (std::getline(file, line)) {
            std::stringstream ss(line);
            std::string value;
            std::vector<float> values;
            
            while (std::getline(ss, value, ',')) {
                if (ss.peek() == EOF) {
                    actionCounts[value]++;
                    totalSamples++;
                } else {
                    values.push_back(std::stof(value));
                }
            }
            
            // Record feature values
            for (size_t i = 0; i < values.size(); i++) {
                featureStats[features[i]].push_back(values[i]);
            }
        }

        std::cout << "Training Data Analysis:" << std::endl;
        std::cout << "Total samples: " << totalSamples << std::endl;
        
        std::cout << "\nAction distribution:" << std::endl;
        for (const auto& pair : actionCounts) {
            float percentage = (100.0f * pair.second) / totalSamples;
            std::cout << std::setw(10) << pair.first << ": " 
                      << std::setw(5) << pair.second << " samples ("
                      << std::fixed << std::setprecision(1) << percentage << "%)" << std::endl;
        }
        
        std::cout << "\nFeature statistics:" << std::endl;
        for (const auto& feature : features) {
            auto& values = featureStats[feature];
            if (!values.empty()) {
                float min = *std::min_element(values.begin(), values.end());
                float max = *std::max_element(values.begin(), values.end());
                float sum = std::accumulate(values.begin(), values.end(), 0.0f);
                float mean = sum / values.size();
                
                std::cout << std::setw(15) << feature << ": "
                         << "min=" << std::setw(8) << std::fixed << std::setprecision(2) << min
                         << " max=" << std::setw(8) << max
                         << " mean=" << std::setw(8) << mean
                         << " unique=" << std::setw(4) << std::set<float>(values.begin(), values.end()).size()
                         << std::endl;
            }
        }
    }
};

#endif // DECISION_TREE_LEARNER_H
