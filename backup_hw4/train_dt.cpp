#include "DecisionTreeLearner.h"
#include <iostream>
#include <iomanip>

// Forward declare the classes we need to access
class DecisionNode;
class LeafNode;

void printTreeStructure(const DTNode* node, int depth = 0) {
    if (!node) return;
    
    if (const DecisionNode* dNode = dynamic_cast<const DecisionNode*>(node)) {
        std::string feature;
        switch(dNode->featureIndex) {
            case 0: feature = "Room"; break;
            case 1: feature = "DistToPlayer"; break;
            case 2: feature = "LineOfSight"; break;
            case 3: feature = "DistToWall"; break;
            case 4: feature = "TimeInAction"; break;
            case 5: feature = "SameRoom"; break;
            case 6: feature = "DistToPatrol"; break;
            case 7: feature = "AngleToPlayer"; break;
            case 8: feature = "PlayerSpeed"; break;
            default: feature = "Unknown";
        }
        
        std::cout << std::string(depth * 2, ' ') << "If " << feature 
                  << " <= " << std::fixed << std::setprecision(2) 
                  << dNode->threshold << ":" << std::endl;
        
        printTreeStructure(dNode->getLeft(), depth + 1);
        std::cout << std::string(depth * 2, ' ') << "Else:" << std::endl;
        printTreeStructure(dNode->getRight(), depth + 1);
    }
    else if (const LeafNode* lNode = dynamic_cast<const LeafNode*>(node)) {
        std::cout << std::string(depth * 2, ' ') << "Return " << lNode->action << std::endl;
    }
}

void analyzeTrainingData(const std::string& filename) {
    std::ifstream file(filename);
    std::string line;
    std::getline(file, line); // Skip header

    int totalSamples = 0;
    std::map<std::string, int> actionCounts;
    
    while (std::getline(file, line)) {
        std::stringstream ss(line);
        std::string value;
        // Skip to the last column (action)
        while (std::getline(ss, value, ',')) {
            if (ss.peek() == EOF) {
                actionCounts[value]++;
                totalSamples++;
            }
        }
    }

    std::cout << "Training Data Analysis:" << std::endl;
    std::cout << "Total samples: " << totalSamples << std::endl;
    std::cout << "Action distribution:" << std::endl;
    for (const auto& pair : actionCounts) {
        float percentage = (100.0f * pair.second) / totalSamples;
        std::cout << std::setw(10) << pair.first << ": " 
                  << std::setw(5) << pair.second << " samples ("
                  << std::fixed << std::setprecision(1) << percentage << "%)" << std::endl;
    }
}

int main() {
    DecisionTreeLearner learner;
    
    std::cout << "Analyzing training data..." << std::endl;
    learner.analyzeTrainingData("monster_behavior.csv");
    
    std::cout << "\nTraining decision tree..." << std::endl;
    learner.train("monster_behavior.csv");
    
    std::cout << "\nDecision Tree Structure:" << std::endl;
    printTreeStructure(learner.getRoot());
    
    // Let's add more diverse test cases
    std::cout << "\nTesting various scenarios:" << std::endl;
    
    std::vector<DataPoint> testCases = {
        // Test case 1: Close to player, with line of sight
        {2, 100.0f, true, 50.0f, 1.0f, true, 200.0f, 45.0f, 0.0f, ""},
        
        // Test case 2: Far from player, no line of sight
        {1, 300.0f, false, 100.0f, 2.0f, false, 50.0f, -90.0f, 30.0f, ""},
        
        // Test case 3: Very close to player
        {3, 20.0f, true, 80.0f, 0.5f, true, 150.0f, 0.0f, 0.0f, ""},
        
        // Test case 4: Near wall, medium distance to player
        {2, 150.0f, true, 10.0f, 1.5f, true, 100.0f, 180.0f, 20.0f, ""},
        
        // Test case 5: Long time in current action
        {1, 200.0f, false, 60.0f, 5.0f, false, 75.0f, 90.0f, 15.0f, ""}
    };

    for (size_t i = 0; i < testCases.size(); i++) {
        const auto& test = testCases[i];
        std::cout << "\nTest Case " << (i + 1) << ":" << std::endl;
        std::cout << "Room: " << test.room 
                  << ", DistToPlayer: " << test.distToPlayer
                  << ", LineOfSight: " << (test.lineOfSight ? "Yes" : "No")
                  << ", DistToWall: " << test.distToWall << std::endl;
        std::cout << "TimeInAction: " << test.timeInAction
                  << ", SameRoom: " << (test.sameRoom ? "Yes" : "No")
                  << ", DistToPatrol: " << test.distToPatrol
                  << ", PlayerSpeed: " << test.playerSpeed << std::endl;
        std::cout << "Predicted Action: " << learner.predict(test) << std::endl;
    }
    
    return 0;
}
