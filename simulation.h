/*
#ifndef ROUTESYS_PROJECT_SIMULATION_H
#define ROUTESYS_PROJECT_SIMULATION_H

#include <cstdio>
#include <vector>
#include <set>
#include <fstream>
#include <cmath>
#include <chrono>
#include <unordered_map>
#include <unordered_set>
#include <ctime>
#include <functional>
#include <utility>
#include <string>
#include <algorithm>
#include <map>
#include <thread>
#include <future>
#include <boost/thread/thread.hpp>
#include <semaphore.h>
#include "boost/thread.hpp"
#include <ctime>
#include <cstdlib>
#include <random>
#include <iostream>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <unistd.h>
#include <cstring>
#include <arpa/inet.h>
#include <sstream>
#include "graph.h"
using namespace std;
constexpr int INF = 999999999;

// Contain road features
struct EdgeInfo {
    int lane_num;
    float speed;
    float length;
    string edge_str;
};

// Define a structure as key in map
struct RoadKey {
    // Input features
    int lane_num;
    float speed_limit;
    float edge_length;
    int driving_number;
    int delay_time;
    int lowSpee_time;
    int wait_time;
    int ratio;
    int length_square;

    // Define comparison operation
    bool operator==(const RoadKey &other) const {
        return tie(lane_num, speed_limit, edge_length, driving_number, delay_time,
                   lowSpee_time, wait_time, ratio, length_square)
               == tie(other.lane_num, other.speed_limit, other.edge_length,
                      other.driving_number, other.delay_time, other.lowSpee_time,
                      other.wait_time, other.ratio, other.length_square);
    }
};

// Hash convert RoadKey (road input features)
namespace std {
    template <>
    struct hash<RoadKey> {
        size_t operator()(const RoadKey& k) const
        {
            size_t res = 17;  // Initialization
            // res = res * 31 + hash<string>()(k.edge_str);
            res = res * 31 + hash<int>()(k.lane_num);
            res = res * 31 + hash<float>()(k.speed_limit);
            res = res * 31 + hash<float>()(k.edge_length);
            res = res * 31 + hash<int>()(k.driving_number);
            res = res * 31 + hash<int>()(k.delay_time);
            res = res * 31 + hash<int>()(k.lowSpee_time);
            res = res * 31 + hash<int>()(k.ratio);
            res = res * 31 + hash<int>()(k.wait_time);
            res = res * 31 + hash<int>()(k.length_square);
            return res;
        }
    };
}

class Simulation: public Graph {
public:

    // 添加构造函数
    Simulation() {
        read_graph();
    }

    // Macroscopic simulation algorithm


    // Data processing


    // Statistics
    int catching_no_found{0}, catching_found{0};
    int minTravel{0}, realTravel{0};

    const float sigma = 0.15; const float varphi = 20; const float beta = 2;

    // Mappings

    // File paths

    // Helper methods

};




#endif //ROUTESYS_PROJECT_SIMULATION_H
*/
