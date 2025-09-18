#ifndef ROUTESYS_PROJECT_GRAPH_H
#define ROUTESYS_PROJECT_GRAPH_H

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
#include "boost/thread.hpp"
#include <ctime>
#include <cstdlib>
#include <random>
#include <iostream>
#include <cstring>
#include <sstream>
using namespace std;
constexpr int INF = 999999999;


// Road structure definition
struct Road {
    int roadID;
    int ID1, ID2;
    int length;
    int travelTime;
};

// Contain road features
struct EdgeInfo {
    int lane_num;
    float speed;
    float length;
    string edge_str;
    int minimal_travel_time;
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
        return tie(lane_num, speed_limit, edge_length, driving_number, delay_time, lowSpee_time, wait_time, ratio, length_square)
               == tie(other.lane_num, other.speed_limit, other.edge_length, other.driving_number, other.delay_time, other.lowSpee_time,
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

class Graph {
public:

    // Configuration
    static constexpr const char* BASE_PATH =
            "/Users/xuzizhuo/Desktop/Main Folder/My_works/Working_Projects/EVDATA_Research/Code/EVData_RouteSys/Manhattan_Data/";

    // Step 1: Load Graph and Data Preparation
    // -----------------------------------------------------------------------------

    void read_graph();
    const string Network = string(BASE_PATH) + "Manhattan_network.txt";
    int nodenum{0};
    int edgenum{0};
    vector<vector<pair<int, float>>> graphLength;  // ID1, ID2, Length
    vector<vector<pair<int, int>>> graphRoadID;    // ID1, ID2, RoadID
    vector<set<int>> adjNodes;                     // Adjacent nodes
    vector<Road> roadInfor;
    map<pair<int, int>, int> nodeID2RoadID;        // <ID1, ID2> -> RoadID
    map<int, pair<int, int>> roadID2NodeID;        // RoadID -> <ID1, ID2>
    unordered_map<int, unordered_map<int, vector<int>>> route_time_Dict;

    vector<vector<int>> read_query(const string& filename, int num);
    int time_2_hour(int int_time);
    vector<vector<int>> queryDataRaw;
    const string queryPath = string(BASE_PATH) + "query.txt";
    int min_hour{0};

    vector<vector<int>> read_route(const string& filename, int num);
    vector<vector<int>> routeDataRaw;
    const string route_path = string(BASE_PATH) + "route.txt";

    vector<vector<int>> read_time(const string& filename, int num, vector<vector<int>>& query);
    vector<vector<int>> timeDataRaw;
    const string time_path = string(BASE_PATH) + "time.txt";

    void average_travel_time_estimation(vector<vector<int>>& routeData, vector<vector<int>>& timeData);

    vector<vector<int>> cut_route_data(vector<vector<int>>& route, int max_length);

    vector<vector<int>> cut_query_data(vector<vector<int>>& query, vector<vector<int>>& route, int max_length);

    vector<vector<int>> cut_time_data(vector<vector<int>>& time, int avg_length);

    vector<vector<int>> route_nodeID_2_roadID(vector<vector<int>>& route);
    vector<vector<int>> routeRoadID;

    // Step 2: Macroscopic Simulation Algorithm
    // -----------------------------------------------------------------------------

    void read_edge_feature(const string& filePath);

    const string edge_id_to_features_path = string(BASE_PATH) + "edge_id_to_features.csv";
    map<int, EdgeInfo> edge_id_to_features;

    unordered_map<RoadKey, double> build_catching_dictionary(const string& filename);
    const string model_catch_dic_path = string(BASE_PATH) + "model_catching_with_travel_time_1.txt";
    unordered_map<RoadKey, double> features_2_travel_time_dictionary;

    vector<vector<pair<int, float>>> simulation_algorithm(
            vector<vector<int>> &queries, vector<vector<int>> &routes,
            bool range, bool catching, bool write_inputs, bool latency, string te_choose);
    const string model_catching_path = string(BASE_PATH) + "model_catching.txt";
    vector<vector<pair<int, float>>> temporal_result;
    vector<vector<pair<int, map<int, vector<vector<int>>>>>> RRIndex;
    int catching_no_found{0}, catching_found{0};
    int minTravel{0}, realTravel{0};
    const float sigma = 0.15; const float varphi = 20; const float beta = 2;

    void evaluation(vector<vector<int>>& truth_time, vector<vector<pair<int, float>>>& temporal_information);

    void preprocess_for_traffic_prediction(vector<vector<pair<int, float>>>& temporal_information,
                                           const string& filename);
    const string temporal_information_path_for_traffic_prediction = string(BASE_PATH) +
            "traffic_prediction_structure_1.txt";

    // Step 3: Route Record Index Construction
    // -----------------------------------------------------------------------------

    void convert_node_pairs_index_2_roadID(vector<vector<pair<int, map<int, vector<vector<int>>>>>>& RRIndex);
    vector<map<int, vector<vector<int>>>> RRIndex_roadID;

    void RRIndex_correct_check();

    void split_RRIndex_2_time_slices(vector<map<int, vector<vector<int>>>>& RRIndex_roadID);
    int hour_2_index(int hour);
    vector<vector<map<int, vector<vector<int>>>>> RRIndex_roadID_slice;

    // Step 4: Generate new route data for insertion operation and initialization
    // -----------------------------------------------------------------------------

    void data_generation_4_insertion(
            vector<vector<int>>& routes, vector<vector<int>>& query,
            const string insert_route_nodeID_path, const string insert_route_depart_time_path,
            const string insert_route_roadID_path, int new_num);
    const string insert_route_node_path = string(BASE_PATH) + "new_data_update/insert_route_node_path";
    const string insert_route_depart_time_path = string(BASE_PATH) + "new_data_update/insert_route_depart_time_path";
    const string insert_route_road_path = string(BASE_PATH) + "new_data_update/insert_route_road_path";

    vector<pair<int, vector<int>>> read_generated_new_route_road(const string insert_route_road_path,
                                                                 vector<vector<int>>& routeDataRaw, int new_num);
    vector<pair<int, vector<int>>> route_road_new_with_index;

    vector<vector<int>> read_generated_new_route_node(const string insert_route_node_path,
                                                             vector<vector<int>>& routeDataRaw, int new_num);
    vector<vector<int>> route_node_new;

    vector<int> read_generated_new_route_departure_time(const string insert_route_depart_time_path);
    vector<int> departure_time_new;

    vector<vector<pair<int, float>>> inserted_temporal_information_initialization(
            vector<vector<pair<int, float>>>& temporal_result);
    vector<vector<pair<int, float>>> temporal_result_insertion;

    // Step 5: Insertion operation
    // -----------------------------------------------------------------------------

    void update_operation_insertion(bool parallel, bool terminal, bool range, bool print, bool catching);
    vector<vector<map<int, vector<vector<int>>>>> RRIndex_roadID_slice_insertion;
    /* int find_next_roadID(vector<int> route, int roadID);*/
    // Debug
    int find_next_roadID(vector<int> route, int roadID, string debug, int routeID);
    vector<int> single_route_node_2_route_road(vector<int> &routeNode);

    vector<int> one_route_update_insertion(pair<int, vector<int>>& new_route_pair, int inTime, bool parallel, bool terminal, bool range, bool print, bool catching);

    vector<pair<pair<int,int>, tuple<int, map<int, vector<vector<int>>>, vector<int>>>> update_operation_1st(
            int& roadID, int &inTime, pair<int, vector<int>>& new_route_pair,
            bool &parallel, bool &terminal, bool &range, bool print, bool catching);

    vector<pair<pair<int,int>, tuple<int, map<int, vector<vector<int>>>, vector<int>>>> updateOperationFurther(
            pair<int,int> &roadID_routeID_pair, int &inTime, pair<int, vector<int>> &new_route_pair, pair<int, vector<int>> &route_prop,
            map<int, vector<vector<int>>> &insert_pre, vector<int> &deletion_pre, bool &parallel, bool &terminal,
            bool &range, bool print, bool catching);


    // Count Number of Lines in ".txt" File
    static int CountLines(string filename){
        ifstream ReadFile;
        int n = 0;
        char line[512];
        string temp;
        ReadFile.open(filename,ios::in);
        if (ReadFile.fail())
            return 0;
        else
        {
            while (getline(ReadFile,temp))
            {
                n++;
            }
            return n;
        }
        ReadFile.close();
    }

    // Randomly generate unordered integer
    vector<int> randperm(int Num)
    {
        vector<int> temp;

        for (int i = 0; i < Num; ++i)
        {
            temp.push_back(i);
        }

        std::random_device rd;
        std::mt19937 g(rd());
        shuffle(temp.begin(), temp.end(),g);

        return temp;
    }
};


namespace benchmark {
#define NULLINDEX 0xFFFFFFFF

    template<int log_k, typename k_t, typename id_t>
    class heap {

    public:

        // Expose types.
        typedef k_t key_t;
        typedef id_t node_t;

        // Some constants regarding the elements.
        //static const node_t NULLINDEX = 0xFFFFFFFF;
        static const node_t k = 1 << log_k;

        // A struct defining a heap element.
        struct element_t{
            key_t key;
            node_t element;

            element_t() : key(0), element(0) {}

            element_t(const key_t k, const node_t e) : key(k), element(e) {}
        };


    public:

        // Constructor of the heap.
        heap(node_t n) : n(0), max_n(n), elements(n), position(n, NULLINDEX){}

        heap()
        {

        }

        // Size of the heap.
        inline node_t size() const
        {
            return n;
        }

        // Heap empty?
        inline bool empty() const
        {
            return size() == 0;
        }

        // Extract min element.
        inline void extract_min(node_t &element, key_t &key)
        {
            assert(!empty());

            element_t &front = elements[0];

            // Assign element and key.
            element = front.element;
            key = front.key;

            // Replace elements[0] by last element.
            position[element] = NULLINDEX;
            --n;
            if (!empty())
            {
                front = elements[n];
                position[front.element] = 0;
                sift_down(0);
            }
        }

        inline key_t top()
        {
            assert(!empty());

            element_t &front = elements[0];

            return front.key;
        }

        inline node_t top_value()
        {
            assert(!empty());

            element_t &front = elements[0];

            return front.element;
        }

        // Update an element of the heap.
        inline void update(const node_t element, const key_t key)
        {
            if (position[element] == NULLINDEX)
            {
                element_t &back = elements[n];
                back.key = key;
                back.element = element;
                position[element] = n;
                sift_up(n++);
            }
            else
            {
                node_t el_pos = position[element];
                element_t &el = elements[el_pos];
                if (key > el.key)
                {
                    el.key = key;
                    sift_down(el_pos);
                }
                else
                {
                    el.key = key;
                    sift_up(el_pos);
                }
            }
        }

        // Clear the heap.
        inline void clear()
        {
            for (node_t i = 0; i < n; ++i)
            {
                position[elements[i].element] = NULLINDEX;
            }
            n = 0;
        }

        // Cheaper clear.
        inline void clear(node_t v)
        {
            position[v] = NULLINDEX;
        }

        inline void clear_n()
        {
            n = 0;
        }


        // Test whether an element is contained in the heap.
        inline bool contains(const node_t element) const
        {
            return position[element] != NULLINDEX;
        }

    protected:

        // Sift up an element.
        inline void sift_up(node_t i)
        {
            assert(i < n);
            node_t cur_i = i;
            while (cur_i > 0)
            {
                node_t parent_i = (cur_i - 1) >> log_k;
                if (elements[parent_i].key > elements[cur_i].key)
                    swap(cur_i, parent_i);
                else
                    break;
                cur_i = parent_i;
            }
        }

        // Sift down an element.
        inline void sift_down(node_t i)
        {
            assert(i < n);

            while (true)
            {
                node_t min_ind = i;
                key_t min_key = elements[i].key;

                node_t child_ind_l = (i << log_k) + 1;
                node_t child_ind_u = std::min(child_ind_l + k, n);

                for (node_t j = child_ind_l; j < child_ind_u; ++j)
                {
                    if (elements[j].key < min_key)
                    {
                        min_ind = j;
                        min_key = elements[j].key;
                    }
                }

                // Exchange
                if (min_ind != i)
                {
                    swap(i, min_ind);
                    i = min_ind;
                }
                else
                {
                    break;
                }
            }
        }

        // Swap two elements in the heap.
        inline void swap(const node_t i, const node_t j)
        {
            element_t &el_i = elements[i];
            element_t &el_j = elements[j];

            // Exchange positions
            position[el_i.element] = j;
            position[el_j.element] = i;

            // Exchange elements
            element_t temp = el_i;
            el_i = el_j;
            el_j = temp;
        }

    private:

        // Number of elements in the heap.
        node_t n;

        // Number of maximal elements.
        node_t max_n;

        // Array of length heap_elements.
        vector<element_t> elements;

        // An array of positions for all elements.
        vector<node_t> position;
    };
}


#endif // ROUTESYS_PROJECT_GRAPH_H