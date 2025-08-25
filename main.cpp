#include "graph.h"

int main() {

    // Step 1: Load Graph and Data Preparation
    // -----------------------------------------------------------------------------
    cout << "\n" << "Step 1: Load Graph and Data Preparation" << endl;
    cout << "-------------------------------------" << endl;
    // Define class "Graph"
    Graph g;

    // Read Road Network
    g.read_graph();

    // Read query, route, and travel time (truth for travel time estimation)
    int readNum = 192484;   // Number of queries to simulate
    g.queryDataRaw = g.read_query(g.queryPath, readNum);
    g.routeDataRaw = g.read_route(g.route_path, readNum);
    g.timeDataRaw = g.read_time(g.time_path, readNum, g.queryDataRaw);

    // Keep first "max_length" roads
    /* Some travel time estimation experiments need to constrain the number of roads */
    int max_length = 30;
    if (true)
    {
        g.routeDataRaw = g.cut_route_data(g.routeDataRaw, max_length);
        g.queryDataRaw = g.cut_query_data(g.queryDataRaw, g.routeDataRaw, max_length);
        g.timeDataRaw = g.cut_time_data(g.timeDataRaw, max_length);
    }

    // Route data transfer
    /* Route constructed by node IDs is converted to road IDs */
    g.routeRoadID = g.route_nodeID_2_roadID(g.routeDataRaw);

    // Step 2: Macroscopic Simulation Algorithm
    // -----------------------------------------------------------------------------
    cout << "\n" << "Step 2: Macroscopic Simulation Algorithm" << endl;
    cout << "-------------------------------------" << endl;

    // Construct dictionary, map road ID to statis features
    g.read_edge_feature(g.edge_id_to_features_path);

    // Macroscopic simulation algorithm
    /* User's options initialization. Detailed description can be found in function simulated_temporal_information */
    bool range = false; bool catching = false; bool latency = true; bool write = false; string te_choose = "latency";
    // Construct model catching dictionary
    if (catching)
        g.features_2_travel_time_dictionary = g.build_catching_dictionary(g.model_catch_dic_path);
    // Macroscopic simulation algorithm
    vector<vector<pair<int, float>>> simulated_temporal_information = g.simulation_algorithm(
            g.queryDataRaw, g.routeDataRaw, range, catching, write, latency, te_choose);

    // Evaluation
    g.evaluation(g.timeDataRaw, simulated_temporal_information);

    g.preprocess_for_traffic_prediction(simulated_temporal_information, g.temporal_information_path_for_traffic_prediction);

    /*
    // Step 3: Route Record Index Construction
    // -----------------------------------------------------------------------------
    cout << "\n" << "Step 3: Route Record Index Construction" << endl;
    cout << "-------------------------------------" << endl;

    // Convert node pairs constructed RR-Index to road ID
    g.convert_node_pairs_index_2_roadID(g.RRIndex);

    // Check correctness of route records in RR-Index
    g.RRIndex_correct_check();

    // Assign time records into one hour time slice
    g.split_RRIndex_2_time_slices(g.RRIndex_roadID);

    // Step 4: Generate new route data for insertion operation and initialization
    // -----------------------------------------------------------------------------
    cout << "\n" << "Step 4: Generate new route data for insertion operation and initialization" << endl;
    cout << "-------------------------------------" << endl;

    int new_num = 1000;
    // Generate new data for insertion operation and store to local file
    // If generated new route number does not change, do not need to generate every time
    if (false)
    {
        srand((unsigned)time(NULL));
        g.data_generation_4_insertion(g.routeDataRaw, g.queryDataRaw, g.insert_route_node_path,
                                      g.insert_route_depart_time_path, g.insert_route_road_path, new_num);
    }

    // Read new route data constructed by road IDs, new route data constructed by node IDs, and related departure time
    g.route_road_new_with_index = g.read_generated_new_route_road(g.insert_route_road_path, g.routeDataRaw, new_num);
    g.route_node_new = g.read_generated_new_route_node(g.insert_route_node_path, g.routeDataRaw, new_num);
    g.departure_time_new = g.read_generated_new_route_departure_time(g.insert_route_depart_time_path);

    // Initialize new temporal information after adding new routes
    g.temporal_result_insertion = g.inserted_temporal_information_initialization(g.temporal_result);

    // Step 5: Insertion operation
    // -----------------------------------------------------------------------------
    cout << "\n" << "Step 5: Insertion operation" << endl;
    cout << "-------------------------------------" << endl;

    bool parallel = false; bool terminal = true; bool print = false;
    range = false; catching = false;
    g.update_operation_insertion(parallel, terminal, range, print, catching);
    */

    std::cout << "Hello, World!" << std::endl;
    return 0;
}
