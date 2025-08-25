#include "graph.h"

// Construct dictionary, map road ID to statis features
void Graph::read_edge_feature(const string& filePath)
{
    // 1. Open road feature data
    // -----------------------------------------------------------------------------
    ifstream file(filePath);    // Open file
    if (!file.is_open())
    {
        cerr << "Error: Could not open file " << filePath << endl;
        return;
    }

    // 2. Go through each row (road with its features)
    // -----------------------------------------------------------------------------
    string line;
    // Skip the first title row
    getline(file, line);

    while (getline(file, line))
    {
        stringstream ss(line);
        string item;
        vector<string> rowData;

        while (getline(ss, item, ','))
        {
            rowData.push_back(item);
        }

        // Convert data structure
        int edge_id = stoi(rowData[0]);
        int lane_num = static_cast<int>(round(stod(rowData[1])));
        float speed = round(stod(rowData[2]) * 100) / 100;
        float length = round(stod(rowData[3]) * 100) / 100;
        string edge_str = rowData[4];
        /* Makesure the unit of road length and speed limit */
        int minimal_travel_time = length / speed;

        // 3. Store to dictionary
        edge_id_to_features[edge_id] = {lane_num, speed, length, edge_str, minimal_travel_time};
    }

    // 4. Print road feature dictionary
    // -----------------------------------------------------------------------------
    /* for (const auto& edge : edge_id_to_features)
    {
        cout << "Edge ID: " << edge.first << ", Lane Num: " << edge.second.lane_num << ", Speed: " << edge.second.speed;
        cout << ", Length: " << edge.second.length << ", Edge Str: " << edge.second.edge_str << endl;
    } */

    cout << "Read edge static features and construct dictionary done." << "\n" << endl;
}


// Construct model catching dictionary
unordered_map<RoadKey, double> Graph::build_catching_dictionary(const string& filename)
{

    // 1. Variable initialization
    // -----------------------------------------------------------------------------
    unordered_map<RoadKey, double> dictionary;

    // 2. Open model catching file
    // -----------------------------------------------------------------------------
    /* It contains input features and predicted travel time to construct dictionary */
    std::ifstream file(filename);
    if (!file.is_open())
    {
        std::cerr << "Error. Failed to open file: " << filename << std::endl;
        return dictionary;
    }

    // 3. Read features and predicted travel time and construct model catching dictionary
    // -----------------------------------------------------------------------------
    RoadKey key;
    double travel_time_predict;

    while (file >> key.lane_num >> key.speed_limit >> key.edge_length >> key.driving_number >> key.delay_time >> key.lowSpee_time >> key.wait_time >> key.ratio >> key.length_square >> travel_time_predict)
    {
        dictionary[key] = travel_time_predict;
    }

    // 4. Close file
    // -----------------------------------------------------------------------------
    file.close();

    cout << "Size of model catching dictionary is: " << dictionary.size() << endl;
    cout << "Construct model catching dictionary done." << "\n" << endl;

    return dictionary;
}


// Macroscopic simulation algorithm
vector<vector<pair<int, float>>> Graph::simulation_algorithm(
        vector<vector<int>> &queries, vector<vector<int>> &routes,
        bool range, bool catching, bool write_inputs, bool latency, string te_choose) {

    // 1. Variable initialization
    // -------------------------------------------------------

    // Initialize travel time for routes with 'INF' as temporal information
    /* The structure is same as route data with estimated temporal information as weight */
    int route_data_length = routes.size();
    temporal_result.resize(route_data_length);

    for (int i = 0; i < routes.size(); ++i)
    {
        vector<int> route_current = routes[i];
        int route_length_current = route_current.size();
        temporal_result[i].resize(route_length_current);

        // Define origin point
        temporal_result[i][0].first = routes[i][0];
        // Define departure time
        temporal_result[i][0].second = queries[i][2];

        for (int k = 1; k < routes[i].size(); ++k)
        {
            // Define rest node IDs
            temporal_result[i][k].first = routes[i][k];
            // Initialize rest temporal information
            temporal_result[i][k].second = INF;
        }
    }

    // Initialize roads' dynamically changed traffic flow
    /* This structure is same as graph to store dynamically changed traffic flow as weight */
    vector<vector<pair<int, int>>> flow_dynamic;
    flow_dynamic.resize(Graph::graphLength.size());

    for (int i = 0; i < flow_dynamic.size(); i++)
    {
        flow_dynamic[i].resize(Graph::graphLength[i].size());
        for (int j = 0; j < flow_dynamic[i].size(); j++)
        {
            // Define node ID
            flow_dynamic[i][j].first = Graph::graphLength[i][j].first;
            // Initialize traffic flow as 0
            flow_dynamic[i][j].second = 0;
        }
    }

    // Initialize Route-Record Index structure
    /* Each road contains numerous route records */
    /* Each route record refers to "routeID", "status" (enter or leave current road), and "traffic flow" */
    RRIndex.resize(flow_dynamic.size());

    for (int i = 0; i < RRIndex.size(); i++)
    {
        int neighbor_num = flow_dynamic[i].size();
        RRIndex[i].resize(neighbor_num);

        for (int j = 0; j < neighbor_num; j++)
        {
            // Define node ID
            RRIndex[i][j].first = flow_dynamic[i][j].first;
        }
    }

    // Initialize route label and priority queue
    /* Each route label records routes' current arrived node during simulation */
    /* Route labels are managed by the priority queue structure */
    benchmark::heap<2, int, int> H(queries.size());   // Priority queue
    vector<tuple<int, int, float>> label(queries.size());

    for (int i = 0; i < queries.size(); i++)
    {
        // Define route information
        int route_current_ID = i;
        int node_current_index = 0;
        float route_current_depart_time = queries[i][2];

        // Initialize route label for each route
        label[i] = make_tuple(route_current_ID, node_current_index, route_current_depart_time);

        // Update initialized route labels into the priority queue
        H.update(route_current_ID, route_current_depart_time);
    }

    // Store input features for model inference to construct model catching dictionary
    /* Model catching: Construct a dictionary to map input features to travel time.
     * Here, it stores all combination of input features for model inference and back to construct dictionary. */
    std::ofstream outFile;

    if (write_inputs)
    {
        outFile.open(model_catching_path);
        /* Open file by append version */
        // outFile.open(model_catching_path, std::ios::app);
        if (!outFile.is_open())
        {
            std::cerr << "Unable to open file: " << model_catching_path << std::endl;
            return temporal_result;
        }
    }

    // 2. Macroscopic simulation algorithm
    // -------------------------------------------------------

    // Record time consumption for experiment
    std::chrono::high_resolution_clock::time_point t0_1, t0_2;
    std::chrono::duration<double> time_span;
    t0_1 = std::chrono::high_resolution_clock::now();

    // Simulation algorithm
    /* Simulation stop until all routes reach their destination point */
    int label_index_current, time_current, node_index_current;

    while (!H.empty())
    {
        // Extract route label with minimal time by priority queue
        H.extract_min(label_index_current, time_current);
        node_index_current = get<1>(label[label_index_current]);
        int current_node = routes[label_index_current][node_index_current];

        /* The simulation algorithm has three types of situations with different levels of analysis detail:
         * for each route, there are the first node, the last node, and the remaining nodes. */

        // 3. Last node
        // -------------------------------------------------------
        int route_length_current = routes[label_index_current].size();

        if (node_index_current == (route_length_current - 1))
        {
            int previous_node = routes[label_index_current][node_index_current - 1];

            // Update traffic flow and add route record to Route Record Index
            /* Go through neighbors of previous node to locate the target node pairs (last road)*/
            int neighbor_num = flow_dynamic[previous_node].size();

            for (int i = 0; i < neighbor_num; i++)
            {
                if (flow_dynamic[previous_node][i].first == current_node)
                {
                    // Update dynamically changed traffic flow
                    /* As route reach its destination point, and level the last road, traffic flow minus one */
                    flow_dynamic[previous_node][i].second = flow_dynamic[previous_node][i].second - 1;

                    // Add route record to Route Record Index
                    /* As traffic flow change, record its route ID, diving status (leaving), and number of traffic flow */
                    int record_time = get<2>(label[label_index_current]);

                    if (RRIndex[previous_node][i].second.find(record_time) == RRIndex[previous_node][i].second.end())
                    {
                        /* 0 here refers to leaving status */
                        RRIndex[previous_node][i].second.insert(pair<int, vector<vector<int>>>(
                                get<2>(label[label_index_current]), {{label_index_current, 0, flow_dynamic[previous_node][i].second}}));
                    }
                    else
                    {
                        RRIndex[previous_node][i].second[record_time].push_back(
                                {label_index_current, 0, flow_dynamic[previous_node][i].second});
                    }
                }
            }
        }
        // Nodes except the last one
        else
        {
            // 4. First node
            // -------------------------------------------------------
            int next_node = routes[label_index_current][node_index_current + 1];

            if (node_index_current == 0)
            {
                /* Go through neighbors of current node to locate the target node pairs (first road) */
                int neighbor_num = flow_dynamic[current_node].size();

                for (int i = 0; i < neighbor_num; i++)
                {
                    if (flow_dynamic[current_node][i].first == next_node)
                    {
                        // Update traffic flow
                        flow_dynamic[current_node][i].second = flow_dynamic[current_node][i].second + 1;

                        // Add route record to Route Record Index
                        /* As traffic flow change, record its route ID, diving status (enter) , and number of traffic flow*/
                        int record_time = get<2>(label[label_index_current]);

                        if (RRIndex[current_node][i].second.find(record_time) ==
                            RRIndex[current_node][i].second.end())
                        {
                            /* 1 here refers to leaving status */
                            RRIndex[current_node][i].second.insert(pair<int, vector<vector<int>>>(
                                    get<2>(label[label_index_current]),
                                    {{label_index_current, 1, flow_dynamic[current_node][i].second}}));
                        }
                        else
                        {
                            RRIndex[current_node][i].second[record_time].push_back(
                                    {label_index_current, 1, flow_dynamic[current_node][i].second});
                        }

                        // Estimate travel time
                        float te_latency, te_catching;
                        int edge_id = nodeID2RoadID[make_pair(current_node, next_node)];

                        /* Users can choose to use model catching or BPR function for travel time estimation */
                        if (catching)
                        {
                            /* Route wait by congestion not red traffic signal */
                            int delay_time = route_time_Dict[label_index_current][current_node][0];
                            /* Route do not stop but contain low speed (smaller than 5) */
                            int low_time = route_time_Dict[label_index_current][current_node][1];
                            /* Route wait red traffic signal */
                            int wait_time = route_time_Dict[label_index_current][current_node][2];
                            /* traffic flow when route enter */
                            int driving_num = route_time_Dict[label_index_current][current_node][3];

                            // Capture road's static features
                            EdgeInfo features = edge_id_to_features[edge_id];

                            int lane_num = features.lane_num;
                            float speed = features.speed;
                            float length = features.length;

                            // Feature engineering
                            int ratio = static_cast<int>(round(length / speed));
                            int length_square = static_cast<int>(round(pow(length, 2)));

                            // Define key in model catching dictionary
                            RoadKey queryKey = {lane_num, speed, length, driving_num, delay_time, low_time, wait_time, ratio, length_square};

                            if (features_2_travel_time_dictionary.find(queryKey) != features_2_travel_time_dictionary.end())
                            {
                                /* Traffic flow change one may not affect travel time change a lot,
                                * which waste time consumption. */
                                /* Here false refers to apply model catching once traffic flow change */
                                if (range == false)
                                {
                                    te_catching = features_2_travel_time_dictionary[queryKey];
                                    /* Correctness check shows found feature key */
                                    catching_found += 1;

                                    // Capture the minimal travel time on the current road
                                    int tm = features.minimal_travel_time;

                                    /* It refers to non-congestion time consumption of all routes */
                                    minTravel += tm;
                                    /* It refers to congested time consumption of all routes */
                                    realTravel += te_catching;
                                }
                                /* Here refers that traffic flow under different range share same travel time */
                                else
                                {
                                    /* This version is preferred but need further develop */
                                }
                            }
                            else
                            {
                                te_catching = 0;
                                /* Correctness check shows un-found feature key */
                                catching_no_found += 1;
                            }

                            /* In case of model catching does not contain all feature combination, user can choose BPR
                             * function to simulate once and write the feature combination down. Then, after model inference
                             * offline, model_catching_path will contain all feature combinations. */
                            if (write_inputs)
                            {
                                outFile << lane_num << " " << speed << " " << length << " " << driving_num << " ";
                                outFile << delay_time << " " << low_time << " " << wait_time << " ";
                                outFile << ratio << " " << length_square << endl;
                            }
                        }

                        /* Users can choose BPR function to estimate travel time */
                        if (latency)
                        {
                            EdgeInfo features = edge_id_to_features[edge_id];

                            // Estimate the minimal travel time on the current road
                            int tm = features.minimal_travel_time;

                            int current_traffic_flow = flow_dynamic[current_node][i].second;

                            /* Traffic flow change one may not affect travel time change a lot,
                             * which waste time consumption. */
                            /* Here false refers to apply BPR function once traffic flow change */
                            if (range == false)
                            {
                                te_latency = tm * (1 + sigma * pow(current_traffic_flow / varphi, beta));
                                /* For experiment of congestion level */
                                /* It refers to non-congestion time consumption of all routes */
                                minTravel += tm;
                                /* It refers to congested time consumption of all routes */
                                realTravel += te_latency;
                            }
                            /* Here refers that traffic flow under different range share same travel time */
                            else
                            {
                                /* This version is preferred but need further develop */
                            }
                        }

                        // Define travel time on the current road by model catching or BPR function
                        float te = 0;
                        if (te_choose == "catching")
                        {
                            te = te_catching;
                        }
                        else if (te_choose == "latency")
                        {
                            te = te_latency;
                        }
                        else
                        {
                            std::cerr << "Error: Invalid option '" << te_choose << "'." << std::endl;
                        }

                        // Update route label, temporal information of routes, and priority queue
                        get<1>(label[label_index_current]) = get<1>(label[label_index_current]) + 1;
                        get<2>(label[label_index_current]) = get<2>(label[label_index_current]) + te;

                        H.update(get<0>(label[label_index_current]), get<2>(label[label_index_current]));

                        int label_index_next = node_index_current + 1;
                        temporal_result[label_index_current][label_index_next].second = get<2>(
                                label[label_index_current]);
                    }
                }
            }
            // 5. If current node is the middle one of the route
            // -------------------------------------------------------
            /* Analyze should contain the previous road and next road of current node */
            else
            {
                int previous_node = routes[label_index_current][node_index_current - 1];

                /* 5.1 Go through neighbors of previous node to locate the target node pairs (previous road) */
                // -------------------------------------------------------
                for (int i = 0; i < flow_dynamic[previous_node].size(); i++)
                {
                    if (flow_dynamic[previous_node][i].first == current_node)
                    {
                        // Update traffic flow
                        flow_dynamic[previous_node][i].second = flow_dynamic[previous_node][i].second - 1;

                        /* Add route's leaving status on previous road */
                        int record_time = get<2>(label[label_index_current]);

                        if (RRIndex[previous_node][i].second.find(record_time) ==
                            RRIndex[previous_node][i].second.end()) {
                            RRIndex[previous_node][i].second.insert(pair<int, vector<vector<int>>>(
                                    get<2>(label[label_index_current]),
                                    {{label_index_current, 0, flow_dynamic[previous_node][i].second}}));
                        }
                        else
                        {
                            RRIndex[previous_node][i].second[record_time].push_back(
                                    {label_index_current, 0, flow_dynamic[previous_node][i].second});
                        }
                    }
                }

                /* 5.2 Go through neighbors of current node to locate the target node pairs (next road) */
                // -------------------------------------------------------
                for (int i = 0; i < flow_dynamic[current_node].size(); i++)
                {
                    if (flow_dynamic[current_node][i].first == next_node)
                    {
                        // Update traffic flow
                        flow_dynamic[current_node][i].second = flow_dynamic[current_node][i].second + 1;

                        /* Add route's entrance status on next road */
                        int record_time = get<2>(label[label_index_current]);

                        if (RRIndex[current_node][i].second.find(record_time) ==
                            RRIndex[current_node][i].second.end())
                        {
                            RRIndex[current_node][i].second.insert(pair<int, vector<vector<int>>>(
                                    get<2>(label[label_index_current]),
                                    {{label_index_current, 1, flow_dynamic[current_node][i].second}}));
                        }
                        else
                        {
                            RRIndex[current_node][i].second[record_time].push_back(
                                    {label_index_current, 1, flow_dynamic[current_node][i].second});
                        }

                        // Estimate travel time
                        float te_latency, te_catching;
                        int edge_id = nodeID2RoadID[make_pair(current_node, next_node)];

                        /* Users can choose to use model catching or BPR function for travel time estimation */
                        if (catching)
                        {
                            /* Route wait by congestion not red traffic signal */
                            int delay_time = route_time_Dict[label_index_current][current_node][0];
                            /* Route do not stop but contain low speed (smaller than 5) */
                            int low_time = route_time_Dict[label_index_current][current_node][1];
                            /* Route wait red traffic signal */
                            int wait_time = route_time_Dict[label_index_current][current_node][2];
                            /* traffic flow when route enter */
                            int driving_num = route_time_Dict[label_index_current][current_node][3];

                            // Capture road's static features
                            EdgeInfo features = edge_id_to_features[edge_id];

                            int lane_num = features.lane_num;
                            float speed = features.speed;
                            float length = features.length;

                            // Feature engineering
                            int ratio = static_cast<int>(std::round(length / speed));
                            int length_square = static_cast<int>(std::round(pow(length, 2)));

                            // Define key in model catching dictionary
                            RoadKey queryKey = {lane_num, speed, length, driving_num, delay_time,
                                                low_time, wait_time, ratio, length_square};

                            if (features_2_travel_time_dictionary.find(queryKey) !=
                                features_2_travel_time_dictionary.end())
                            {
                                /* Here false refers to apply model catching once traffic flow change */
                                if (range == false) {
                                    te_catching = features_2_travel_time_dictionary[queryKey];
                                    /* Correctness check shows found feature key */
                                    catching_found += 1;

                                    EdgeInfo features = edge_id_to_features[edge_id];

                                    // Estimate the minimal travel time on the current road
                                    int tm = features.minimal_travel_time;

                                    /* It refers to non-congestion time consumption of all routes */
                                    minTravel += tm;
                                    /* It refers to congested time consumption of all routes */
                                    realTravel += te_catching;
                                }
                                /* Here refers that traffic flow under different range share same travel time */
                                else
                                {
                                    /* This version is preferred but need further develop */
                                }
                            }
                            else
                            {
                                te_catching = 0;
                                /* Correctness check shows un-found feature key */
                                catching_no_found += 1;
                            }

                            /* In case of model catching does not contain all feature combination, user can choose BPR
                             * function to simulate once and write the feature combination down. Then, after model inference
                             * offline, model_catching_path will contain all feature combinations. */
                            if (write_inputs)
                            {
                                outFile << lane_num << " " << speed << " " << length << " " << driving_num << " ";
                                outFile << delay_time << " " << low_time << " " << wait_time << " ";
                                outFile << ratio << " " << length_square << endl;
                            }
                        }

                        /* Users can choose BPR function to estimate travel time */
                        if (latency)
                        {
                            EdgeInfo features = edge_id_to_features[edge_id];

                            // Estimate the minimal travel time on the current road
                            int tm = features.minimal_travel_time;

                            int current_traffic_flow = flow_dynamic[current_node][i].second;

                            /* Here false refers to apply BPR function once traffic flow change */
                            if (range == false)
                            {
                                te_latency = tm * (1 + sigma * pow(current_traffic_flow / varphi, beta));
                                /* For experiment of congestion level */
                                /* It refers to non-congestion time consumption of all routes */
                                minTravel += tm;
                                /* It refers to congested time consumption of all routes */
                                realTravel += te_latency;
                            }
                            else
                            {
                                /* This version is preferred but need further develop */
                            }
                        }

                        // Define travel time on the current road by model catching or BPR function
                        float te = 0;
                        if (te_choose == "catching")
                        {
                            te = te_catching;
                        }
                        else if (te_choose == "latency")
                        {
                            te = te_latency;
                        }
                        else
                        {
                            std::cerr << "Error: Invalid option '" << te_choose << "'." << std::endl;
                        }

                        // Update route label, temporal information of routes, and priority queue
                        get<1>(label[label_index_current]) = get<1>(label[label_index_current]) + 1;
                        get<2>(label[label_index_current]) = get<2>(label[label_index_current]) + te;

                        H.update(get<0>(label[label_index_current]), get<2>(label[label_index_current]));

                        int label_index_next = node_index_current + 1;
                        temporal_result[label_index_current][label_index_next].second = get<2>(
                                label[label_index_current]);
                    }
                }
            }
        }
    }

    // 6. Print nodes_label
    // -------------------------------------------------------
    /* for (int i = 0; i < label.size(); i++)
    {
        cout << "route ID " << i << ": ";
        for (int j = 0; j < label[i].size(); j++)
        {
            cout << label[i][j][1] << " " << label[i][j][2] << " ";
        }
        cout << endl;
    }*/

    // 7. Print RR-Index
    // -------------------------------------------------------
    /* for (int i = 0; i < RRIndex.size(); i++)
    {
        cout << "node ID1: " << i << endl;
        for (int j = 0; j < flow_dynamic[i].size(); j++)
        {
            cout << " node ID2: " << RRIndex[i][j].first;
            cout << " with time size: " << RRIndex[i][j].second.size() << " ";

            map<float, vector<vector<int>>>::iterator itr;
            for (itr = RRIndex[i][j].second.begin(); itr != RRIndex[i][j].second.end(); ++itr)
            {
                for (int k = 0; k < itr->second.size(); k++)
                {
                    cout << " time " << itr->first << " routeID " << itr->second[k][0];
                    cout << " status " << itr->second[k][1] << " flow " << itr->second[k][2] << "||";
                }
            }
            cout << "\n" << endl;
        }
        cout << "\n" << endl;
    } */

    // 8. Print temporal information of simulated route data
    // -------------------------------------------------------
    /* for (int i = 0; i < temporal_result.size(); i++)
    {
        cout << "route " << i << ": ";

        for (int j = 0; j < temporal_result[i].size(); j++)
        {
            cout << "node " << temporal_result[i][j].first << " with ";
            cout << "time " << temporal_result[i][j].second << " ";
        }
        cout << endl;
    } */

    // Print time consumption
    t0_2 = std::chrono::high_resolution_clock::now();
    time_span = std::chrono::duration_cast<std::chrono::duration<double>>(t0_2 - t0_1);

    if (range == false)
        cout << "Macroscopic simulation time consumption without range is: " << time_span.count() << "\n" << endl;
    else
        cout << "Macroscopic simulation time consumption with range is: " << time_span.count() << "s." << "\n" << endl;

    // 9. Close files
    // -------------------------------------------------------

    if (outFile.is_open())
        outFile.close();

    // 10. Statistics
    // -------------------------------------------------------
    /* Estimate the average time consumption of each route*/
    int all_travel_time = 0;
    int single_route_travel_time = 0;

    for (int i = 0; i < temporal_result.size(); i++)
    {
        int end_time = temporal_result[i][temporal_result[i].size() - 1].second;
        int departure_time = temporal_result[i][0].second;

        single_route_travel_time = end_time - departure_time;

        if (single_route_travel_time < 0)
        {
            cout << "Error. Simulation Error." << endl;
        }

        all_travel_time += single_route_travel_time;
    }

    cout << "Exp: Average travel time" << endl;
    cout << "routes' total travel time is: " << all_travel_time << endl;
    cout << "simulated average route travel time is: " << all_travel_time / queries.size() << "\n" << endl;

    if (catching)
    {
        cout << "Exp: Catching found" << endl;
        cout << "Catching found: " << catching_found << " & Catching No-Found: " << catching_no_found << "\n" << endl;
    }

    cout << "Exp: Congested travel time" << endl;
    cout << "If no congestion, all simulated routes travel time is: " << minTravel << "s";
    cout << " and travel time consider congestion is " << realTravel << "s" << "\n" << endl;

    // 11. Return simulated temporal information
    // -------------------------------------------------------
    cout << "Macroscopic simulation done." << "\n" << endl;

    return temporal_result;
}


// Evaluation
void Graph::evaluation(vector<vector<int>>& truth_time, vector<vector<pair<int, float>>>& temporal_information)
{
    // 1. Variable initialization
    // -----------------------------------------------------------------------------
    float mse = 0;
    float mae = 0;
    float mape = 0;

    // 2. Evaluation on travel time of each route
    // -----------------------------------------------------------------------------
    int route_num = temporal_information.size();

    // Truth travel time of the current route
    for (int i = 0; i < temporal_information.size(); i++)
    {
        float truth_time_current = 0;
        /* weight of truth_time refers to the truth travel time on each road */
        for (int j = 1; j < truth_time[i].size(); j++)
        {
            truth_time_current += truth_time[i][j];
        }

        // Predicted travel time of the current route
        /* weight of temporal_information refers to the accumulated estimated travel time */
        float end_time = temporal_information[i][temporal_information[i].size() - 1].second;
        float departure_time = temporal_information[i][0].second;
        float estimated_time_current = end_time - departure_time;
        float difference_current = truth_time_current - estimated_time_current;

        // Evaluation
        mae += abs(difference_current);
        mse += difference_current * difference_current;
        if (difference_current != 0 and truth_time_current >= 1)  // To avoid division by zero
            mape += abs(difference_current) / truth_time_current;
    }


    // 3. Evaluation
    // -----------------------------------------------------------------------------
    float MSE = mse / route_num;
    float MAE = mae / route_num;
    float RMSE = sqrt(mse / route_num);
    float MAPE = (mape / route_num) * 100;

    cout << "Evaluation: " << "MSE is: " << abs(MSE) << " MAE is: " << abs(MAE);
    cout << " RMSE is: " << abs(RMSE) << " MAPE is: " << abs(MAPE) << "%" << "\n" << endl;
}

// Convert the weight of temporal_information and write it out for further traffic prediction experiments
void Graph::preprocess_for_traffic_prediction(vector<vector<pair<int, float>>>& temporal_information, const string& filename)
{
    // 1. Variable initialization
    // -----------------------------------------------------------------------------
    int route_num = temporal_information.size();
    vector<vector<pair<int, float>>> traffic_prediction_structure(route_num);

    // 2. Convert weight in the simulated temporal_information
    // -----------------------------------------------------------------------------
    /* Weight in temporal_information is accumulated temporal information,
     * Here, we should convert the weight to travel time on the road */

    for (int i = 0; i < temporal_information.size(); i++)
    {
        int route_ID = i;

        for (int j = 1; j < temporal_information[i].size(); j++)
        {
            int node_ID1 = temporal_information[route_ID][j - 1].first;
            int node_ID2 = temporal_information[route_ID][j].first;

            int edge_ID = nodeID2RoadID[make_pair(node_ID1, node_ID2)];

            float time_enter = temporal_information[route_ID][j - 1].second;
            float time_leave = temporal_information[route_ID][j].second;
            float travel_time = time_leave - time_enter;

            traffic_prediction_structure[route_ID].push_back(make_pair(edge_ID, travel_time));
        }
    }

    // 3. Print
    // -----------------------------------------------------------------------------
    /* for (int i = 0; i < 1; i++)
    {
        cout << "Route ID: " << i << ": ";

        for (int j = 0; j < traffic_prediction_structure[i].size(); j++)
        {
            int edge_ID = traffic_prediction_structure[i][j].first;
            float travel_time = traffic_prediction_structure[i][j].second;

            cout << "(" << edge_ID << ", " << travel_time << ")";
        }
        cout << endl;
    } */

    // 4. Write to file
    // -----------------------------------------------------------------------------
    ofstream outfile(filename);  // Open file

    if (outfile.is_open())
    {
        for (int i = 0; i < traffic_prediction_structure.size(); i++)
        {
            for (int j = 0; j < traffic_prediction_structure[i].size(); j++)
            {
                int edge_ID = traffic_prediction_structure[i][j].first;
                float travel_time = traffic_prediction_structure[i][j].second;

                outfile << edge_ID << " " << travel_time << " ";
            }

            outfile << endl;
        }

        outfile.close();

        cout << "Data for further traffic prediction evaluation is written to " << endl;
        cout << filename << endl;
    }
    else
    {
        cout << "Unable to open file " << filename << " for writing!" << endl;
    }
}