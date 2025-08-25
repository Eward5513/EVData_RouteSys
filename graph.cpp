#include "graph.h"

// Read Graph
void Graph::read_graph()
{
    // 1. Open graph file and read in the number of nodes and edges
    // -----------------------------------------------------------------------------
    ifstream IF(Network);  // Open graph file
    if(!IF)
        cout<<"Cannot open Graph BJ"<<endl;
    //Read Node Number and Edge Number
    IF >> nodenum >> edgenum;
    cout << "Read graph: ";
    cout << "Network contains " << nodenum << " nodes and " << edgenum << " edges" << endl;

    // 2. Variable initialization
    // -----------------------------------------------------------------------------
    nodeID2RoadID.clear();                     // map node ID pairs to road ID
    roadID2NodeID.clear();                     // map road ID to node ID pairs

    // Initialize graph with road ID as weight
    vector<pair<int, int>> vecp;
    graphRoadID.assign(nodenum, vecp);         // node ID1, nodeID2, RoadID

    // Initialize graph with length as weight
    vector<pair<int, float>> vecp_float;
    graphLength.assign(nodenum, vecp_float);   // node ID1, node ID2, Length

    // Initialize road ID with its features
    roadInfor.reserve(edgenum);                // Road Info (node ID1, node ID2, RoadID, Length, Time)

    // Initialize graph without any feature as weight
    set<int> setp;
    adjNodes.assign(nodenum, setp);            // node ID1, node ID2

    // Initialize set to void redundant edge during read graph
    set<pair<int,int>> edgeRedun;              // node ID1, node ID2

    // Declare variables
    int ID1, ID2, edgeID;
    float length;

    // 3. Read graph
    // -----------------------------------------------------------------------------
    for (int i = 0; i < edgenum; i++)
    {
        // Each line indicates node ID1, node ID2, road ID, and length
        IF >> ID1 >> ID2 >> edgeID >> length;

        // Store road information
        Road r;
        r.ID1 = ID1;
        r.ID2 = ID2;
        r.roadID = edgeID;
        r.length = length;
        r.travelTime = -1;
        roadInfor.push_back(r);

        // Construct maps
        nodeID2RoadID.insert(make_pair(make_pair(ID1, ID2), edgeID));
        roadID2NodeID.insert(make_pair(edgeID, make_pair(ID1, ID2)));

        // Check redundant and construct graph
        if (edgeRedun.find(make_pair(ID1,ID2)) == edgeRedun.end())
        {
            graphLength[ID1].push_back(make_pair(ID2, length));
            graphRoadID[ID1].push_back(make_pair(ID2, edgeID));
            adjNodes[ID1].insert(ID2);
        }
        edgeRedun.insert(make_pair(ID1,ID2));
    }

    // 4. Print graph length as weight
    // -----------------------------------------------------------------------------
    /* for (int i = 0; i < graphLength.size(); i++)
    {
        cout << "node ID " << i << " ";
        for (int j = 0; j < graphLength[i].size(); j++)
        {
            cout << "node ID " << graphLength[i][j].first << " with length " << graphLength[i][j].second << " ";
        }
        cout << endl;
    } */

    cout << "Read graph done." << "\n" << endl;
}

// Read query data
vector<vector<int>> Graph::read_query(const string& filename, int num)
{
    // 1. Variable initialization
    // -----------------------------------------------------------------------------
    int input_num, DepartureID, DestinationID, DepartureTime;
    vector<vector<int>> Q;

    // 2. Open query file and check user defined read number
    // -----------------------------------------------------------------------------
    ifstream file_name(filename);   // Open file
    if(!file_name)
        cout << "Cannot open query data" << endl;

    // Count number of lines
    int lines = CountLines(filename);
    // Check users defined number of query data is higher than the maximum
    input_num = num;
    if (num > lines or num < 0)
    {
        input_num = lines;
        cout << "Warning. Predefined query number " << num << " is higher than the maximum " << lines;
        cout << " The number of " << lines << " is used in the simulation." << endl;
    }

    // 3. Read query data
    // -----------------------------------------------------------------------------
    for (int i = 0; i < input_num; i++)
    {
        file_name >> DepartureID >> DestinationID >> DepartureTime;
        Q.push_back({DepartureID,DestinationID,DepartureTime});
    }
    cout << "Read query data: read " << Q.size() << " query data." << endl;

    // 4. Close File
    // -----------------------------------------------------------------------------
    file_name.close();

    // 5. Statistics: Find the hour index that the minimal departure time belongs
    // -----------------------------------------------------------------------------
    vector<int> times;
    // Query Data Size Depend on Different Manipulation
    if (Q.size() != 0)
    {
        for (int i = 0; i < Q.size(); i++)
        {
            times.push_back(Q[i][2]);
        }

        int min_departure = *min_element(times.begin(), times.end());
        cout << "Statistics: The minimal departure time is: " << min_departure << endl;
        /* function 'time_2_hour' use the 'localtime' function to returns a tm structure with date information for a
         * timestamp in the computer's local timezone. UTC time zone and Beijing time zone has 8 hours difference.
         * Under this situation, the minimal hour index must be zero.
         * Please check the correct time and time zone in your situation,  */
        // min_hour = time_2_hour(min_departure);       // UTC time zone
        min_hour = time_2_hour(min_departure);   // Beijing time zone
        cout << "Statistics: The minimal hour index is: " << min_hour << endl;

        cout << "!!! Note: This time convert should consider the time zone of current system. ";
        cout << "Please double check the correctness." << endl;
    }
    else
    {
        cout << "Error. The size of query data is 0. Cannot find the minimal departure time." << endl;
    }

    // 6. Print Query Data
    // -----------------------------------------------------------------------------
    /* for (int i = 0; i < Q.size(); i++)
    {
        cout << "query " << i << ": ";
        cout << "origin " << Q[i][0] << " destination " << Q[i][1] << " departure time " << Q[i][2] << endl;
    } */
    cout << "Read query data done." << "\n" << endl;

    return Q;
}


// Read Route Data
vector<vector<int>> Graph::read_route(const string& filename, int num)
{
    // 1.Variable Initialization
    // -----------------------------------------------------------------------------
    vector<int> pi;
    vector<vector<int>> Pi;
    int input_num, vertexNum, vertex, delay_time, low_time, wait_time, driving_num, avg_roads, total_roads = 0;

    // 2. Open Route Data and check input number
    // -----------------------------------------------------------------------------
    ifstream file_name(filename);   // Open file
    if(!file_name)
        cout<< "Cannot open Route Data" <<endl;

    // Count number of lines
    int lines = CountLines(filename);
    // Check users defined number of query data is higher than the maximum
    input_num = num;
    if (num > lines or num < 0)
    {
        input_num = lines;
        cout << "Warning. Predefined route number " << num << " is higher than the maximum " << lines;
        cout << " The number of " << lines << " is used in the simulation." << endl;
    }

    // 3. Read route data
    // -----------------------------------------------------------------------------
    for (int i = 0; i < input_num; i++)
    {
        file_name >> vertexNum;     // Read one route each round

        // Correctness Check
        if (vertexNum == 0)
            cout << "Warning. Route num is 0 when read in." << endl;

        for (int j = 0; j < vertexNum; j++)
        {
            /* "delay_time", "low_time", and "wait_time" are boolean value, indicating the traffic signal state and
             * routes driving behaviors on this road. In addition, "driving_num" indicates the number of current traffic
             * flow. These features will be input information during simulation for travel time estimation accuracy. */
            file_name >> vertex >> delay_time >> low_time >> wait_time >> driving_num;
            route_time_Dict[i][vertex] = {delay_time, low_time, wait_time, driving_num};
            total_roads += 1;

            pi.push_back(vertex);
        }

        Pi.push_back(pi);
        pi.clear();
    }
    cout << "Read route data: read " << Pi.size() << " route data." << endl;

    // Estimate the number of average roads
    avg_roads = total_roads / input_num;
    cout << "Each route contains an average of " << avg_roads << " roads." << endl;

    // 4. Close file
    // -----------------------------------------------------------------------------
    file_name.close();

    // 5. Print route data
    // -----------------------------------------------------------------------------
    /* for (int i = 0; i < Pi.size(); i++)
    {
        cout << "route " << i << ": ";
        for (int j = 0; j < Pi[i].size(); j++)
        {
            cout << Pi[i][j] << " ";
        }
        cout << endl;
    } */
    cout << "Read route data done." << "\n" << endl;

    return Pi;
}


// Read time data
vector<vector<int>> Graph::read_time(const string& filename, int num, vector<vector<int>>& query)
{
    // 1. Variable Initialization
    // -----------------------------------------------------------------------------
    vector<int> t;
    vector<vector<int>> T;
    int input_num, timeNum, time;

    // Open time data and check input number
    // -----------------------------------------------------------------------------
    ifstream file_name(filename);   // Open file
    if(!file_name)
        cout<<"Cannot open time data" <<endl;

    // Count number of lines
    int lines = CountLines(filename);
    // Check users defined number of query data is higher than the maximum
    input_num = num;
    if (num > lines or num < 0)
    {
        input_num = lines;
        cout << "Warning. Predefined time number " << num << " is higher than the maximum " << lines;
        cout << " The number of " << lines << " is used in the simulation." << endl;
    }

    // 2. Read toute Data
    // -----------------------------------------------------------------------------
    for (int i = 0; i < input_num; i++)
    {
        file_name >> timeNum;   // Read one route's arrived time each round

        t.push_back(query[i][2]);

        // Correctness Check
        if (timeNum == 0)
            cout << "route num is 0 when read in." << endl;

        for (int j = 0; j < timeNum; j++)
        {
            /* Each vertex is associated with a travel time that the route spend on the road.
             * E.g. For node ID1, node ID2, node ID3 with time stamp, time 2, time 3, the route's departure time is
             * time stamp such as 59825, and pass through node ID1 and node ID2 with time 2 (such as 3 seconds). */
            file_name >> time;
            t.push_back(time);
        }

        T.push_back(t);
        t.clear();
    }
    cout << "Read time data: read " << T.size() << " time data." << endl;

    // 3. Close File
    // -----------------------------------------------------------------------------
    file_name.close();

    // 4. Estimate average travel time
    // -----------------------------------------------------------------------------
    average_travel_time_estimation(query, T);

    // 5. Print Route Data
    // -----------------------------------------------------------------------------
    /* for (int i = 0; i < T.size(); i++)
    {
        cout << "route " << i << ": ";
        for (int j = 0; j < T[i].size(); j++)
        {
            cout << T[i][j] << " ";
        }
        cout << endl;
    } */
    cout << "Read time data done." << "\n" << endl;

    return T;
}


// Estimate average travel time of ground truth
void Graph::average_travel_time_estimation(vector<vector<int>>& queryData, vector<vector<int>>& timeData)
{
    // 1. Variable Initialization
    // -----------------------------------------------------------------------------
    float total_time = 0;
    int route_num = queryData.size();

    // 2. Calculate all routes' travel time truth
    // -----------------------------------------------------------------------------
    for (int i = 0; i < timeData.size(); i++)
    {
        int travel_time = 0;
        for (int j = 1; j < timeData[i].size(); j++)
        {
            travel_time += timeData[i][j];
        }

        total_time += travel_time;
    }

    int average_travel_time = total_time / route_num;
    cout << "ground truth travel time avg is: " << average_travel_time << "s."<< endl;
}


// Cut route data
vector<vector<int>> Graph::cut_route_data(vector<vector<int>> &route, int max_length)
{
    // 1. Variable initialization
    // -----------------------------------------------------------------------------
    vector<vector<int>> routeData;
    vector<int> placeHolder;
    routeData.assign(routeDataRaw.size(), placeHolder);

    // 2.Go through route Data and cut
    // -----------------------------------------------------------------------------
    for (int i = 0; i < routeDataRaw.size(); i++)
    {
        // Keep first "max_length" roads for each route data
        if (routeDataRaw[i].size() <= max_length)
        {
            routeData[i] = routeDataRaw[i];
        }
        else{
            vector<int>::const_iterator start = routeDataRaw[i].begin();
            vector<int>::const_iterator end = routeDataRaw[i].begin() + max_length;
            vector<int> cutVector(start, end);
            routeData[i] = cutVector;
        }
    }

    // 3. Print route data out
    // -----------------------------------------------------------------------------
    /* for (int i = 0; i < routeData.size(); i++)
    {
        cout << "route " << i << " with size: " << routeData[i].size() << endl;
        cout << "route data are: ";
        for (int j = 0; j < routeData[i].size(); j++)
        {
            cout << routeData[i][j] << " ";
        }
        cout << endl;
    } */
    cout << "Route data cut done." << "\n" << endl;

    return routeData;
}


// Cut query data
vector<vector<int>> Graph::cut_query_data(vector<vector<int>> &query, vector<vector<int>> &route, int max_length)
{
    // 1. Variable initialization
    // -----------------------------------------------------------------------------
    vector<vector<int>> queryData = query;
    // Go through query data and modify its destination point
    for (int i = 0; i < queryData.size(); i++)
    {
        int route_length = route[i].size();
        queryData[i][1] = route[i][route_length - 1];
    }

    // 2. Correctness check
    // -----------------------------------------------------------------------------
    for (int i = 0; i < route.size(); i++)
    {
        int route_length = route[i].size();
        if (route[i][0] != queryData[i][0])
            cout << "Error! First nodes are not match." << endl;
        if (route[i][route_length - 1] != queryData[i][1])
            cout << "Error! Last nodes are not match." << endl;
    }

    // 3. Print query data
    // -----------------------------------------------------------------------------
    /* for (int i = 0; i < queryData.size(); i++)
    {
        cout << "query " << i << ": ";
        cout << "origin " << queryData[i][0] << " destination " << queryData[i][1] << " departure time ";
        cout << queryData[i][2] << endl;
    } */

    cout << "query data cut done." << "\n" << endl;

    return queryData;
}


// Cut time data
vector<vector<int>> Graph::cut_time_data(vector<vector<int>> &time, int max_length)
{
    // Variable initialization
    // -----------------------------------------------------------------------------
    vector<vector<int>> timeData;
    vector<int> placeHolder;
    timeData.assign(time.size(), placeHolder);

    // Go Through Route Data and Select Parts
    // -----------------------------------------------------------------------------
    for (int i = 0; i < time.size(); i++)
    {
        if (time[i].size() <= max_length)
        {
            timeData[i] = time[i];
        }
        else
        {
            vector<int>::const_iterator start = time[i].begin();
            vector<int>::const_iterator end = time[i].begin() + max_length;
            vector<int> cutVector(start, end);
            timeData[i] = cutVector;
        }
    }

    // Print route data out
    // -----------------------------------------------------------------------------
    /* for (int i = 0; i < timeData.size(); i++)
    {
        cout << "time data " << i << " with size: " << timeData[i].size() << endl;
        for (int j = 0; j < timeData[i].size(); j++)
        {
            cout << timeData[i][j] << " ";
        }
        cout << endl;
    } */

    cout << "time data cut done." << "\n" << endl;

    return timeData;
}


// Convert Route from "Node ID Pair" to "Road ID"
vector<vector<int>> Graph::route_nodeID_2_roadID(vector<vector<int>> &route_nodeID)
{
    // 1. Variable initialization
    // -----------------------------------------------------------------------------
    int node_ID1, node_ID2, roadID;
    vector<vector<int>> route_RoadID;
    vector<int> placeHolder;
    route_RoadID.assign(route_nodeID.size(), placeHolder);

    // 2. Route data transfer
    // -----------------------------------------------------------------------------
    for (int i = 0; i < route_RoadID.size(); i++)
    {
        if (route_nodeID[i].size() <= 1)    // Check length
            continue;

        // Resize route data
        int route_nodeID_length = route_nodeID[i].size();
        route_RoadID[i].resize(route_nodeID_length - 1);

        for (int j = 0; j < route_RoadID[i].size(); j++)
        {
            node_ID1 = route_nodeID[i][j];
            node_ID2 = route_nodeID[i][j+1];

            // If Node Pairs Contained, Convert
            if (nodeID2RoadID.find(make_pair(node_ID1, node_ID2)) != nodeID2RoadID.end())
            {
                roadID = nodeID2RoadID[make_pair(node_ID1, node_ID2)];
                route_RoadID[i][j] = roadID;
            }
            else
            {
                cout << "Warning. Unfounded node pairs are: " << node_ID1 << " " << node_ID2 << endl;
            }
        }
    }

    // 3. Print Transfer Route Data
    // -----------------------------------------------------------------------------
    /* for (int i = 0; i < route_RoadID.size(); i++)
    {
        cout << "Route ID: " << i << endl;
        for (int j = 0; j < route_RoadID[i].size(); j++)
        {
            cout << route_RoadID[i][j] << " ";
        }
        cout << endl;
    } */
    cout << "Route transfer done. Route constructed by node IDs is converted to road IDs." << endl;

    return route_RoadID;
}


