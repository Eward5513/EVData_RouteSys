#include "graph.h"

// Find the next road ID in the route constructed by road IDs
/* int Graph::find_next_roadID(vector<int> route, int roadID) */
// Debug
int Graph::find_next_roadID(vector<int> route, int roadID, string debug, int routeID)
{
    int nextRoadID = -1;

    auto itFind = std::find(route.begin(), route.end(), roadID);

    if (itFind == route.end())
    {
        /* std::cout << "Error: roadID " << roadID << " not found in route.\n";*/
        // Debug
        std::cout << "Error: roadID " << roadID << " not found in route " << routeID << ". [Context: "
                  << debug << "]" << std::endl;
        for (int i = 0; i < route.size(); i++){
            cout << route[i] << " ";
        }
        cout << endl;

        return -1;
    }

    auto last = std::prev(route.end());

    if (itFind == last)
        nextRoadID = -1;
    else
        nextRoadID = *(++itFind);

    return nextRoadID;
}



// Convert a node constructed route to a road ID constructed route
vector<int> Graph::single_route_node_2_route_road(vector<int> &routeNode)
{
    // 1. Variable initialization
    // -----------------------------------------------------------------------------
    int routeID = 0;
    int node_ID1, node_ID2, roadID;

    vector<int> route_road;
    route_road.reserve(routeNode.size() - 1);

    // 2. Convert
    // -----------------------------------------------------------------------------
    for (int i = 0; i < routeNode.size() - 1; i++)
    {
        node_ID1 = routeNode[i];
        node_ID2 = routeNode[i + 1];

        roadID = nodeID2RoadID[make_pair(node_ID1, node_ID2)];

        route_road.push_back(roadID);
    }

    // 3. Print result
    // -----------------------------------------------------------------------------
    /* cout << "route with roadSegment ID is: ";

    for (int i = 0; i < route_road.size(); i++)
    {
        cout << route_road[i] << " ";
    }
    cout << endl; */

    return route_road;
}


// Insertion operation (Main)
void Graph::update_operation_insertion(bool parallel, bool terminal, bool range, bool print, bool catching)
{
    // 1. Variable initialization
    // -----------------------------------------------------------------------------

    vector<int> affected_roads;
    // Initialize RR-Index structure to store route records after insertion operation
    RRIndex_roadID_slice_insertion = RRIndex_roadID_slice;

    // 2. Update operation
    // -----------------------------------------------------------------------------
    // Record time consumption for experiment
    std::chrono::high_resolution_clock::time_point t0, t1;
    std::chrono::duration<double> time_span1;
    t0=std::chrono::high_resolution_clock::now();

    for (int i = 0; i < departure_time_new.size(); i++)
    {
        /* */
        affected_roads = one_route_update_insertion(route_road_new_with_index[i], departure_time_new[i], parallel, terminal, range, print, catching);
    }

    t1=std::chrono::high_resolution_clock::now();
    time_span1 = std::chrono::duration_cast<std::chrono::duration<double>>(t1 - t0);

    // Print
    if (terminal)
        cout << "Insertion operation's time consumption w termination is: " << time_span1.count() << "s." << endl;
    else
        cout << "Insertion operation's time consumption w/o termination is: " << time_span1.count() << "s." << endl;
}

// Insertion Operation by Inserting New Route Data One by One
/* "new_route_pair" contains: route ID + route constructed by roads */
vector<int> Graph::one_route_update_insertion(pair<int, vector<int>>& new_route_pair, int inTime, bool parallel, bool terminal, bool range, bool print, bool catching)
{
    bool prop;
    prop = false;

    // 1. Variable initialization
    // -----------------------------------------------------------------------------

    // Initialize a structure to store affected roads
    /* Background: Insertion of a new route can not only affect existing route records in roads contained in the new route,
     * but these effected can also propagate to other roads even not contained in the new route */

    /* Design: The input information should contain road ID, route ID, the entrance time of new route (0 if not passing
     * through by new route), affected route records propagate from the previous road */

    /* roadId: road_info_affected[i].first.first
     * routeID: road_info_affected[i].first.second
     * entrance time or 0: tuple<0>
     * affected route records need to insert: tuple<1>
     * affected route records need to delete: tuple<2> */
    vector<pair<pair<int,int>, tuple<int, map<int, vector<vector<int>>>, vector<int>>>> road_info_affected;

    int road_1st = new_route_pair.second[0];

    // Count affected road number
    int count = 0;

    // Add New Inserted Route Data into List
    vector<int> roadID_affected;
    roadID_affected.push_back(road_1st);

    if (print == true)
    {
        cout << "=======================================================================" << endl;
        cout << "count: " << count << endl;
    }

    // Step 2. Iterate from the first road in new route
    // -----------------------------------------------------------------------------
    road_info_affected = update_operation_1st(road_1st, inTime, new_route_pair, parallel, terminal, range, print, catching);
    count += 1;

    // Step 3. Iterate on the rest roads (no matter it is contained in the new route)
    // -----------------------------------------------------------------------------

    // When List Is Not Empty
    while(!road_info_affected.empty())
    {
        count += 1;

        // Select the first affected road
        std::vector<pair<pair<int,int>, tuple<int, map<int, vector<vector<int>>>, vector<int>>>>::iterator itr;
        itr = road_info_affected.begin();

        // Step 3.1 Define input features
        // -----------------------------------------------------------------------------

        // Define next road and contained route ID
        pair<int,int> road_route_ID_pairs = itr->first;
        int roadID = itr->first.first;
        int routeID = itr->first.second;

        /* If the previous iterated road is the last one in a route, its returned next road is -1,
         * we can skip it to the next iteration */
        if (roadID == -1)
        {
            road_info_affected.erase(itr);
            continue;
        }

        // Define route contained by road IDs
        vector<int> affected_route_node = route_node_new[routeID];
        vector<int> affected_route_road = single_route_node_2_route_road(affected_route_node);
        pair<int, vector<int>> route_ID_roads_pair = make_pair(routeID, affected_route_road);

        // Define the propagated information from the previous road
        int leaveTime = get<0>(itr->second);
        map<int, vector<vector<int>>> insert = get<1>(itr->second);
        vector<int> deletion = get<2>(itr->second);

        // Print Road ID and Driving in Time
        if (print == true)
        {
            cout << "=======================================================================" << endl;
            cout << endl;

            cout << "Step 1: round start" << endl;
            cout << "-----------------------------------------" << endl;
            cout << "count: " << count << endl;
            cout << "road segment ID is: " << roadID << " with leaveTime: " << leaveTime << endl;
        }

        // Print "insert_pre" and "deletion_pre" for debug
        if (print == true)
        {
            cout << "Insert set size is: " << insert.size() << " Deletion size is: " << deletion.size() << endl;
            map<int, vector<vector<int>>>::iterator itrInsert;

            for (itrInsert = insert.begin(); itrInsert != insert.end();++itrInsert)
            {
                cout << "inserted time is: " << itrInsert->first << " with size: " << itrInsert->second.size() << endl;
            }
        }

        // Step 3.2 Iteration for further affected roads
        // -----------------------------------------------------------------------------
        road_info_affected.erase(itr);

        // Update operation for further road
        vector<pair<pair<int,int>, tuple<int, map<int, vector<vector<int>>>, vector<int>>>> roadSegmentSetTemp;
        roadSegmentSetTemp = updateOperationFurther(road_route_ID_pairs, leaveTime, new_route_pair, route_ID_roads_pair, insert, deletion, parallel, terminal, range, print, catching);

        // Add affected road ID to list
        roadID_affected.push_back(roadID);

        // Update return value (input features for the next affected road) for the next iteration
        for (int i = 0; i < roadSegmentSetTemp.size(); i++)
        {
            road_info_affected.push_back(roadSegmentSetTemp[i]);
        }
    }

    if (print == true)
    {
        cout << "This Route Update Done." << endl;
        cout << "=======================================================================" << endl;
    }

    return roadID_affected;
}



// Insertion for the first road of a new route
/* First road and other roads of new route have different operation */
/* "new_route_pair" contains both route ID and route constructed by roads */
// ??? 为什么第一个道路没有判断使用 terminal condition 的 bool
vector<pair<pair<int,int>, tuple<int, map<int, vector<vector<int>>>, vector<int>>>> Graph::update_operation_1st(
        int& roadID, int &inTime, pair<int, vector<int>>& new_route_pair, bool &parallel, bool &terminal, bool &range, bool print, bool catching)
{

    // 1. Variable initialization
    // -----------------------------------------------------------------------------
    /* The main idea to check if route records under a road are affected is to compare the difference between
     * traffic flow before and after the insertion operation. */
    int traffic_flow_origin = 0;        // Traffic flow before insertion operation
    int traffic_flow_insertion = 0;     // Traffic flow after insertion operation

    // Check the status of terminal condition: 0 -> Not, 1 -> Yes
    /* Activation of the terminal condition indicates there is no further affected route records under this road,
     * and the insertion operation can stop on this road. */
    int terminal_condition = 0;

    // Record time stamp when traffic flow before and after insertion operation same for further termination check
    /* When insert a new route record in the RR-Index, assume route records before the inserted time are not affected, and
     * we re-simulate and update the further affected route records. This update can terminate early if: */
    /* 1. When a vehicle insert this road, traffic flow before and after insertion are same, we record this time stamp. */
    /* 2. If further traffic flow before and after insertion operation are still same when that vehicle leave this road. */
    int record_time = 0;

    // Initialize insert and deletion list
    /* As insertion, part of original route records should be updated (delete first and insert another one) */
    map<int, vector<vector<int>>> insert_list;
    vector<int> deletion_list;

    // Capture new route ID and new route constructed by roads from argument
    int new_route_ID = new_route_pair.first;
    vector<int> new_route_road = new_route_pair.second;

    // Initialize returned values
    /* As new vehicle enter current road, we re-simulate all affected original route records.
     * Then, this function returns two kinds of information:
     * 1. New vehicle's entrance time for the next road
     * 2. Part of original route records will be delayed, their update entrance time for the next road */
    vector<pair<pair<int,int>, tuple<int, map<int, vector<vector<int>>>, vector<int>>>> roadSegmentAffected;
    /* "null" if current road is the last one of this route */
    vector<pair<pair<int,int>, tuple<int, map<int, vector<vector<int>>>, vector<int>>>> null;

    // Define belonged time slice
    int hour = time_2_hour(inTime);
    int index = hour_2_index(hour);

    // Define node ID1 and node ID2
    int current_node = roadID2NodeID[roadID].first;
    int next_node = roadID2NodeID[roadID].second;
    /* int next_road_ID = find_next_roadID(new_route_pair.second, roadID); */
    // Debug
    int next_road_ID = find_next_roadID(new_route_pair.second, roadID, "A_1", new_route_pair.first);


    // Define the minimal travel time
    EdgeInfo road_features = edge_id_to_features[roadID];
    int tm = road_features.minimal_travel_time;

    vector<int> affected_road_ID;
    int time_new_leave;

    tuple<int, map<int, vector<vector<int>>>, vector<int>> return_temp;

    // Step 2. Insert newly inserted route record into RR-Index and estimate its leaving time
    // -----------------------------------------------------------------------------

    // Step 2.1. Find time value (key) or insert a new one
    // -----------------------------------------------------------------------------
    /* For all times (key) in RR-Index, if inserted vehicle's entrance time already exist,
     * we directly query it. If not, we should insert a new time as key to further insert records. */

    // Define traffic flow of previous record (value)
    /* The newly inserted traffic flow should be the previous plus one */
    int preFlow;
    /* Insert a temporary record (value) and update further */
    vector<int> features_temp = {new_route_ID, 1, 0};

    map<int, vector<vector<int>>>::iterator itFind;
    itFind = RRIndex_roadID_slice_insertion[roadID][index].find(inTime);

    // If time (key) not be found, insert
    if (itFind == RRIndex_roadID_slice_insertion[roadID][index].end())
    {
        // Insert a new key with a temporary defined feature
        /* Note: traffic flow will be modified further, no worry here */
        RRIndex_roadID_slice_insertion[roadID][index].insert(pair<int, vector<vector<int>>>(inTime, {features_temp}));

        // Query previous traffic flow and update inserted on based on it
        map<int, vector<vector<int>>>::iterator itPre;
        itPre = RRIndex_roadID_slice_insertion[roadID][index].find(inTime);

        if (itPre != RRIndex_roadID_slice_insertion[roadID][index].end())
        {
            // If newly inserted time (key) is the first one
            if (itPre == RRIndex_roadID_slice_insertion[roadID][index].begin())
            {
                preFlow = 0;
            }
            else
            {
                itPre = --itPre;
                preFlow = itPre->second.back()[2];
            }
        }
        else
        {
            // If time (key) is not found, it must be an error since we just insert it
            cout << "No value found. Please check Insertion." << endl;
            return null;
        }
    }
    else
    {
        // If time (key) is found, directly push back feature in it
        preFlow = itFind->second.back()[2];
        features_temp = {new_route_ID, 1, preFlow};

        /* Note: traffic flow will be modified further, no worry here */
        RRIndex_roadID_slice_insertion[roadID][index][inTime].push_back(features_temp);
    }

    // Step 2.1. Estimate travel time for newly inserted route on this road
    // -----------------------------------------------------------------------------

    // Update traffic flow when analyze the newly inserted route record
    traffic_flow_origin = preFlow;
    traffic_flow_insertion = preFlow + 1;

    // Update traffic flow in RR-Index
    map<int, vector<vector<int>>>::iterator itFind1;

    itFind1 = RRIndex_roadID_slice_insertion[roadID][index].find(inTime);
    itFind1->second.back()[2] = traffic_flow_insertion;

    // Estimate new route record's leaving time
    int te = 0;

    if (catching)
    {
        if (range)
        {
            /* This version is preferred but need further develop */
            cout << "This version is preferred but need further develop" << endl;
        }
        else
        {
            /* This version is preferred but need further develop */
            cout << "This version is preferred but need further develop" << endl;
        }
    }
    else
    {
        if (range)
        {
            /* This version is preferred but need further develop */
            cout << "This version is preferred but need further develop" << endl;
        }
        else
        {
            te = tm * (1 + sigma * pow(traffic_flow_insertion/varphi, beta));
        }
    }

    // Estimate leave time of the newly inserted vehicle
    time_new_leave = inTime + te;

    // Store route record of the new route with leaving status in a list
    /* Note: Compared to the route record with entrance status, route record with leaving status do not need to estimate
     * travel time. Thus, after inserting route records with entrance status and estimate their updated leaving time, we
     * insert leaving route records and update their traffic flow */
    if (insert_list.find(time_new_leave) == insert_list.end())
    {
        /* Note: traffic flow here is a temporary value, and we will modify it further */
        insert_list.insert(pair<int, vector<vector<int>>>((time_new_leave), {{new_route_ID, 0, 0}}));
    }
    else
    {
        insert_list[time_new_leave].push_back({new_route_ID, 0, 0});
    }

    // Special situation: the min travel time is smaller than 1
    /* After insert its route record with entrance status, we should add its route record with leaving status */
    /* There is no route records are affected under this situation */
    if (tm < 1)
    {
        int routeID = new_route_pair.first;

        vector<int> affected_route_node = route_node_new[routeID];
        vector<int> affected_route_road = single_route_node_2_route_road(affected_route_node);

        /* int next_roadID = find_next_roadID(affected_route_road, roadID); */
        // Debug
        int next_roadID = find_next_roadID(affected_route_road, roadID, "B_1", routeID);

        // Insert newly inserted route records with leaving status
        if (next_roadID == -1)  // Last road
        {
            // Correctness check
            if ((traffic_flow_insertion - 1) < 0)
                cout << "Error. Traffic flow is smaller than 0. " << endl;

            if (RRIndex_roadID_slice_insertion[roadID][index].find(time_new_leave) ==
            RRIndex_roadID_slice_insertion[roadID][index].end())
            {
                RRIndex_roadID_slice_insertion[roadID][index].
                insert(pair<int, vector<vector<int>>>(time_new_leave, {{new_route_ID, 0, traffic_flow_insertion - 1}}));
            }
            else
            {
                RRIndex_roadID_slice_insertion[roadID][index][time_new_leave].
                push_back({new_route_ID, 0, traffic_flow_insertion - 1});
            }

            return null;
        }
        else
        {
            // Correctness check
            if ((traffic_flow_insertion - 1) < 0)
                cout << "Error. Traffic flow is smaller than 0. " << endl;

            if (RRIndex_roadID_slice_insertion[roadID][index].find(time_new_leave) ==
            RRIndex_roadID_slice_insertion[roadID][index].end())
            {
                RRIndex_roadID_slice_insertion[roadID][index].
                insert(pair<int, vector<vector<int>>>(time_new_leave, {{new_route_ID, 0, traffic_flow_insertion - 1}}));
            }
            else
            {
                RRIndex_roadID_slice_insertion[roadID][index][time_new_leave].
                push_back({new_route_ID, 0, traffic_flow_insertion - 1});
            }

            // Return entrance time for the next road
            get<0>(return_temp) = inTime;
            roadSegmentAffected.push_back(make_pair(make_pair(next_roadID,routeID), return_temp));

            return roadSegmentAffected;
        }
    }

    // Step 3. Update operation for the affected original route record
    // -----------------------------------------------------------------------------

    // Step 3.1. Iteration for route records start from new's next (Core Design).
    // -----------------------------------------------------------------------------
    int time_current = 0;
    int routeID_current = 0;

    // Locate the inserted time (key)
    /* Start iteration from the next of newly inserted route record */
    map<int, vector<vector<int>>>::iterator itrInser;
    itrInser = RRIndex_roadID_slice_insertion[roadID][index].find(inTime);

    // Correctness check
    if (itrInser == RRIndex_roadID_slice_insertion[roadID][index].end())
        cout << "Error. Time Records Do Not Contain Others Except the New Driving In One" << endl;

    /* Check if traffic flow change from new's next route record to the last one (can terminate early) */
    map<int, vector<vector<int>>>::iterator itr;
    for (itr = ++itrInser; itr != RRIndex_roadID_slice_insertion[roadID][index].end();)
    {
        bool bBreak = false;
        bool bBreak2 = false;

        // Define number of route records under current time (key)
        int dynamicSize = itr->second.size();

        for (int i = 0; i < dynamicSize; i++)
        {
            time_current = itr->first;
            routeID_current = itr->second[i][0];

            // Step 3.2: Check if insert updated original route record before current time (key)
            /* Iterate to a specific route record here, check if the affected original route record should be updated
             * in front of the current route record. */
            // -----------------------------------------------------------------------------
            int routeIDI;

            map<int, vector<vector<int>>>::iterator itrI;
            // ??? 不明白为什么从++insert_list开始
            /* for (itrI = ++insert_list.begin(); itrI != insert_list.end();) */
            for (itrI = insert_list.begin(); itrI != insert_list.end();)
            {
                int timeI = itrI->first;

                if (timeI < time_current)
                {
                    traffic_flow_insertion -= 1;
                    routeIDI = itrI->second[i][0];

                    // route records in "inset_list" should be inserted here with entrance status
                    if (itrI->second[i][1] == 1)     // Status: In
                    {
                        cout << "Error. Why insert in the first road has entrance status?" << endl;
                        /* if (RRIndex_roadID_slice_insertion[roadID][index].find(timeI) ==
                        RRIndex_roadID_slice_insertion[roadID][index].end())
                        {
                            // ??? 为什么不需要更新traffic flow here
                            RRIndex_roadID_slice_insertion[roadID][index].
                            insert(pair<int, vector<vector<int>>>(timeI, {{routeIDI, 1, traffic_flow_insertion}}));
                        }
                        else
                        {
                            RRIndex_roadID_slice_insertion[roadID][index][timeI].
                            push_back({routeIDI, 1, traffic_flow_insertion});
                        } */
                    }
                    else    // Status: Out
                    {
                        if (RRIndex_roadID_slice_insertion[roadID][index].find(timeI) ==
                        RRIndex_roadID_slice_insertion[roadID][index].end())
                        {
                            RRIndex_roadID_slice_insertion[roadID][index].
                            insert(pair<int, vector<vector<int>>>(timeI, {{routeIDI, 0, traffic_flow_insertion}}));
                        }
                        else
                        {
                            RRIndex_roadID_slice_insertion[roadID][index][timeI].
                            push_back({routeIDI, 0, traffic_flow_insertion});
                        }
                    }

                    // Erase it from “insert_list”
                    itrI = insert_list.erase(itrI);
                    continue;
                }

                // Next time (key) and route records
                ++itrI;
            }

            // Step 3.4. If route record is entrance status
            // -----------------------------------------------------------------------------

            map<int, vector<vector<int>>>::iterator itrIPre;
            int RouteIDIPre;
            int te_current = 0;
            if (itr->second[i][1] == 1)
            {
                // If Terminal Condition Is Active
                if (terminal_condition == 1)
                {
                    // Terminate early (do not check record time here)
                    /* record time refers to vehicle's entrance time when traffic flow do not change, if traffic flows
                     * still do not change when the vehicle leave this road, we terminate the procedure on this road. */
                    /* If current time (key) is the last one of current route, and current route record has an entrance
                     * status, its travel time on this road must be one, we can direct stop further operation. */
                    // ??? 这里仅判断了 time 是最后，但是其中的 route record 并没有判断，需要判断嘛
                    if (itr == RRIndex_roadID_slice_insertion[roadID][index].end())
                        break;

                    // Situation: Terminal condition is active but still not reach the record time
                    if (time_current <= record_time)
                    {
                        if (traffic_flow_origin != traffic_flow_insertion)
                        {
                            if (range)
                            {
                                /* Same range: Terminate condition still active, continue iteration procedure until
                                 * current time is bigger than the record time. */
                                /* Note: Although same range, since traffic flow change, flow number still need to
                                 * update in the RR-Index. Such as: itr->second[i][2] = traffic_flow_insertion; */

                                /* Not same range: Terminate condition check stop, continue iteration until two traffic
                                 * flow same. */

                                /* This version is preferred but need further develop */
                                cout << "This version is preferred but need further develop" << endl;
                            }
                            else
                            {
                                terminal_condition = 0;

                                // Update traffic flow
                                traffic_flow_origin += 1;
                                traffic_flow_insertion += 1;

                                // Update traffic flow in route record
                                itr->second[i][2] = traffic_flow_insertion;

                                routeID_current = itr->second[i][0];

                                // Estimate new travel time
                                if (catching)
                                {
                                    if (range)
                                    {
                                        /* This version is preferred but need further develop */
                                        cout << "This version is preferred but need further develop" << endl;
                                    }
                                    else
                                    {
                                        /* This version is preferred but need further develop */
                                        cout << "This version is preferred but need further develop" << endl;
                                    }
                                }
                                else
                                {
                                    if (range)
                                    {
                                        /* This version is preferred but need further develop */
                                        cout << "This version is preferred but need further develop" << endl;
                                    }
                                    else
                                    {
                                        te_current = tm * (1 + sigma * pow(traffic_flow_insertion/varphi, beta));
                                    }
                                }

                                // Add estimated travel time to "insert_list" for further insertion
                                /* Note: Its traffic flow will be updated further */
                                if (insert_list.find(time_current + te_current) == insert_list.end())
                                    insert_list.insert(pair<int, vector<vector<int>>>((time_current + te_current), {{routeID_current, 0, 0}}));
                                else
                                    insert_list[time_current + te_current].push_back({routeID_current, 0, 0});

                                // Add original route ID to the "deletion_list" to delete the original one further
                                deletion_list.push_back(routeID_current);

                                // Find next road ID
                                /* As its leaving time change on current road, its entrance time for its next road
                                 * should also be updated, which may propagate to other routes */

                                vector<int> affected_route_node = route_node_new[routeID_current];
                                vector<int> affected_route_road = single_route_node_2_route_road(affected_route_node);

                                /* int next_roadID = find_next_roadID(affected_route_road, roadID); */
                                // Debug
                                int next_roadID = find_next_roadID(affected_route_road, roadID, "C_1", routeID_current);

                                // Check if next affected road belongs to the newly inserted route
                                /* 1. If next affected road belongs to the newly inserted route, we should insert new
                                 * route record */
                                /* 2. If next affected road is not belong to the new, we do not need to insert a new one.
                                 * We delete original affected ones and insert updated ones */
                                std::vector<int>::iterator itrFindNew;
                                itrFindNew = std::find(new_route_road.begin(), new_route_road.end(), next_roadID);

                                // Check if returned value already exit same affected road ID
                                affected_road_ID.clear();

                                for (int i = 0; i < roadSegmentAffected.size(); i++)
                                {
                                    int roadID_temp = roadSegmentAffected[i].first.first;
                                    affected_road_ID.push_back(roadID_temp);
                                }

                                std::vector<int>::iterator itFindExit;
                                itFindExit = std::find(affected_road_ID.begin(), affected_road_ID.end(), next_roadID);

                                if (itFindExit != affected_road_ID.end())
                                {
                                    int position = itFindExit - affected_road_ID.begin();
                                    return_temp = roadSegmentAffected[position].second;

                                    if (get<1>(return_temp).find(time_current + te_current) == get<1>(return_temp).end())
                                        get<1>(return_temp).insert(pair<int, vector<vector<int>>>((time_current + te_current), {{routeID_current, 1, 0}}));
                                    else
                                        get<1>(return_temp)[time_current + te_current].push_back({routeID_current, 1, 0});

                                    get<2>(return_temp).push_back(routeID_current);

                                    // If the next affected road belongs to new route
                                    if (itrFindNew != new_route_road.end())
                                    {
                                        get<0>(return_temp) = time_new_leave;
                                        roadSegmentAffected[position].second = return_temp;
                                    }
                                    else
                                    {
                                        /* "0" refers to the next road are not contained by the new route */
                                        get<0>(return_temp) = 0;
                                        roadSegmentAffected[position].second = return_temp;
                                    }
                                }
                                else
                                {
                                    // If the next affected road belongs to new route
                                    if (itrFindNew != new_route_road.end())
                                    {
                                        get<0>(return_temp) = time_new_leave;

                                        get<1>(return_temp).clear();
                                        if (get<1>(return_temp).find(time_current + te_current) == get<1>(return_temp).end())
                                            get<1>(return_temp).insert(pair<int, vector<vector<int>>>((time_current + te_current), {{routeID_current, 1, 0}}));
                                        else
                                            get<1>(return_temp)[time_current + te_current].push_back({routeID_current, 1, 0});

                                        get<2>(return_temp).clear();
                                        get<2>(return_temp).push_back(routeID_current);

                                        roadSegmentAffected.
                                        push_back(make_pair(make_pair(next_road_ID, routeID_current), return_temp));
                                    }
                                    else
                                    {
                                        get<0>(return_temp) = 0;

                                        get<1>(return_temp).clear();
                                        if (get<1>(return_temp).find(time_current + te_current) == get<1>(return_temp).end())
                                            get<1>(return_temp).insert(pair<int, vector<vector<int>>>((time_current + te_current), {{routeID_current, 1, 0}}));
                                        else
                                            get<1>(return_temp)[time_current + te_current].push_back({routeID_current, 1, 0});

                                        get<2>(return_temp).clear();
                                        get<2>(return_temp).push_back(routeID_current);

                                        roadSegmentAffected.push_back(make_pair(make_pair(next_roadID,routeID_current), return_temp));
                                    }
                                }
                            }
                        }
                        else
                        {
                            // Two traffic flow same, update traffic flow and continue iteration
                            /* Note: If two traffic flows are same, the traffic flow of original route records in
                             * RR-Index is same as the updated version, so we do not need to update traffic flow in
                             * RR-Index. We only need to increase traffic flow here for the further operation */

                            traffic_flow_origin += 1;
                            traffic_flow_insertion += 1;
                        }
                    }
                    else{
                        // Active terminal condition, no further route records are affected
                        /* Note: When terminal condition activate, and "insert_list" is not empty,
                         * we iterate to insert route record only */
                        if (terminal and insert_list.empty() and traffic_flow_insertion == 0)
                        {
                            bBreak2 = true;
                            break;
                        }
                    }
                }
                else
                {
                    // Update traffic flow
                    traffic_flow_origin += 1;
                    traffic_flow_insertion += 1;

                    if (traffic_flow_origin == traffic_flow_insertion)
                    {
                        // Active terminal condition
                        terminal_condition = 1;

                        // Update traffic flow to RR-Index
                        itr->second[i][2] = traffic_flow_insertion;

                        // Estimate record time (determine if no further affect)
                        if (catching)
                        {
                            if (range)
                            {
                                /* This version is preferred but need further develop */
                                cout << "This version is preferred but need further develop" << endl;
                            }
                            else
                            {
                                /* This version is preferred but need further develop */
                                cout << "This version is preferred but need further develop" << endl;
                            }
                        }
                        else
                        {
                            if (range)
                            {
                                /* This version is preferred but need further develop */
                                cout << "This version is preferred but need further develop" << endl;
                            }
                            else
                            {
                                te_current = tm * (1 + sigma * pow(traffic_flow_insertion/varphi, beta));
                            }
                        }

                        /* Note: Because traffic flow same, RR-Index do not need to update travel time.
                         * Estimation here is to define record time */
                        record_time = time_current + te_current;
                    }
                    else
                    {
                        // Update traffic flow to RR-Index
                        itr->second[i][2] = traffic_flow_insertion;

                        routeID_current = itr->second[i][0];

                        // Estimate new travel time as traffic flow change
                        if (catching)
                        {
                            if (range)
                            {
                                /* This version is preferred but need further develop */
                                cout << "This version is preferred but need further develop" << endl;
                            }
                            else
                            {
                                /* This version is preferred but need further develop */
                                cout << "This version is preferred but need further develop" << endl;
                            }
                        }
                        else
                        {
                            if (range)
                            {
                                /* This version is preferred but need further develop */
                                cout << "This version is preferred but need further develop" << endl;
                            }
                            else
                            {
                                te_current = tm * (1 + sigma * pow(traffic_flow_insertion/varphi, beta));
                            }
                        }

                        // Add corresponding route record with leaving status to "insert_list"
                        /* As its travel time are affected, we should delete the original one and insert a new one */
                        if (insert_list.find(time_current + te_current) == insert_list.end())
                            insert_list.insert(pair<int, vector<vector<int>>>((time_current + te_current), {{routeID_current, 0, 0}}));
                        else
                            insert_list[time_current + te_current].push_back({routeID_current, 0, 0});
                        // Add original route record to "deletion_list"
                        deletion_list.push_back(routeID_current);


                        vector<int> affected_route_node = route_node_new[routeID_current];
                        vector<int> affected_route_road = single_route_node_2_route_road(affected_route_node);
                        /* int unaffected_next_roadSegmentID = find_next_roadID(affected_route_road, roadID); */
                        // Debug
                        int unaffected_next_roadSegmentID = find_next_roadID(affected_route_road, roadID, "D_1", routeID_current);

                        // Check if next affected road belongs to the newly inserted route
                        std::vector<int>::iterator itrFindNew;
                        itrFindNew = std::find(new_route_road.begin(), new_route_road.end(), unaffected_next_roadSegmentID);

                        // Find IF Next Road Belongs to Affected Road
                        affected_road_ID.clear();

                        for (int i = 0; i < roadSegmentAffected.size(); i++)
                        {
                            int roadID_temp = roadSegmentAffected[i].first.first;
                            affected_road_ID.push_back(roadID_temp);
                        }

                        std::vector<int>::iterator itFindExit;
                        itFindExit = std::find(affected_road_ID.begin(), affected_road_ID.end(), unaffected_next_roadSegmentID);

                        if (itFindExit != affected_road_ID.end())
                        {
                            int position = itFindExit - affected_road_ID.begin();
                            return_temp = roadSegmentAffected[position].second;

                            if (get<1>(return_temp).find(time_current + te_current) == get<1>(return_temp).end())
                                get<1>(return_temp).insert(pair<int, vector<vector<int>>>((time_current + te_current), {{routeID_current, 1, 0}}));
                            else
                                get<1>(return_temp)[time_current + te_current].push_back({routeID_current, 1, 0});

                            get<2>(return_temp).push_back(routeID_current);

                            // If the next affected road belongs to new route
                            if (itrFindNew != new_route_road.end())
                            {
                                get<0>(return_temp) = time_new_leave;
                                roadSegmentAffected[position].second = return_temp;
                            }
                            else
                            {
                                /* "0" refers to the next road are not contained by the new route */
                                get<0>(return_temp) = 0;
                                roadSegmentAffected[position].second = return_temp;
                            }
                        }
                        else
                        {
                            // If the next affected road belongs to new route
                            if (itrFindNew != new_route_road.end())
                            {
                                get<0>(return_temp) = time_new_leave;

                                get<1>(return_temp).clear();

                                if (get<1>(return_temp).find(time_current + te_current) == get<1>(return_temp).end())
                                    get<1>(return_temp).insert(pair<int, vector<vector<int>>>((time_current + te_current), {{routeID_current, 1, 0}}));
                                else
                                    get<1>(return_temp)[time_current + te_current].push_back({routeID_current, 1, 0});

                                get<2>(return_temp).clear();
                                get<2>(return_temp).push_back(routeID_current);

                                roadSegmentAffected.push_back(make_pair(make_pair(next_road_ID,routeID_current), return_temp));
                            }
                            else
                            {
                                get<0>(return_temp) = 0;

                                get<1>(return_temp).clear();
                                if (get<1>(return_temp).find(time_current + te_current) == get<1>(return_temp).end())
                                    get<1>(return_temp).insert(pair<int, vector<vector<int>>>((time_current + te_current),{{routeID_current, 1, 0}}));
                                else
                                    get<1>(return_temp)[time_current + te_current].push_back({routeID_current, 1, 0});

                                get<2>(return_temp).clear();
                                get<2>(return_temp).push_back(routeID_current);

                                roadSegmentAffected.push_back(make_pair(make_pair(unaffected_next_roadSegmentID,routeID_current), return_temp));
                            }
                        }
                    }
                }
            }
            // Step 3.5. If route record is leaving status.
            // -----------------------------------------------------------------------------
            else
            {
                // Compare route records with "deletion_list"
                /* Because in the first road of new route, only affected original travel time should be deleted and
                 * insert the updated one. We only consider "deletion_list" here */
                if (!deletion_list.empty())
                {
                    int deleID;

                    /* If current routeID does not match any routeID in "deletion_list",
                     * update traffic flow and iterate to the next route record */
                    int deletion_list_size = deletion_list.size();

                    for (int l = 0; l < deletion_list.size(); l++)
                    {
                        deleID = deletion_list[l];

                        if (routeID_current == deleID)
                        {
                            traffic_flow_origin -= 1;

                            // Delete route record
                            itr->second.erase(itr->second.begin() + i);

                            // Update route record size as deletion
                            dynamicSize = itr->second.size();
                            // Back to previous route record
                            /* Will locate to the next route record in the next iteration */
                            i--;

                            // Delete this route record in the "deletion_list"
                            vector<int>::iterator k = deletion_list.begin() + l;
                            deletion_list.erase(k);

                            /* If route records under this time (key) are empty after deletion, this key-value pair
                             * should be deleted */
                            if (itr->second.empty())
                            {
                                itr = RRIndex_roadID_slice_insertion[roadID][index].erase(itr);

                                if (RRIndex_roadID_slice_insertion[roadID][index].size() == 0)
                                {
                                    /* If all times (key) under this time slice is empty after deletion,
                                     * it is unnecessary to iterate route record under this time slice */
                                    // ??? the loop can check empty to stop iteration,, maybe it is unnecessary to set this "bBreak2"
                                    bBreak2 = true;
                                    break;
                                }

                                /* After deletion, the current time (key) is empty. It is unnecessary to
                                 * iterate the rest route record and continues to the next time (key) */
                                // ??? "dynamic_size" is empty now, maybe it is unnecessary to set this "bBreak"
                                bBreak = true;
                                break;
                            }

                            /* After deletion, the current route record is deleted. It is unnecessary to
                             * iterate the "deletion_list" to decide deletion */
                            break;
                        }
                    }

                    // ??? Not sure if we should update traffic flow here
                    // ??? If no route ID should be deleted in the "deletion_list", we may need to update traffic flow
                    if (deletion_list_size == deletion_list.size())
                    {
                        // Update traffic flow
                        traffic_flow_insertion -= 1;
                        traffic_flow_origin -= 1;

                        // Update in RR-Index
                        itr->second[i][2] = traffic_flow_insertion;
                    }
                }
                else
                {
                    // Update traffic flow
                    traffic_flow_insertion -= 1;
                    traffic_flow_origin -= 1;

                    // Update in RR-Index
                    itr->second[i][2] = traffic_flow_insertion;
                }
            }
        }
        if(bBreak)
            continue;
        if(bBreak2)
            break;
        dynamicSize = itr->second.size();
        ++itr;
    }

    // Step 3.6 Insert Rest Insert Elements
    // -----------------------------------------------------------------------------
    /* If iteration end but "insert_list" are not empty, time (key) in "insert_list" is later than times (key) in
     * original time slice. We should insert them and update corresponding route record. */

    // Find the route record
    map<int, vector<vector<int>>>::iterator itrEnd;
    itrEnd = --RRIndex_roadID_slice_insertion[roadID][index].end();

    // Define traffic flow for the last route record
    int FlowEnd = itrEnd->second.back()[2];

    int timeRest, RouteIDRest, FlowRest;
    map<int, vector<vector<int>>>::iterator itrRestI;

    for (itrRestI = insert_list.begin(); itrRestI != insert_list.end(); ++itrRestI)
    {
        for (int i = 0; i < itrRestI->second.size(); i++)
        {
            timeRest = itrRestI->first;
            RouteIDRest = itrRestI->second[i][0];

            // Update traffic flow
            FlowRest = FlowEnd - 1;

            if (RRIndex_roadID_slice_insertion[roadID][index].find(timeRest) == RRIndex_roadID_slice_insertion[roadID][index].end())
                RRIndex_roadID_slice_insertion[roadID][index].
                insert(pair<int, vector<vector<int>>>(timeRest, {{RouteIDRest, 0, FlowRest}}));
            else
                RRIndex_roadID_slice_insertion[roadID][index][timeRest].push_back({RouteIDRest, 0, FlowRest});
        }
    }

    // Step 4: Print Time Records
    // -----------------------------------------------------------------------------
    map<int, vector<vector<int>>>::iterator itrPrint;

    map<int, vector<vector<int>>>* time_record_pairs_current;
    time_record_pairs_current = &RRIndex_roadID_slice_insertion[roadID][index];

    if (print)
    {
        cout << "Time and route record pairs size is: " << time_record_pairs_current->size() << endl;
        cout << "roadID: " << roadID << " with time slice: " << index << endl;

        for (itrPrint = time_record_pairs_current->begin(); itrPrint != time_record_pairs_current->end(); ++itrPrint)
        {
            for (int i = 0; i < itrPrint->second.size(); i++)
            {
                cout << " time " << itrPrint->first << " routeID " << itrPrint->second[i][0];
                cout << " status " << itrPrint->second[i][1] << " flow " << itrPrint->second[i][2] << "||";
            }
        }
        cout << endl;
    }

    // Step 5: Update Result
    // -----------------------------------------------------------------------------
    /* RRIndex_roadID_slice_insertion[roadID][index] = timeRecords;*/

    // Step 6:
    // -----------------------------------------------------------------------------

    /* Until now, the return value contains affected further road IDs with affected route records.
     * If these road IDs do not contain the next road ID contained in the newly insert route, we add it to the return value */

    std::vector<int>::iterator itr_findNextExist;

    affected_road_ID.clear();
    for (int i = 0; i < roadSegmentAffected.size(); i++)
    {
        affected_road_ID.push_back(roadSegmentAffected[i].first.first);
    }

    itr_findNextExist = std::find(affected_road_ID.begin(), affected_road_ID.end(), next_road_ID);

    tuple<int, map<int, vector<vector<int>>>, vector<int>> fillReturn;

    if (next_road_ID != -1 and itr_findNextExist == affected_road_ID.end())
    {
        get<0>(fillReturn) = time_new_leave;
        get<1>(fillReturn).clear();
        get<2>(fillReturn).clear();

        roadSegmentAffected.push_back(make_pair(make_pair(next_road_ID,new_route_pair.first),fillReturn));
    }

    // Step 7: Print Further Affected Road Segment
    // -------------------------------------------
    if (print)
    {
        cout << endl;
        cout << "Step 4: Returned Results" << endl;
        cout << "-----------------------------------------" << endl;

        for (int i = 0; i < roadSegmentAffected.size(); i++)
        {
            cout << " route ID is: " << roadSegmentAffected[i].first.second << " with entrance time: " << get<0>(roadSegmentAffected[i].second) << endl;

            map<int, vector<vector<int>>>::iterator itrInsertPrint;

            for (itrInsertPrint = get<1>(roadSegmentAffected[i].second).begin(); itrInsertPrint != get<1>(roadSegmentAffected[i].second).end(); ++itrInsertPrint)
            {
                for (int j = 0; j < itrInsertPrint->second.size(); j++)
                {
                    cout << " time " << itr->first << " routeID " << itr->second[i][0];
                }
            }
            cout << endl;
        }
        cout << endl;
    }
    return roadSegmentAffected;
}


// This Is Insertion Operation for The Further Road of New Route or Propagated Road
// ??? 为什么 deletion_pre 没有添加过值
vector<pair<pair<int,int>, tuple<int, map<int, vector<vector<int>>>, vector<int>>>> Graph::updateOperationFurther(
        pair<int,int> &roadID_routeID_pair, int &inTime, pair<int, vector<int>> &new_route_pair, pair<int, vector<int>> &route_prop,
        map<int, vector<vector<int>>> &insert_pre, vector<int> &deletion_pre, bool &parallel, bool &terminal,
        bool &range, bool print, bool catching)
{
    // ???
    bool STAll;
    STAll = true;

    // 1. Variable initialization
    // -----------------------------------------------------------------------------
    // Initialize traffic flow with and without insertion operation
    /* traffic flow before and after the insertion operation. */
    int traffic_flow_origin = 0;
    int traffic_flow_insertion = 0;

    // Check the status of terminal condition: 0 -> Not, 1 -> Yes
    int terminal_condition = 0;

    // Record time stamp when traffic flow before and after insertion operation same for further termination check
    int record_time = 0;

    // Initialize insert and deletion list
    /* As insertion, part of original route records should be updated (delete first and insert another one) */
    map<int, vector<vector<int>>> insert_list;
    vector<int> deletion_list;
    /* Initialize value of "deletion_list". Part of delayed route records should be deleted in this road,
     * and insert an updated one. */
    deletion_list = deletion_pre;

    // Capture new route ID and new route constructed by roads from argument
    int new_route_ID = new_route_pair.first;
    vector<int> new_route_road = new_route_pair.second;

    // Initialize returned values
    /* As new vehicle enter current road, we re-simulate all affected original route records.
     * Then, this function returns two kinds of information:
     * 1. New vehicle's entrance time for the next road
     * 2. Part of original route records will be delayed, their update entrance time for the next road */
    vector<pair<pair<int,int>, tuple<int, map<int, vector<vector<int>>>, vector<int>>>> roadSegmentAffected;
    /* "null" if current road is the last one of this route */
    vector<pair<pair<int,int>, tuple<int, map<int, vector<vector<int>>>, vector<int>>>> null;

    // Define belonged time slice
    int hour, index;

    // ??? 这里为什么要定义成 pair 的形式呢，后面没用到其对应的 second
    int roadID = roadID_routeID_pair.first;

    // Define node ID1 and node ID2
    int current_node = roadID2NodeID[roadID].first;
    int next_node = roadID2NodeID[roadID].second;
    int next_roadID;

    // Define the minimal travel time
    EdgeInfo road_features = edge_id_to_features[roadID];
    int tm = road_features.minimal_travel_time;

    vector<int> affected_road_ID;
    int time_new_leave;

    tuple<int, map<int, vector<vector<int>>>, vector<int>> return_temp;

    // If current road is the last one without any information input to update
    if (inTime == 0 and insert_pre.size() == 0 and deletion_pre.size() == 0)
        return null;

    // Step 2. Insert newly inserted route record into RR-Index and estimate its leaving time
    // -----------------------------------------------------------------------------

    // 2.1. If current road belongs to newly inserted route
    // -----------------------------------------------------------------------------
    if (inTime != 0)
    {
        // Print route records before manipulation
        if (print == true)
        {
            map<int, vector<vector<int>>>::iterator itrPrintV1;

            cout << endl;
            cout << "Step 2: route records before manipulation" << endl;
            cout << "-----------------------------------------" << endl;

            cout << "time (key) size is: " << RRIndex_roadID_slice_insertion[roadID][index].size() << endl;
            cout << "roadID: " << roadID << " with time slice: " << index << endl;

            for (itrPrintV1 = RRIndex_roadID_slice_insertion[roadID][index].begin(); itrPrintV1 != RRIndex_roadID_slice_insertion[roadID][index].end(); ++itrPrintV1)
            {
                for (int i = 0; i < itrPrintV1->second.size(); i++)
                {
                    cout << " time " << itrPrintV1->first << " routeID " << itrPrintV1->second[i][0];
                    cout << " status " << itrPrintV1->second[i][1] << " flow " << itrPrintV1->second[i][2] << "||";
                }
            }
            cout << endl;
        }

        // Step 2.2. Find time value (key) or insert a new one
        // -----------------------------------------------------------------------------
        /* For all times (key) in RR-Index, if inserted vehicle's entrance time already exist,
         * we directly query it. If not, we should insert a new time as key to further insert records. */

        // Define belonged time slice
        hour = time_2_hour(inTime);
        index = hour_2_index(hour);

        // Define traffic flow of previous record (value)
        /* The newly inserted traffic flow should be the previous plus one */
        int preFlow;
        /* Insert a temporary record (value) and update further */
        vector<int> features = {new_route_ID, 1, 0};

        map<int, vector<vector<int>>>::iterator itFind;
        itFind = RRIndex_roadID_slice_insertion[roadID][index].find(inTime);

        // If time (key) not be found, insert
        if (itFind == RRIndex_roadID_slice_insertion[roadID][index].end())
        {
            // Insert a new key with a temporary defined feature
            /* Note: traffic flow will be modified further, no worry here */
            RRIndex_roadID_slice_insertion[roadID][index].insert(pair<int, vector<vector<int>>>(inTime, {features}));

            // Query previous traffic flow and update inserted on based on it
            map<int, vector<vector<int>>>::iterator itPre;
            itPre = RRIndex_roadID_slice_insertion[roadID][index].find(inTime);

            if (itPre != RRIndex_roadID_slice_insertion[roadID][index].end())
            {
                // If newly inserted time (key) is the first one
                if (itPre == RRIndex_roadID_slice_insertion[roadID][index].begin())
                {
                    preFlow = 0;
                }
                else
                {
                    itPre = --itPre;
                    preFlow = itPre->second.back()[2];
                }
            }
            else
            {
                // If time (key) is not found, it must be an error since we just insert it
                cout << "No value found, please check Insertion." << endl;
                return null;
            }
        }
        else
        {
            // If time (key) is found, directly push back feature in it
            preFlow = itFind->second.back()[2];
            features = {new_route_ID, 1, preFlow};

            /* Note: traffic flow will be modified further, no worry here */
            RRIndex_roadID_slice_insertion[roadID][index][inTime].push_back(features);
        }

        // Step 2.3: Estimate travel time for newly inserted route on this road
        // -----------------------------------------------------------------------------

        // Update traffic flow when analyze the newly inserted route record
        traffic_flow_origin = preFlow;
        traffic_flow_insertion = preFlow + 1;

        // Update traffic flow in RR-Index
        map<int, vector<vector<int>>>::iterator itFind1;

        itFind1 = RRIndex_roadID_slice_insertion[roadID][index].find(inTime);
        itFind1->second.back()[2] = traffic_flow_insertion;

        // Estimate new route record's leaving time
        int te = 0;

        if (catching)
        {
            if (range)
            {
                /* This version is preferred but need further develop */
                cout << "This version is preferred but need further develop" << endl;
            }
            else
            {
                /* This version is preferred but need further develop */
                cout << "This version is preferred but need further develop" << endl;
            }
        }
        else
        {
            if (range)
            {
                /* This version is preferred but need further develop */
                cout << "This version is preferred but need further develop" << endl;
            }
            else
            {
                te = tm * (1 + sigma * pow(traffic_flow_insertion/varphi, beta));
            }
        }

        // Estimate leave time of the newly inserted vehicle
        time_new_leave = inTime + te;

        // Store route record of the new route with leaving status in a list
        if (insert_list.find(time_new_leave) == insert_list.end())
        {
            /* Note: traffic flow here is a temporary value, and we will modify it further */
            insert_list.insert(pair<int, vector<vector<int>>>((time_new_leave), {{new_route_ID, 0, 0}}));
        }
        else
        {
            insert_list[time_new_leave].push_back({new_route_ID, 0, 0});
        }
    }
    // Step 2.4: If current route record does not belong to the new route
    // -----------------------------------------------------------------------------
    else
    {
        /* We should locate the index of time slice for further iteration */
        std::map<int, vector<vector<int>>>::iterator itr_find1st;
        if (insert_pre.size() != 0)
        {
            itr_find1st = insert_pre.begin();

            // Define belonged time slice
            hour = time_2_hour(itr_find1st->first);
            index = hour_2_index(hour);

            time_new_leave = 0;

            // Print route records before manipulation
            if (print == true)
            {
                map<int, vector<vector<int>>>::iterator itrPrintV1;

                cout << endl;
                cout << "Step 2: time records before manipulation" << endl;
                cout << "-----------------------------------------" << endl;

                cout << "time (key) size is: " << RRIndex_roadID_slice_insertion[roadID][index].size() << endl;
                cout << "roadID: " << roadID << " with time slice: " << index << endl;

                for (itrPrintV1 = RRIndex_roadID_slice_insertion[roadID][index].begin(); itrPrintV1 != RRIndex_roadID_slice_insertion[roadID][index].end(); ++itrPrintV1)
                {
                    for (int i=0; i<itrPrintV1->second.size(); i++)
                    {
                        cout << " time " << itrPrintV1->first << " routeID " << itrPrintV1->second[i][0];
                        cout << " status " << itrPrintV1->second[i][1] << " flow " << itrPrintV1->second[i][2] << "||";
                    }
                }
                cout << endl;

            }
        }
        else
        {
            return null;
        }
    }

    // Special situation: the min travel time is smaller than 1
    /* After insert its route record with entrance status, we should add its route record with leaving status */
    /* There is no route records are affected under this situation */
    if (tm < 1)
    {
        int iNextRoadID;

        if (inTime == 0)
        {
            /* iNextRoadID = find_next_roadID(route_prop.second, roadID); */
            // Debug
            iNextRoadID = find_next_roadID(route_prop.second, roadID, "E_rest", route_prop.first);
        }
        else
        {
            /* iNextRoadID = find_next_roadID(new_route_pair.second, roadID); */
            // Debug
            iNextRoadID = find_next_roadID(new_route_pair.second, roadID, "F_rest", route_prop.first);
        }

        // Insert newly inserted route records with leaving status
        if (iNextRoadID == -1)  // Last road
        {
            if (RRIndex_roadID_slice_insertion[roadID][index].find(time_new_leave) == RRIndex_roadID_slice_insertion[roadID][index].end())
            {
                RRIndex_roadID_slice_insertion[roadID][index].
                insert(pair<int, vector<vector<int>>>(time_new_leave, {{new_route_ID, 0, traffic_flow_insertion-1}}));
            }
            else
            {
                RRIndex_roadID_slice_insertion[roadID][index][time_new_leave].
                push_back({new_route_ID, 0, traffic_flow_insertion - 1});
            }

            return null;
        }
        else
        {
            if (RRIndex_roadID_slice_insertion[roadID][index].find(time_new_leave) == RRIndex_roadID_slice_insertion[roadID][index].end())
            {
                RRIndex_roadID_slice_insertion[roadID][index].
                insert(pair<int, vector<vector<int>>>(time_new_leave, {{new_route_ID, 0, traffic_flow_insertion-1}}));
            }
            else
            {
                RRIndex_roadID_slice_insertion[roadID][index][time_new_leave].
                push_back({new_route_ID, 0, traffic_flow_insertion-1});
            }

            /* As travel time is smaller than 1, information can be directly propagate to the further road */
            tuple<int, map<int, vector<vector<int>>>, vector<int>> tempReturn;
            get<0>(tempReturn) = inTime;
            get<1>(tempReturn) = insert_pre;
            get<2>(tempReturn) = deletion_pre;

            if (inTime == 0)
                roadSegmentAffected.push_back(make_pair(make_pair(iNextRoadID,route_prop.first),tempReturn));
            else
                roadSegmentAffected.push_back(make_pair(make_pair(iNextRoadID,new_route_pair.first),tempReturn));

            return roadSegmentAffected;
        }
    }

    // Step 3. Update operation for the affected original route record
    // -----------------------------------------------------------------------------
    /* Concepts:
     * 1. "insert_pre" contains route records with entrance status, which is delayed in the previous road.
     * 2. "insert_list" contains route records with leaving status, which refers route is affected in the current road,
     * and delayed in the current road. */

    /* Main idea:
     * for the current time (key), we should insert time (key) in both "insert_pre" and "insert_list". For each
     * time (key) and route records in "insert_pre", we compare all time (key) and route records in "insert_list".
     * Then, when iterate all time in "insert_pre", we should compare each rest (un-inserted) time with current
     * time to make sure all time in both list is inserted with correct order. But there may have more efficient way. */

    int time_current = 0;
    int routeID_current = 0;
    int te_current = 0;
    int RouteIDIPre;

    // Find next road ID for different situations
    /* 1. newly inserted route, 2. affected route */
    if (inTime == 0)
    {
        /* next_roadID = find_next_roadID(route_prop.second, roadID); */
        // Debug
        next_roadID = find_next_roadID(route_prop.second, roadID, "G_rest", route_prop.first);

    }
    else
    {
        /* next_roadID = find_next_roadID(new_route_pair.second, roadID); */
        // Debug
        next_roadID = find_next_roadID(new_route_pair.second, roadID, "H_rest", route_prop.first);

    }

    map<int, vector<vector<int>>>::iterator itrInser;

    // Define the start iteration position
    /* 1. Newly inserted route: Iteration starts from the next position of the inserted route record. As the first
     * in and first out assumption, time (key) of affected route record from the previous road must be later than
     * the time (key) of newly inserted route from the previous road */
    /* 2. Affected route: Iteration starts from the first route record. Or a potential way is to sort time (key) in
     * the "insert_pre" and iterate start to the earliest time (key )*/

    if (inTime != 0)
        itrInser = ++RRIndex_roadID_slice_insertion[roadID][index].find(inTime);
    else
        itrInser = RRIndex_roadID_slice_insertion[roadID][index].begin();

    /* Check if traffic flow change from new's next route record to the last one (can terminate early) */
    map<int, vector<vector<int>>>::iterator itr;
    for (itr = itrInser; itr != RRIndex_roadID_slice_insertion[roadID][index].end();)
    {
        // Define break to stop iteration early under different situation
        bool bBreak = false;        // Continues to next round (route record)
        bool bBreak2 = false;       // Break loop

        // Define number of route records under current time (key)
        int dynamicSize = itr->second.size();

        for (int i = 0; i < dynamicSize; i++)
        {
            time_current = itr->first;
            routeID_current = itr->second[i][0];

            // Step 3.1. Insert route record in the "insert_pre"
            // -----------------------------------------------------------------------------
            int te_current = 0;

            map<int, vector<vector<int>>>::iterator itrInPre;
            int RouteIDInPre;

            for (itrInPre = insert_pre.begin(); itrInPre != insert_pre.end();)
            {
                int timeIPre = itrInPre->first;

                if (timeIPre < time_current)
                {
                    // Step 3.1.1 Insert route records in the "insert_list"
                    // -----------------------------------------------------------------------------
                    /* We should insert route record in both "insert_pre" and "insert_list" */
                    /* Here we insert route records in "insert_list" first and then "insert_pre" */
                    map<int, vector<vector<int>>>::iterator itrI0;
                    int RouteIDI0;
                    for (itrI0 = insert_list.begin(); itrI0 != insert_list.end();)
                    {
                        int timeI0 = itrI0->first;

                        if (timeI0 < timeIPre)
                        {
                            for (int j = 0; j < itrI0->second.size(); j++)
                            {
                                // Update traffic flow
                                /* "insert_list" only contain route records with leaving status */
                                traffic_flow_insertion -= 1;

                                RouteIDI0 = itrI0->second[j][0];

                                // Insert time records in the "insert_list"
                                if (itrI0->second[j][1] == 1)
                                {
                                    cout << "Error. Element in 'insert_list' is not leaving status." << endl;
                                }
                                else
                                {
                                    if (RRIndex_roadID_slice_insertion[roadID][index].find(timeI0) == RRIndex_roadID_slice_insertion[roadID][index].end())
                                    {
                                        RRIndex_roadID_slice_insertion[roadID][index].
                                                insert(pair<int, vector<vector<int>>>(timeI0, {{RouteIDI0, 0, traffic_flow_insertion}}));
                                    }
                                    else
                                    {
                                        RRIndex_roadID_slice_insertion[roadID][index][timeI0].
                                                push_back({RouteIDI0, 0, traffic_flow_insertion});
                                    }
                                }
                            }
                            // after insertion, remove the current route record in the "insert_list"
                            itrI0 = insert_list.erase(itrI0);
                        }
                        else
                        {
                            // Directly iterate to the next route record in the "insert_list"
                            ++itrI0;
                        }
                    }

                    // Step 3.1.2 Insert route records in "insert_pre"
                    // -----------------------------------------------------------------------------
                    int insert_pre_time_current;
                    int insert_pre_routeID_current;

                    for (int j = 0; j < itrInPre->second.size(); j++)
                    {
                        // Traffic flow update
                        /* "insert_pre'" Set Only Contains Time Records with Entrance Status */
                        traffic_flow_insertion += 1;

                        RouteIDInPre = itrInPre->second[j][0];

                        // Insert time records in the "insert_pre"
                        if (itrInPre->second[j][1] == 1){
                            if (RRIndex_roadID_slice_insertion[roadID][index].find(timeIPre) == RRIndex_roadID_slice_insertion[roadID][index].end())
                            {
                                RRIndex_roadID_slice_insertion[roadID][index].
                                insert(pair<int, vector<vector<int>>>(timeIPre, {{RouteIDInPre, 1, traffic_flow_insertion}}));
                            }
                            else
                            {
                                RRIndex_roadID_slice_insertion[roadID][index][timeIPre].
                                push_back({RouteIDInPre, 1, traffic_flow_insertion});
                            }
                        }
                        else
                        {
                            cout << "Error. Element in 'insert_pre' is not driving in status." << endl;
                        }

                        // Step 3.1.3 Estimate updated travel time for the inserted route record
                        // -----------------------------------------------------------------------------
                        /* Because route's leaving time on the previous road is affected, its entrance time on the
                         * current road is also affected. Then, its leaving time is also delayed, which should be
                         * estimated and update */
                        insert_pre_time_current = itrInPre->first;
                        insert_pre_routeID_current = itrInPre->second[j][0];

                        // Estimate new travel time
                        if (catching)
                        {
                            if (range)
                            {
                                /* This version is preferred but need further develop */
                                cout << "This version is preferred but need further develop" << endl;
                            }
                            else
                            {
                                /* This version is preferred but need further develop */
                                cout << "This version is preferred but need further develop" << endl;
                            }
                        }
                        else
                        {
                            if (range)
                            {
                                /* This version is preferred but need further develop */
                                cout << "This version is preferred but need further develop" << endl;
                            }
                            else
                            {
                                te_current = tm * (1 + sigma * pow(traffic_flow_insertion/varphi, beta));
                            }
                        }

                        // Add Estimated Time Record with Leaving Status to "I" List
                        if (insert_list.find(time_new_leave) == insert_list.end())
                        {
                            insert_list.insert(pair<int, vector<vector<int>>>((insert_pre_time_current + te_current),
                                                                              {{insert_pre_routeID_current, 0, 0}}));
                        }
                        else
                        {
                            insert_list[insert_pre_time_current+te_current].push_back({insert_pre_routeID_current, 0, 0});
                        }

                        // Step 3.1.3 Add route ID to "deletion_list"
                        // -----------------------------------------------------------------------------
                        /* Original route records with leaving status are changed,
                         * we should delete the original one and insert a new one */

                        // Step 3.1.4 Updated return value
                        // -----------------------------------------------------------------------------
                        /* Updated leaving time on the current road should be also updated to the next road */

                        // Decide if the next road belongs to new route
                        std::vector<int>::iterator itrFindNew;
                        itrFindNew = std::find(new_route_road.begin(), new_route_road.end(), next_roadID);

                        // Find IF Next Road Belongs to Affected Road
                        affected_road_ID.clear();

                        for (int k = 0; k < roadSegmentAffected.size(); k++)
                        {
                            int roadID_temp = roadSegmentAffected[k].first.first;
                            affected_road_ID.push_back(roadID_temp);
                        }

                        std::vector<int>::iterator itFindExit;
                        itFindExit = std::find(affected_road_ID.begin(), affected_road_ID.end(), next_roadID);

                        /* Because route records with leaving status in the previous road is delayed,
                         * its entrance time in the current road should be changed */
                        if (itFindExit != affected_road_ID.end())
                        {
                            int position = itFindExit - affected_road_ID.begin();

                            /* "return_temp" will be assigned to "roadSegmentAffected" further */
                            return_temp = roadSegmentAffected[position].second;

                            if (get<1>(return_temp).find(insert_pre_time_current + te_current) == get<1>(return_temp).end())
                            {
                                get<1>(return_temp).insert(pair<int, vector<vector<int>>>((
                                        insert_pre_time_current + te_current), {{insert_pre_routeID_current, 1, 0}}));
                            }
                            else
                            {
                                get<1>(return_temp)[insert_pre_time_current + te_current].
                                push_back({insert_pre_routeID_current, 1, 0});
                            }

                            get<2>(return_temp).push_back(insert_pre_routeID_current);

                            // Decide if "next_roadID" belongs to the newly inserted route
                            if (itrFindNew != new_route_road.end())
                            {
                                get<0>(return_temp) = time_new_leave;
                                roadSegmentAffected[position].second = return_temp;
                            }
                            else
                            {
                                get<0>(return_temp) = 0;
                                roadSegmentAffected[position].second = return_temp;
                            }
                        }
                        else
                        {
                            /* Because "roadSegmentAffected" does not contain next_roadID,
                             * we should initialize "return_temp" and add it to "roadSegmentAffected" */
                            if (itrFindNew != new_route_road.end())
                            {
                                get<0>(return_temp) = time_new_leave;

                                /* Make sure initialize values in "return_temp" */
                                get<1>(return_temp).clear();

                                if (get<1>(return_temp).find(insert_pre_time_current + te_current) == get<1>(return_temp).end())
                                {
                                    get<1>(return_temp).insert(pair<int, vector<vector<int>>>((
                                            insert_pre_time_current + te_current), {{insert_pre_routeID_current, 1, 0}}));
                                }
                                else
                                {
                                    get<1>(return_temp)[insert_pre_time_current + te_current].
                                            push_back({insert_pre_routeID_current, 1, 0});
                                }

                                get<2>(return_temp).clear();
                                get<2>(return_temp).push_back(insert_pre_routeID_current);

                                roadSegmentAffected.push_back(make_pair(make_pair(next_roadID,itrInPre->second[j][0]), return_temp));
                            }
                            else
                            {
                                get<0>(return_temp) = 0;

                                get<1>(return_temp).clear();
                                if (get<1>(return_temp).find(insert_pre_time_current + te_current) == get<1>(return_temp).end())
                                {
                                    get<1>(return_temp).insert(pair<int, vector<vector<int>>>((
                                            insert_pre_time_current + te_current), {{insert_pre_routeID_current, 1, 0}}));
                                }
                                else
                                {
                                    get<1>(return_temp)[insert_pre_time_current + te_current].
                                            push_back({insert_pre_routeID_current});
                                }

                                get<2>(return_temp).clear();
                                get<2>(return_temp).push_back(insert_pre_routeID_current);

                                roadSegmentAffected.push_back(make_pair(make_pair(next_roadID,itrInPre->second[j][0]), return_temp));
                            }
                        }
                    }
                    // Remove value in "insert_pre"
                    /* Because the current time in "insert_pre" is smaller than the current time,
                     * it must be inserted to the RR-Index */
                    itrInPre = insert_pre.erase(itrInPre);
                }
                else
                {
                    // Iterate to Next Time Record in "insert_pre"
                    ++itrInPre;
                }
            }

            // Step 3.1.4 Compare rest route records with the current one
            // -----------------------------------------------------------------------------
            int RouteIDI;
            map<int, vector<vector<int>>>::iterator itrI;

            for (itrI = insert_list.begin(); itrI != insert_list.end();)
            {
                int timeI = itrI->first;

                if (timeI < time_current)
                {
                    for (int j = 0; j < itrI->second.size(); j++)
                    {
                        // Update traffic flow
                        traffic_flow_insertion -= 1;
                        RouteIDI = itrI->second[j][0];

                        // RR-Index update
                        if (itrI->second[j][1] == 1)
                        {
                            cout << "Error. element in I is not leaving status." << endl;
                        }
                        else
                        {
                            if (RRIndex_roadID_slice_insertion[roadID][index].find(timeI) == RRIndex_roadID_slice_insertion[roadID][index].end())
                            {
                                RRIndex_roadID_slice_insertion[roadID][index].
                                insert(pair<int, vector<vector<int>>>(timeI,{{RouteIDI, 0, traffic_flow_insertion}}));
                            }
                            else
                            {
                                RRIndex_roadID_slice_insertion[roadID][index][timeI].
                                push_back({RouteIDI, 0, traffic_flow_insertion});
                            }
                        }
                    }

                    // Remove inserted value in "insert_list"
                    itrI = insert_list.erase(itrI);
                }
                else
                {
                    // Iterate to the next time (key)
                    ++itrI;
                }
            }

            // Stop iteration
            /* If no values route record need to be inserted or deleted and current road does not belong to new route */
            if (STAll == true)
            {
                if (inTime == 0 and insert_pre.size() == 0 and deletion_pre.size() == 0 and insert_list.size() == 0 and deletion_list.size() == 0)
                {
                    bBreak2 = true;
                    break;
                }
            }

            // Step 4. Update operation for the affected original route record
            // -----------------------------------------------------------------------------

            // Step 4.1 If route record is entrance status
            // -----------------------------------------------------------------------------
            map<int, vector<vector<int>>>::iterator itrIPre;

            if (itr->second[i][1] == 1)
            {
                // Step 4.1.1 Check termination
                // -----------------------------------------------------------------------------
                if (terminal_condition == 1 and insert_pre.size() == 0 and deletion_pre.size() == 0 and insert_list.size() == 0 and deletion_list.size() == 0)
                {
                    /* "traffic_flow_insertion" equals zero, refers effect as insertion end.
                     * Thus, if terminal condition is activate and "traffic_flow_insertion" equals to zero, do not need to
                     * consider record time. But considering record time may be a better choice */
                    if (terminal == true and insert_list.size() == 0 and traffic_flow_insertion == 0)
                    {
                        bBreak2 = true;
                        break;
                    }
                }

                // Step 4.1.2 Check deletion in "deletion_pre"
                // -----------------------------------------------------------------------------
                /* "deletion_pre" contains route records with entrance status.
                 * Their leaving time on the previous road is delayed. We should delete the original one on this road,
                 * and insert the updated one which contained in "insert_pre". */
                if (deletion_pre.size() != 0)
                {
                    if (routeID_current == deletion_pre[0])
                    {
                        // Update traffic flow
                        traffic_flow_origin += 1;

                        // Remove current route record
                        itr->second.erase(itr->second.begin() + i);
                        // Update number of route record under this time (key)
                        dynamicSize = itr->second.size();
                        i--;

                        // Remove current route ID in "deletion_pre" list
                        vector<int>::iterator k = deletion_pre.begin();
                        deletion_pre.erase(k);

                        /* If time (key) is empty after deletion, remove time (key) from RR-Index,
                         * and skip iteration to the next time (key) */
                        if(itr->second.size() == 0)
                        {
                            itr = RRIndex_roadID_slice_insertion[roadID][index].erase(itr);
                            bBreak = true;
                            break;
                        }

                        // continues to the next route record
                        continue;
                    }
                }

                // Step 4.1.3 Check if terminal condition is activated
                // -----------------------------------------------------------------------------
                if (terminal_condition == 1)
                {
                    // If Current Time Record Is The Last One of Current Route
                    // And Current Time Record (Value) Is The Last One In Time (Key)
                    if (itr == RRIndex_roadID_slice_insertion[roadID][index].end() and i == itr->second.size())
                    {
                        if (terminal == true  and insert_list.size() == 0 and traffic_flow_insertion == 0){
                            bBreak2 = true;
                            break;
                        }
                    }

                    // Step 4.1.4 Situation: Terminal condition is active but still not reach the record time
                    // -----------------------------------------------------------------------------
                    if (time_current <= record_time)
                    {
                        if (traffic_flow_origin != traffic_flow_insertion)
                        {
                            /* Same range: Terminate condition still active, continue iteration procedure until
                             * current time is bigger than the record time. */
                            /* Not same range: Terminate condition check stop, continue iteration until two traffic
                             * flow same. */

                            /* Note: Check if two traffic flows are located in the same range is preferred here */
                            cout << "Check if two traffic flows are located in the same range is preferred here" << endl;

                            if (range == false)
                            {
                                terminal_condition = 0;

                                // Update traffic flow
                                traffic_flow_origin += 1;
                                traffic_flow_insertion += 1;

                                // Update traffic flow in route record
                                itr->second[i][2] = traffic_flow_insertion;

                                routeID_current = itr->second[i][0];

                                // Estimate new travel time
                                if (catching)
                                {
                                    if (range)
                                    {
                                        /* This version is preferred but need further develop */
                                        cout << "This version is preferred but need further develop" << endl;
                                    }
                                    else
                                    {
                                        /* This version is preferred but need further develop */
                                        cout << "This version is preferred but need further develop" << endl;
                                    }
                                }
                                else
                                {
                                    if (range)
                                    {
                                        /* This version is preferred but need further develop */
                                        cout << "This version is preferred but need further develop" << endl;
                                    }
                                    else
                                    {
                                        te_current = tm * (1 + sigma * pow(traffic_flow_insertion/varphi, beta));
                                    }
                                }

                                // Add estimated travel time to "insert_list" for further insertion
                                /* Note: Its traffic flow will be updated further */
                                if (insert_list.find(time_current + te_current) == insert_list.end())
                                    insert_list.insert(pair<int, vector<vector<int>>>((time_current + te_current), {{routeID_current, 0, 0}}));
                                else
                                    insert_list[time_current + te_current].push_back({routeID_current, 0, 0});

                                // Add original route ID to the "deletion_list" to delete the original one further
                                deletion_list.push_back(routeID_current);

                                // Define next road ID
                                int next_roadID_current;
                                if (itr->second[i][0] == new_route_ID)
                                {
                                    next_roadID_current = next_roadID;
                                }
                                else
                                {
                                    vector<int> affected_route_node = route_node_new[itr->second[i][0]];
                                    vector<int> affected_route_road = single_route_node_2_route_road(affected_route_node);
                                    /* next_roadID_current = find_next_roadID(affected_route_road, roadID); */
                                    // Debug
                                    next_roadID_current = find_next_roadID(affected_route_road, roadID, "I_rest", itr->second[i][0]);
                                }

                                // Check if next affected road belongs to the newly inserted route
                                /* 1. If next affected road belongs to the newly inserted route, we should insert new
                                 * route record */
                                /* 2. If next affected road is not belong to the new, we do not need to insert a new one.
                                 * We delete original affected ones and insert updated ones */
                                std::vector<int>::iterator itrFindNew;
                                itrFindNew = std::find (new_route_road.begin(), new_route_road.end(), next_roadID_current);

                                // Check if returned value already exit same affected road ID
                                affected_road_ID.clear();
                                for (int k = 0; k < roadSegmentAffected.size(); k++)
                                {
                                    int roadID_temp = roadSegmentAffected[k].first.first;
                                    affected_road_ID.push_back(roadID_temp);
                                }

                                std::vector<int>::iterator itFindExit;
                                itFindExit = std::find(affected_road_ID.begin(), affected_road_ID.end(), next_roadID_current);

                                if (itFindExit != affected_road_ID.end())
                                {
                                    int position = itFindExit - affected_road_ID.begin();
                                    return_temp = roadSegmentAffected[position].second;

                                    if (get<1>(return_temp).find(time_current + te_current) == get<1>(return_temp).end())
                                        get<1>(return_temp).insert(pair<int, vector<vector<int>>>((time_current + te_current), {{routeID_current, 1, 0}}));
                                    else
                                        get<1>(return_temp)[time_current + te_current].push_back({routeID_current, 1, 0});

                                    get<2>(return_temp).push_back(routeID_current);

                                    // If the next affected road belongs to new route
                                    if (itrFindNew != new_route_road.end())
                                    {
                                        get<0>(return_temp) = time_new_leave;
                                        roadSegmentAffected[position].second = return_temp;
                                    }
                                    else
                                    {
                                        /* "0" refers to the next road are not contained by the new route */
                                        get<0>(return_temp) = 0;
                                        roadSegmentAffected[position].second = return_temp;
                                    }
                                }
                                else
                                {
                                    // If the next affected road belongs to new route
                                    if (itrFindNew != new_route_road.end())
                                    {
                                        get<0>(return_temp) = time_new_leave;

                                        get<1>(return_temp).clear();
                                        if (get<1>(return_temp).find(time_current + te_current) == get<1>(return_temp).end())
                                            get<1>(return_temp).insert(pair<int, vector<vector<int>>>((time_current + te_current), {{routeID_current, 1, 0}}));
                                        else
                                            get<1>(return_temp)[time_current + te_current].push_back({routeID_current, 1, 0});

                                        get<2>(return_temp).clear();
                                        get<2>(return_temp).push_back(routeID_current);

                                        roadSegmentAffected.push_back(make_pair(make_pair(next_roadID_current,itr->second[i][0]), return_temp));
                                    }
                                    else
                                    {
                                        get<0>(return_temp) = 0;

                                        get<1>(return_temp).clear();
                                        if (get<1>(return_temp).find(time_current + te_current) == get<1>(return_temp).end())
                                            get<1>(return_temp).insert(pair<int, vector<vector<int>>>((time_current + te_current), {{routeID_current, 1, 0}}));
                                        else
                                            get<1>(return_temp)[time_current + te_current].push_back({routeID_current, 1, 0});

                                        get<2>(return_temp).clear();
                                        get<2>(return_temp).push_back(routeID_current);

                                        roadSegmentAffected.push_back(make_pair(make_pair(next_roadID_current,itr->second[i][0]), return_temp));
                                    }
                                }
                            }
                            else
                            {
                                // Update traffic flow
                                traffic_flow_origin += 1;
                                traffic_flow_insertion += 1;

                                /* Note: Although same range, since traffic flow change, flow number still need to
                                 * update in the RR-Index. Such as: itr->second[i][2] = traffic_flow_insertion; */
                                itr->second[i][2] = traffic_flow_insertion;
                            }
                        }
                        else
                        {
                            // Update traffic flow
                            traffic_flow_origin += 1;
                            traffic_flow_insertion += 1;
                        }
                    }
                    else
                    {
                        // Active terminal condition, no further route records are affected
                        if (insert_pre.size() == 0 and deletion_pre.size() == 0)
                        {
                            if (terminal == true  and insert_list.size() == 0 and traffic_flow_insertion == 0)
                            {
                                bBreak2 = true;
                                break;
                            }
                        }

                        // If "insert_pre"and "deletion_pre" still have value, iteration until them are empty
                        traffic_flow_origin += 1;
                        traffic_flow_insertion += 1;

                        itr->second[i][2] = traffic_flow_insertion;
                    }
                }
                // Step 4.1.5 If terminal condition is not activated
                // -----------------------------------------------------------------------------
                else
                {
                    // Update traffic flow
                    traffic_flow_origin += 1;
                    traffic_flow_insertion += 1;

                    if (traffic_flow_origin == traffic_flow_insertion)
                    {
                        // Active terminal condition
                        terminal_condition = 1;

                        // Update traffic flow to RR-Index
                        itr->second[i][2] = traffic_flow_insertion;

                        // Estimate record time (determine if no further affect)
                        if (catching)
                        {
                            if (range)
                            {
                                /* This version is preferred but need further develop */
                                cout << "This version is preferred but need further develop" << endl;
                            }
                            else
                            {
                                /* This version is preferred but need further develop */
                                cout << "This version is preferred but need further develop" << endl;
                            }
                        }
                        else
                        {
                            if (range)
                            {
                                /* This version is preferred but need further develop */
                                cout << "This version is preferred but need further develop" << endl;
                            }
                            else
                            {
                                te_current = tm * (1 + sigma * pow(traffic_flow_insertion/varphi, beta));
                            }
                        }

                        /* Note: Because traffic flow same, RR-Index do not need to update travel time.
                         * Estimation here is to define record time */
                        record_time = time_current + te_current;
                    }
                    else
                    {
                        // Update traffic flow to RR-Index
                        itr->second[i][2] = traffic_flow_insertion;

                        routeID_current =  itr->second[i][0];

                        // Estimate new travel time as traffic flow change
                        if (catching)
                        {
                            if (range)
                            {
                                /* This version is preferred but need further develop */
                                cout << "This version is preferred but need further develop" << endl;
                            }
                            else
                            {
                                /* This version is preferred but need further develop */
                                cout << "This version is preferred but need further develop" << endl;
                            }
                        }
                        else
                        {
                            if (range)
                            {
                                /* This version is preferred but need further develop */
                                cout << "This version is preferred but need further develop" << endl;
                            }
                            else
                            {
                                te_current = tm * (1 + sigma * pow(traffic_flow_insertion/varphi, beta));
                            }
                        }

                        // Add corresponding route record with leaving status to "insert_list"
                        /* As its travel time are affected, we should delete the original one and insert a new one */
                        if (insert_list.find(time_current + te_current) == insert_list.end())
                            insert_list.insert(pair<int, vector<vector<int>>>((time_current + te_current), {{routeID_current, 0, 0}}));
                        else
                            insert_list[time_current + te_current].push_back({routeID_current, 0, 0});

                        // Add original route record to "deletion_list"
                        deletion_list.push_back(routeID_current);

                        // Debug
                        if (itr->second[i][0] == 192603)
                            cout << "Here. Route ID is 192603." << endl;
                        // if (itr->second[i][0] == 181 and roadID == 6344){
                        if (roadID == 6344 and index == 0){
                            map<int, vector<vector<int>>>::iterator itr;
                            for (itr = RRIndex_roadID_slice_insertion[6344][0].begin(); itr != RRIndex_roadID_slice_insertion[6344][0].end(); ++itr){
                                cout << "time: " << itr->first;
                                for (int j = 0; j < itr->second.size(); j++){
                                    cout << " route id: " << itr->second[j][0];
                                    cout << " status: " << itr->second[j][1];
                                    cout << " flow: " << itr->second[j][2];
                                }
                            }
                            cout << "??? Why this happen. Route ID is 181. " << "\n" << endl;
                        }

                        int next_roadID_current;
                        if (itr->second[i][0] == new_route_ID)
                        {
                            next_roadID_current = next_roadID;
                            cout << "Check. Is this code necessary? Why this situation can happen?" << endl;
                        }
                        else
                        {
                            vector<int> affected_route_node = route_node_new[itr->second[i][0]];
                            vector<int> affected_route_road = single_route_node_2_route_road(affected_route_node);
                            /* next_roadID_current = find_next_roadID(affected_route_road, roadID); */
                            // Debug
                            next_roadID_current = find_next_roadID(affected_route_road, roadID, "J_rest", itr->second[i][0]);

                        }

                        // Check if next affected road belongs to the newly inserted route
                        std::vector<int>::iterator itrFindNew;
                        itrFindNew = std::find (new_route_road.begin(), new_route_road.end(), next_roadID_current);

                        // Find IF Next Road Belongs to Affected Road
                        affected_road_ID.clear();
                        for (int k=0;k<roadSegmentAffected.size();k++)
                        {
                            int roadID_temp = roadSegmentAffected[k].first.first;
                            affected_road_ID.push_back(roadID_temp);
                        }

                        std::vector<int>::iterator itFindExit;
                        itFindExit = std::find(affected_road_ID.begin(), affected_road_ID.end(), next_roadID_current);

                        if (itFindExit != affected_road_ID.end())
                        {
                            int position = itFindExit - affected_road_ID.begin();
                            return_temp = roadSegmentAffected[position].second;

                            if (get<1>(return_temp).find(time_current + te_current) == get<1>(return_temp).end())
                                get<1>(return_temp).insert(pair<int, vector<vector<int>>>((time_current + te_current), {{routeID_current, 1, 0}}));
                            else
                                get<1>(return_temp)[time_current + te_current].push_back({routeID_current, 1, 0});

                            get<2>(return_temp).push_back(routeID_current);

                            // If the next affected road belongs to new route
                            if (itrFindNew != new_route_road.end())
                            {
                                get<0>(return_temp) = time_new_leave;
                                roadSegmentAffected[position].second = return_temp;
                            }
                            else
                            {
                                /* "0" refers to the next road are not contained by the new route */
                                get<0>(return_temp) = 0;
                                roadSegmentAffected[position].second = return_temp;
                            }
                        }
                        else
                        {
                            // If the next affected road belongs to new route
                            if (itrFindNew != new_route_road.end())
                            {
                                get<0>(return_temp) = time_new_leave;

                                get<1>(return_temp).clear();
                                if (get<1>(return_temp).find(time_current + te_current) == get<1>(return_temp).end())
                                {
                                    get<1>(return_temp).insert(pair<int, vector<vector<int>>>((time_current + te_current),{{routeID_current, 1, 0}}));
                                }
                                else
                                {
                                    get<1>(return_temp)[time_current + te_current].push_back({routeID_current, 1, 0});
                                }

                                get<2>(return_temp).clear();
                                get<2>(return_temp).push_back(routeID_current);

                                roadSegmentAffected.push_back(make_pair(make_pair(next_roadID_current,itr->second[i][0]), return_temp));
                            }
                            else
                            {
                                get<0>(return_temp) = 0;

                                get<1>(return_temp).clear();
                                if (get<1>(return_temp).find(time_current + te_current) == get<1>(return_temp).end())
                                    get<1>(return_temp).insert(pair<int, vector<vector<int>>>((time_current + te_current), {{routeID_current, 1, 0}}));
                                else
                                    get<1>(return_temp)[time_current + te_current].push_back({routeID_current, 1, 0});

                                get<2>(return_temp).clear();
                                get<2>(return_temp).push_back(routeID_current);

                                roadSegmentAffected.push_back(make_pair(make_pair(next_roadID_current,itr->second[i][0]), return_temp));
                            }
                        }
                    }
                }
            }
            // Step 4.3 If route record is leaving status
            // -----------------------------------------------------------------------------
            else
            {
                // ??? why these statment? what is diff between inTime=0 or not
                if (deletion_pre.size() != 0 or inTime != 0 or deletion_list.size() != 0)
                {
                    // Compare route records with "deletion_list"
                    /* Because in the first road of new route, only affected original travel time should be deleted and
                     * insert the updated one. We only consider "deletion_list" here */
                    if (deletion_list.size() != 0)
                    {
                        int deleID;

                        /* If current routeID does not match any routeID in "deletion_list",
                         * update traffic flow and iterate to the next route record */
                        int deletion_list_size = deletion_list.size();

                        for (int l = 0; l < deletion_list.size(); l++)
                        {
                            deleID = deletion_list[l];

                            if (routeID_current == deleID)
                            {
                                traffic_flow_origin -= 1;

                                // Delete route record
                                itr->second.erase(itr->second.begin() + i);

                                // Update route record size as deletion
                                dynamicSize = itr->second.size();
                                // Back to previous route record
                                /* Will locate to the next route record in the next iteration */
                                i--;

                                // Delete this route record in the "deletion_list"
                                vector<int>::iterator k = deletion_list.begin() + l;
                                deletion_list.erase(k);

                                /* If route records under this time (key) are empty after deletion, this key-value pair
                                 * should be deleted */
                                if(itr->second.size() == 0)
                                {
                                    itr = RRIndex_roadID_slice_insertion[roadID][index].erase(itr);

                                    if (RRIndex_roadID_slice_insertion[roadID][index].size() == 0)
                                    {
                                        /* If all times (key) under this time slice is empty after deletion,
                                         * it is unnecessary to iterate route record under this time slice */
                                        bBreak2 = true;
                                        break;
                                    }

                                    /* After deletion, the current time (key) is empty. It is unnecessary to
                                     * iterate the rest route record and continues to the next time (key) */
                                    bBreak = true;
                                    break;
                                }

                                /* After deletion, the current route record is deleted. It is unnecessary to
                                 * iterate the "deletion_list" to decide deletion */
                                break;
                            }
                        }

                        // ??? Not sure if we should update traffic flow here
                        // ??? If no route ID should be deleted in the "deletion_list", we may need to update traffic flow
                        if (deletion_list_size == deletion_list.size())
                        {
                            // Update traffic flow
                            traffic_flow_insertion -= 1;
                            traffic_flow_origin -= 1;

                            // Update in RR-Index
                            itr->second[i][2] = traffic_flow_insertion;
                        }
                    }
                    else
                    {
                        // Update traffic flow
                        traffic_flow_insertion -= 1;
                        traffic_flow_origin -= 1;

                        // Update in RR-Index
                        itr->second[i][2] = traffic_flow_insertion;
                    }
                }
            }
        }
        if(bBreak)
            continue;
        if(bBreak2)
            break;
        dynamicSize = itr->second.size();
        ++itr;
    }

    // Step 5. Insert rest values contained in "insert_pre"
    // -----------------------------------------------------------------------------
    /* If updated entrance time from the previous road and delayed travel time is later than all times (key) in the
     * current road, above iteration cannot insert them successfully. We should insert them here */

    int RouteIDInPre;
    map<int, vector<vector<int>>>::iterator itrInPreRest;

    if (insert_pre.size() != 0)
    {
        for (itrInPreRest = insert_pre.begin(); itrInPreRest != insert_pre.end(); ++itrInPreRest)
        {
            int timeIPre = itrInPreRest->first;

            for (int i = 0; i < itrInPreRest->second.size(); i++)
            {
                /* For each time (key) in the "insert_pre", iterate all route records in "insert_list" */
                int RouteIDI;
                map<int, vector<vector<int>>>::iterator itrI2;

                for (itrI2 = insert_list.begin(); itrI2 != insert_list.end();)
                {
                    int timeI2 = itrI2->first;

                    if (timeI2 < timeIPre)
                    {
                        for (int j = 0; j < itrI2->second.size(); j++)
                        {
                            // Compare Each Time Record in Insert Set.
                            traffic_flow_insertion += 1;

                            RouteIDI = itrI2->second[j][0];

                            // Update RR-Index
                            if (itrI2->second[j][1] == 1)
                            {
                                cout << "Error. Element in 'insert_list' is not leaving status." << endl;
                            }
                            else
                            {
                                if (RRIndex_roadID_slice_insertion[roadID][index].find(timeI2) == RRIndex_roadID_slice_insertion[roadID][index].end())
                                    RRIndex_roadID_slice_insertion[roadID][index].insert(pair<int, vector<vector<int>>>(timeI2, {{RouteIDI, 0, traffic_flow_insertion}}));
                                else
                                    RRIndex_roadID_slice_insertion[roadID][index][timeI2].push_back({RouteIDI, 0, traffic_flow_insertion});
                            }
                        }

                        itrI2 = insert_list.erase(itrI2);
                    }
                    else
                    {
                        // Iterate to the next time (key)
                        ++itrI2;
                    }
                }

                traffic_flow_insertion += 1;

                RouteIDInPre = itrInPreRest->second[i][0];

                if (RRIndex_roadID_slice_insertion[roadID][index].find(timeIPre) == RRIndex_roadID_slice_insertion[roadID][index].end())
                    RRIndex_roadID_slice_insertion[roadID][index].insert(pair<int, vector<vector<int>>>(timeIPre, {{RouteIDInPre,1,traffic_flow_insertion}}));
                else
                    RRIndex_roadID_slice_insertion[roadID][index][timeIPre].push_back({RouteIDInPre,1,traffic_flow_insertion});

                time_current = itrInPreRest->first;
                routeID_current = itrInPreRest->second[i][0];

                // Estimate new travel time as traffic flow change
                if (catching)
                {
                    if (range)
                    {
                        /* This version is preferred but need further develop */
                        cout << "This version is preferred but need further develop" << endl;
                    }
                    else
                    {
                        /* This version is preferred but need further develop */
                        cout << "This version is preferred but need further develop" << endl;
                    }
                }
                else
                {
                    if (range)
                    {
                        /* This version is preferred but need further develop */
                        cout << "This version is preferred but need further develop" << endl;
                    }
                    else
                    {
                        te_current = tm * (1 + sigma * pow(traffic_flow_insertion/varphi, beta));
                    }
                }

                // Update the new travel time in "insert_list"
                if (insert_list.find(time_current + te_current) == insert_list.end())
                    insert_list.insert(pair<int, vector<vector<int>>>((time_current + te_current), {{routeID_current, 0, 0}}));
                else
                    insert_list[time_current + te_current].push_back({routeID_current, 0, 0});

                // Add the affected route ID into "deletion_list"
                deletion_list.push_back(routeID_current);

                // Update return value
                std::vector<int>::iterator itrFindNew;
                itrFindNew = std::find(new_route_road.begin(), new_route_road.end(), next_roadID);

                // If next road has already added in the return value
                affected_road_ID.clear();
                for (int j = 0; j < roadSegmentAffected.size(); j++)
                {
                    int roadID_temp = roadSegmentAffected[j].first.first;
                    affected_road_ID.push_back(roadID_temp);
                }

                std::vector<int>::iterator itFindExit;
                itFindExit = std::find(affected_road_ID.begin(), affected_road_ID.end(), next_roadID);

                // If next road has already added
                if (itFindExit != affected_road_ID.end())
                {
                    int position = itFindExit - affected_road_ID.begin();
                    return_temp = roadSegmentAffected[position].second;

                    if (get<1>(return_temp).find(time_current + te_current) == get<1>(return_temp).end())
                        get<1>(return_temp).insert(pair<int, vector<vector<int>>>((time_current + te_current), {{routeID_current, 1, 0}}));
                    else
                        get<1>(return_temp)[time_current + te_current].push_back({routeID_current, 1, 0});

                    get<2>(return_temp).push_back(routeID_current);

                    // Decide if next road belongs to the new route
                    if (itrFindNew != new_route_road.end())
                    {
                        get<0>(return_temp) = time_new_leave;
                        roadSegmentAffected[position].second = return_temp;
                    }
                    else
                    {
                        get<0>(return_temp) = 0;
                        roadSegmentAffected[position].second = return_temp;
                    }
                }
                else
                {
                    if (itrFindNew != new_route_road.end())
                    {
                        get<0>(return_temp) = time_new_leave;

                        get<1>(return_temp).clear();

                        if (get<1>(return_temp).find(time_current + te_current) == get<1>(return_temp).end())
                            get<1>(return_temp).insert(pair<int, vector<vector<int>>>((time_current + te_current), {{routeID_current, 1, 0}}));
                        else
                            get<1>(return_temp)[time_current + te_current].push_back({routeID_current, 1, 0});

                        get<2>(return_temp).clear();
                        get<2>(return_temp).push_back(routeID_current);

                        roadSegmentAffected.push_back(make_pair(make_pair(next_roadID,itrInPreRest->second[i][0]), return_temp));
                    }
                    else
                    {
                        get<0>(return_temp) = 0;
                        get<1>(return_temp).clear();

                        if (get<1>(return_temp).find(time_current + te_current) == get<1>(return_temp).end())
                            get<1>(return_temp).insert(pair<int, vector<vector<int>>>((time_current + te_current), {{routeID_current, 1, 0}}));
                        else
                            get<1>(return_temp)[time_current + te_current].push_back({routeID_current, 1, 0});

                        get<2>(return_temp).clear();
                        get<2>(return_temp).push_back(routeID_current);

                        roadSegmentAffected.push_back(make_pair(make_pair(next_roadID,itrInPreRest->second[i][0]), return_temp));
                    }
                }
            }
        }
    }

    // Step 6. delete rest values contained in "deletion_list"
    // -----------------------------------------------------------------------------
    if (deletion_list.size() != 0)
    {
        // Update traffic flow
        traffic_flow_origin -= 1;

        map<int, vector<vector<int>>>::iterator itrend;
        // ??? Why deletion here? Why iteration cannot delete all information
        // ??? Why only consider the last time (key)
        itrend = --RRIndex_roadID_slice_insertion[roadID][index].end();

        for (int i = 0; i < itrend->second.size();)
        {
            if (deletion_list.size() == 0)
                break;

            // Define Route ID
            routeID_current = itrend->second[i][0];
            // Check If Is Target Route and Leaving Status
            if (itrend->second[i][1] == 0 and routeID_current == deletion_list[0]){
                itrend->second.erase(itrend->second.begin() + i);
                if (deletion_list.size() == 0){
                    cout << "deletion_list size is 0 " << endl;
                }
                vector<int>::iterator k = deletion_list.begin();
                deletion_list.erase(k);
                if(itrend->second.size() == 0){
                    RRIndex_roadID_slice_insertion[roadID][index].erase(itrend);
                    break;
                }
            }
            else{
                i++;
                continue;
            }
        }
    }

    // Step 7. insert rest values contained in "insert_list"
    // -----------------------------------------------------------------------------
    int FlowEnd;
    map<int, vector<vector<int>>>::iterator itrEnd;

    itrEnd = --RRIndex_roadID_slice_insertion[roadID][index].end();
    FlowEnd = itrEnd->second.back()[2];

    map<int, vector<vector<int>>>::iterator itrRestI;
    int timeRest, RouteIDRest, FlowRest;

    if (insert_list.size() != 0)
    {
        for (itrRestI = insert_list.begin(); itrRestI != insert_list.end() ;++itrRestI)
        {
            for (int i = 0; i < itrRestI->second.size(); i++)
            {
                timeRest = itrRestI->first; RouteIDRest = itrRestI->second[i][0];
                FlowRest = FlowEnd - 1;;
                FlowEnd -= 1;

                if (RRIndex_roadID_slice_insertion[roadID][index].find(timeRest) == RRIndex_roadID_slice_insertion[roadID][index].end())
                    RRIndex_roadID_slice_insertion[roadID][index].insert(pair<int, vector<vector<int>>>(timeRest, {{RouteIDRest, 0, FlowRest}}));
                else
                    RRIndex_roadID_slice_insertion[roadID][index][timeRest].push_back({RouteIDRest, 0, FlowRest});
            }
        }
    }

    // Step 8. Print result
    // -----------------------------------------------------------------------------

    if (print == true)
    {
        cout << endl;

        cout << "Step 3: time records after manipulation" << endl;
        cout << "-----------------------------------------" << endl;

        map<int, vector<vector<int>>>::iterator itrPrint;
        cout << "TimeRecord Size is: " << RRIndex_roadID_slice_insertion[roadID][index].size() << endl;
        cout << "roadID: " << roadID << " with TimeSlice: " << index << endl;

        for (itrPrint = RRIndex_roadID_slice_insertion[roadID][index].begin(); itrPrint != RRIndex_roadID_slice_insertion[roadID][index].end(); ++itrPrint)
        {
            for (int i=0;i<itrPrint->second.size();i++)
            {
                cout << " time " << itrPrint->first << " routeID " << itrPrint->second[i][0];
                cout << " status " << itrPrint->second[i][1] << " flow " << itrPrint->second[i][2] << "||";
            }
        }
        cout << endl;
    }

    // Step 9. Add next road in the newly inserted route to the return value
    // -----------------------------------------------------------------------------

    /* Until now, the return value contains affected further road IDs with their affected route records.
     * If these road IDs do not contain the next road ID in the newly insert route, we add it to the return value */

    std::vector<int>::iterator itr_findNextExist;

    affected_road_ID.clear();
    for (int i = 0; i < roadSegmentAffected.size(); i++)
    {
        affected_road_ID.push_back(roadSegmentAffected[i].first.first);
    }

    itr_findNextExist = std::find(affected_road_ID.begin(), affected_road_ID.end(), next_roadID);

    tuple<int, map<int, vector<vector<int>>>, vector<int>> fillReturn;

    if (next_roadID != -1 and itr_findNextExist == affected_road_ID.end())
    {
        get<0>(fillReturn) = time_new_leave;
        get<1>(fillReturn).clear();
        get<2>(fillReturn).clear();

        if (inTime == 0)
            roadSegmentAffected.push_back(make_pair(make_pair(next_roadID,route_prop.first),fillReturn));
        else
            roadSegmentAffected.push_back(make_pair(make_pair(next_roadID,new_route_pair.first),fillReturn));
    }

    return roadSegmentAffected;
}