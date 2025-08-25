#include "graph.h"


// Convert node pairs constructed RR-Index to road ID
void Graph::convert_node_pairs_index_2_roadID(vector<vector<pair<int, map<int, vector<vector<int>>>>>>& RRIndex)
{
    /* RR-Index after macroscopic simulation is lead by node paris, which is not convenient for further update.
     * Here, we convert it to road ID constructed version that route records are stored by road ID. */
    // 1. Variable initialization
    // -----------------------------------------------------------------------------
    map<int, vector<vector<int>>> placeholder;
    RRIndex_roadID.assign(roadID2NodeID.size(), placeholder);

    int node_ID1 = 0;
    int node_ID2 = 0;
    int route_ID = 0;

    for (int i = 0; i < RRIndex.size(); i++)
    {
        node_ID1 = i;

        for (int j = 0; j < RRIndex[i].size(); j++)
        {
            node_ID2 = RRIndex[i][j].first;
            route_ID = nodeID2RoadID[make_pair(node_ID1, node_ID2)];

            map<int, vector<vector<int>>> route_records = RRIndex[i][j].second;
            RRIndex_roadID[route_ID] = route_records;
        }
    }

    // 2. Print
    // -----------------------------------------------------------------------------
	/* for (int i = 0; i < RRIndex_roadID.size(); i++)
    {
	    cout << "Road ID: " << i << " with size " << RRIndex_roadID[i].size() << endl;

	    map<float, vector<vector<int>>>::iterator itr;

	    for (itr = RRIndex_roadID[i].begin(); itr != RRIndex_roadID[i].end(); ++itr)
        {
	        cout << " time " << itr->first;

	        for (int j = 0; j < itr->second.size(); j++)
            {
	            cout << " routeID " << itr->second[j][0] << " status ";
                cout << itr->second[j][1] << " flow " << itr->second[j][2] << "||";
	        }
	    }

	    cout << "\n" << endl;
	} */

    cout << "RR-Index convert node pairs to road ID done." << "\n" << endl;
}


// Check correctness of route records in RR-Index
void Graph::RRIndex_correct_check()
{
    /* Make sure the simulated route records follows the following constrains:
     * 1. Under each road, traffic flow of the first route record should be 1,
     * 2. Under each road, traffic flow of the last route record should be 0,
     * 3. Under each road, traffic flow of route records should increase or decrease one each time. */

    // 1. Check traffic flow of first and last route records
    // -----------------------------------------------------------------------------
    for (int i = 0; i < RRIndex_roadID.size(); i++)
    {
        if (RRIndex_roadID[i].size() != 0)
        {
            // 1.1 Check if traffic flow of the first route record is 1
            // -----------------------------------------------------------------------------
            map<int, vector<vector<int>>>::iterator itrBegin;
            itrBegin = RRIndex_roadID[i].begin();   // Point to first time (key)

            int traffic_flow_first = itrBegin->second[0][2];    // first time record (index: 0)

            if (traffic_flow_first != 1)
            {
                // Delete all the route records under this road ID
                RRIndex_roadID[i].clear();

                cout << "Error. Traffic flow of the first time record is not 1." << endl;

                continue;
            }

            // 1.2 Check if traffic flow of the last route record is 0
            // -----------------------------------------------------------------------------
            map<int, vector<vector<int>>>::iterator itrEnd;
            itrEnd = --RRIndex_roadID[i].end();     // Point to last time (key)

            int traffic_flow_last = itrEnd->second.back()[2];   // last time record
            if (traffic_flow_last != 0)
            {
                cout << "road id: " << i << endl;
                for (int a = 0; a < itrEnd->second.size(); a++){
                    cout << "route id: " << itrEnd->second[a][0] << " statue: " << itrEnd->second[a][1];
                    cout << " flow: " << itrEnd->second[a][2] << endl;
                }

                // Delete all the time records under this road ID
                RRIndex_roadID[i].clear();

                cout << "Error. Traffic flow of the last time record is not 0. " << traffic_flow_last << endl;

                continue;
            }

            // 2. Check traffic flow of route records are increase or decrease by one
            // -----------------------------------------------------------------------------
            map<int, vector<vector<int>>>::iterator itrCurrent;

            for (itrCurrent = RRIndex_roadID[i].begin(); itrCurrent != --RRIndex_roadID[i].end(); ++itrCurrent)
            {
                // 2.1 Check special situation: only contain one route record (value) under current time (key)
                // -----------------------------------------------------------------------------
                if (itrCurrent->second.size() == 1)
                {
                    /* traffic flow of the only route record */
                    int traffic_flow_current = itrCurrent->second[0][2];

                    ++itrCurrent;   // Point to the next time (key)
                    int traffic_flow_next = itrCurrent->second[0][2];

                    int FlowChange = abs(traffic_flow_current - traffic_flow_next);
                    --itrCurrent; // Point back to the current time (key)

                    /* Although */
                    /* E.g. a route enter and leave a road under different time, these two route records are under
                     * different time (key), but the difference of traffic flow should be one. */
                    if (FlowChange != 1)
                    {
                        // Delete all the route records under this road ID
                        RRIndex_roadID[i].clear();

                        cout << "Error. Traffic flow difference is not one." << endl;

                        continue;
                    }
                }
                else
                {
                    // 2.2 Check route records under the current time (key)
                    // -----------------------------------------------------------------------------
                    for (int j = 0; j < itrCurrent->second.size() - 1; j++)
                    {
                        int flow_current = itrCurrent->second[j][2];
                        int flow_next = itrCurrent->second[j + 1][2];

                        int flow_difference = abs(flow_current - flow_next);

                        if (flow_difference != 1)
                        {
                            // Delete all the route records under this road ID
                            RRIndex_roadID[i].clear();

                            cout << "Error. Traffic flow difference is not one." << endl;

                            continue;
                        }
                    }

                    // 2.3 Check traffic flow of last route record with the first route record of the next time (key)
                    // -----------------------------------------------------------------------------
                    int flow_current = itrCurrent->second.back()[2];

                    ++itrCurrent;   // Point to the next time (key)
                    int flow_next = itrCurrent->second[0][2];

                    int flow_difference = abs(flow_current - flow_next);
                    --itrCurrent;   // Point back to the current time (key)

                    if (flow_difference != 1)
                    {
                        // Delete all the route records under this road ID
                        RRIndex_roadID[i].clear();

                        cout << "Error. Traffic flow difference is not one." << endl;

                        continue;
                    }
                }
            }
            // 2.4 Check route records under the current time (key)
            // -----------------------------------------------------------------------------
            itrCurrent = --RRIndex_roadID[i].end();     // Point to last time (key)

            for (int k = 0; k < itrCurrent->second.size() - 1; k++)
            {
                int flow_current = itrCurrent->second[k][2];
                int flow_next = itrCurrent->second[k + 1][2];

                int flow_difference = abs(flow_current - flow_next);

                if (flow_difference != 1)
                {
                    // Delete all the route records under this road ID
                    RRIndex_roadID[i].clear();

                    cout << "Error. Traffic flow difference is not one." << endl;

                    continue;
                }
            }
        }
    }

    cout << "RR-Index correctness check done." << "\n" << endl;
}


// Filter hour index from an int time
/* This time convert should consider the time zone of current system. */
int Graph::time_2_hour(int int_time)
{
    // 1. Convert time structure
    // -----------------------------------------------------------------------------
    /* time_t is usually used to represent the number of seconds elapsed since a fixed point in time
     * E.g., January 1, 1970, 00:00:00 UTC */
    time_t time = int_time;

    // 2. Extract the hour information from the tm structure
    // -----------------------------------------------------------------------------
    /* Use the localtime function to convert the time_t type time to local time.
     * The localtime function returns a pointer to a tm structure, which contains information
     * such as year, month, day, hour, minute, and second */
    /* this function 'time_2_hour' use the 'localtime' function to returns a tm structure with date information for a
     * timestamp in the computer's local timezone. UTC time zone and Beijing time zone has 8 hours difference.
     * Under this situation, the hour index must be zero, so I directly define it to 0. Under your situation,
     * Please check the correct time and time zone. */
    tm *ltm = gmtime(&time);
    int hour = ltm->tm_hour;

    // 3. Check if the calculated hour is negative
    // -----------------------------------------------------------------------------
    if (hour < 0)
        std::cerr << "Error. Calculated hour is negative: " << hour << std::endl;

    return hour;
}


// Convert hour to its belonged index
/* Note: It only supports 24 hours. */
int Graph::hour_2_index(int hour)
{
    // 1. Estimate the index
    // -----------------------------------------------------------------------------
    /* Special situation: If the simulation time range cross several days, and hour may be smaller than the min_hour. */
    int index = hour - min_hour;

    if (index >= 0)
    {
        return index;
    }
    else
    {
        index += 24;
        return index;
    }
}



// Assign time records into one hour time slice
void Graph::split_RRIndex_2_time_slices(vector<map<int, vector<vector<int>>>>& RRIndex_roadID)
{
    // 1. Variable initialization
    // -----------------------------------------------------------------------------
    map<int, vector<vector<int>>> placeholder1;
    vector<map<int, vector<vector<int>>>> placeholder2;
    placeholder2.assign(24, placeholder1);
    RRIndex_roadID_slice.assign(RRIndex_roadID.size(), placeholder2);

    for (int i = 0; i < RRIndex_roadID.size(); i++)
    {
        int road_ID = i;

        map<int, vector<vector<int>>>::iterator itr;

        for (itr = RRIndex_roadID[i].begin(); itr != RRIndex_roadID[i].end(); ++itr)
        {
            float time_key_float = itr->first;
            int time_key = (int)time_key_float;

            int hour = time_2_hour(time_key);

            int index = hour_2_index(hour);

            for (int j = 0; j < itr->second.size(); j++)
            {
                if (RRIndex_roadID_slice[road_ID][index].find(time_key) == RRIndex_roadID_slice[road_ID][index].end())
                {
                    RRIndex_roadID_slice[road_ID][index].insert(
                            pair<int, vector<vector<int>>>(time_key, {itr->second[j]}));
                }
                else
                {
                    RRIndex_roadID_slice[road_ID][index][time_key].push_back(itr->second[j]);
                }
            }
        }
    }

    cout << "Add hour index to RR-Index done." << "\n" << endl;
}




