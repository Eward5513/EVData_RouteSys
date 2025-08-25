#include "graph.h"

// Generate new data for insertion operation and store to local file
/* Existing route data with different departure time */
void Graph::data_generation_4_insertion(
        vector<vector<int>>& routes, vector<vector<int>>& query,
        const string insert_route_node_path, const string insert_route_depart_time_path,
        const string insert_route_road_path, int new_num)
{
    // 1. Correctness Check
    // -----------------------------------------------------------------------------
    /* !!! Note: Because the idea is to assign different departure time to same route data,
     * the size of generated new route data should smaller than the size of route data. */
    if (new_num > routes.size())
    {
        cout << "Error. The number of new route size " << new_num;
        cout << " should smaller than the size of route data " << routes.size();
        cout << " Please check the data generation logic to revise." << endl;
    }

    // 2. New route data generation
    // -----------------------------------------------------------------------------
    vector<int> random_num = randperm(routes.size());       // Randomly generate unordered integer

    vector<vector<int>> route_node_temp;

    // 2.1 Capture node ID pairs constructed route data
    for (int i = 0; i < new_num; i++)
    {
        int route_index = random_num[i];
        route_node_temp.push_back(routes[route_index]);
    }

    // 2.2 Capture road IDs constructed route data
    vector<vector<int>> route_road_temp;

    for (int i = 0; i < new_num; i++)
    {
        route_road_temp.push_back(single_route_node_2_route_road(route_node_temp[i]));
    }

    // 3. Write data out
    // -----------------------------------------------------------------------------

    // 3.1 Write captured route data (node ID pairs) Out
    // -------------------------------------------------------
    ofstream outRoute;
    outRoute.open(insert_route_node_path);

    for (int i = 0; i < route_node_temp.size(); i++)
    {
        outRoute << route_node_temp[i].size() << " ";

        for (int j = 0; j < route_node_temp[i].size(); j++)
        {
            outRoute << route_node_temp[i][j] << " ";
        }

        outRoute << endl;
    }

    outRoute.close();

    // 3.2 Write captured route data out
    // -------------------------------------------------------
    ofstream outRouteRoad;
    outRouteRoad.open(insert_route_road_path);

    for (int i = 0; i < route_road_temp.size(); i++)
    {
        outRouteRoad << route_road_temp[i].size() << " ";

        for (int j = 0; j < route_road_temp[i].size(); j++)
        {
            outRouteRoad << route_road_temp[i][j] << " ";
        }

        outRouteRoad << endl;
    }

    outRouteRoad.close();

    // 3.3 Write randomly generated departure time out
    // -------------------------------------------------------
    ofstream outDepar;
    outDepar.open(insert_route_depart_time_path);

    vector<int> depar_time(new_num);

    int time, time_generated, time_range_right;

    for (int i = 0; i < depar_time.size(); i++)
    {
        time = query[i][2];

        /* New data's departure time is ranged in two minutes */
        time_range_right = time + 2*60;
        time_generated = (rand() % (time_range_right - time + 1)) + time;

        outDepar << time_generated << " " << endl;
    }

    outDepar.close();

    // 4. Print operation procedure
    // -----------------------------------------------------------------------------
    cout << "New inserted data generation done." << "\n" << endl;

}


// Read new route data constructed by road IDs
/* Note: returned variable only contains inserted route data. */
vector<pair<int, vector<int>>> Graph::read_generated_new_route_road(const string insert_route_road_path,
                                                         vector<vector<int>>& routeDataRaw, int new_num)
{
    // 1. Open file
    // -----------------------------------------------------------------------------
    ifstream IFRoute(insert_route_road_path);
    if(!IFRoute)
        cout << "Cannot open route (road) file: "<< insert_route_road_path << endl;

    // 2. Variable initialization
    // -----------------------------------------------------------------------------
    int road_num;
    vector<vector<int>> route_road_new;
    vector<int> route_road_new_temp;

    // 3. Read route constructed by road IDs
    // -----------------------------------------------------------------------------
    while(IFRoute >> road_num)
    {
        route_road_new_temp.clear();
        route_road_new_temp.resize(road_num);

        for (int j = 0; j < road_num; j++)
        {
            IFRoute >> route_road_new_temp[j];
        }

        route_road_new.push_back(route_road_new_temp);
    }

    // 4. Add index (ID) for generated new routes as input for update operation
    // -----------------------------------------------------------------------------
    route_road_new_with_index.reserve(new_num);

    int original_route_num = routeDataRaw.size();

    for (int i = 0; i < route_road_new.size(); i++)
    {
        int current_route_index = i + original_route_num;

        /* To query its index in all route data, we record its index here, and the passing routes only contain
         * the newly generated ones. */
        route_road_new_with_index.push_back(make_pair(current_route_index, route_road_new[i]));
    }

    // 5. Print
    // -----------------------------------------------------------------------------
    /* for (int i = 0; i < route_road_new_with_index.size(); i++)
    {
        int accumulate_index = route_road_new_with_index[i].first;

        cout << "New route by road IDs" << i << " with accumulated index " << accumulate_index << endl;

        for (int j = 0; j < route_road_new_with_index[i].second.size(); j++)
        {
            cout << route_road_new_with_index[i].second[j] << " ";
        }
    } */

    cout << "The number of generated new route data is： " << route_road_new_with_index.size() << endl;
    cout << "Read inserted route data constructed by road IDs done." << "\n" << endl;

    return route_road_new_with_index;
}


// Read new route data constructed by node IDs
/* Note: returned variable contains both original and inserted route data.
 * Inserted route data is accumulated on the original. */
vector<vector<int>> Graph::read_generated_new_route_node(const string insert_route_node_path,
                                                         vector<vector<int>>& routeDataRaw, int new_num)
{
    // 1. Open file
    // -----------------------------------------------------------------------------
    ifstream IFRouteNode(insert_route_node_path);
    if(!IFRouteNode)
        cout << "Cannot open route constructed by nodes file: "<< insert_route_node_path << endl;

    // 2. Variable initialization
    // -----------------------------------------------------------------------------
    route_node_new = routeDataRaw;
    route_node_new.reserve(routeDataRaw.size() + new_num);

    // 3. Read route data constructed by node IDs
    // -----------------------------------------------------------------------------
    int route_node_num = 0;
    vector<int> route_node_new_temp;

    while(IFRouteNode >> route_node_num)
    {
        route_node_new_temp.clear();
        route_node_new_temp.resize(route_node_num);

        for (int j = 0; j < route_node_num; j++)
        {
            IFRouteNode >> route_node_new_temp[j];
        }

        route_node_new.push_back(route_node_new_temp);
    }

    // 4. Print
    // -----------------------------------------------------------------------------
    /* for (int i = 0; i < route_node_new.size(); i++)
    {
        cout << "New route by node IDs " << i << endl;

        for (int j = 0; j < route_node_new[i].size(); j++)
        {
            cout << route_node_new[i][j] << " ";
        }

        cout << endl;
    } */

    cout << "The number of total route data is: " << route_node_new.size() << endl;
    cout << "Read inserted route data constructed by node IDs done." << "\n" << endl;

    return route_node_new;
}


// Read new route data constructed by node IDs
vector<int> Graph::read_generated_new_route_departure_time(const string insert_route_depart_time_path)
{
    // 1. Open file
    // -----------------------------------------------------------------------------
    ifstream IFDepar(insert_route_depart_time_path);
    if(!IFDepar)
        cout << "Cannot open departure time file: "<< insert_route_depart_time_path << endl;

    // 2. Variable Initialization
    // -----------------------------------------------------------------------------
    int departure_time;
    int depart_length = CountLines(insert_route_depart_time_path);
    departure_time_new.reserve(depart_length);

    for (int i = 0; i < depart_length; i++)
    {
        IFDepar >> departure_time;

        departure_time_new.push_back(departure_time);
    }

    // 3. Print
    // -----------------------------------------------------------------------------
    /* for (int i = 0; i < departure_time_new.size(); i++)
    {
      cout << "Route " << i << "'s departure time is: " << departure_time_new[i] << endl;
    } */

    cout << "New routes contains " << departure_time_new.size() << " departure time." << endl;
    cout << "Read new routes' departure time done." << endl;

    return departure_time_new;
}


// Initialize new temporal information after adding new routes
vector<vector<pair<int, float>>> Graph::inserted_temporal_information_initialization(
        vector<vector<pair<int, float>>>& temporal_result)
{
    // 1. Re-Initialize "temporal_result_insertion"
    // -----------------------------------------------------------------------------
    /* route_node_new contains the original route and newly inserted route data
     * route_road_new_with_index only contains the newly inserted route data, but it also contains the index of new
     * data accumulated on the original data. */
    temporal_result_insertion.resize(route_node_new.size());

    for (int i = 0; i < route_node_new.size(); i++)
    {
        temporal_result_insertion[i].resize(route_node_new[i].size());
    }

    // Assign original simulated results
    for (int i = 0; i < temporal_result.size(); i++)
    {
        for (int j = 0; j < temporal_result[i].size(); j++)
        {
            temporal_result_insertion[i][j].first = temporal_result[i][j].first;
            temporal_result_insertion[i][j].second = temporal_result[i][j].second;
        }
    }
    for (int i = temporal_result.size(); i < temporal_result_insertion.size(); i++)
    {
        for (int j = 0; j < 1; j++)
        {
            temporal_result_insertion[i][j].first = route_node_new[i][j];
            temporal_result_insertion[i][j].second = departure_time_new[i - temporal_result.size()];
        }
        for (int k = 1; k < temporal_result_insertion[i].size(); k++)
        {
            temporal_result_insertion[i][k].first = route_node_new[i][k];
            temporal_result_insertion[i][k].second = INF;
        }
    }

    // 2. Print
    // -----------------------------------------------------------------------------
    /* for (int i = 0; i < temporal_result_insertion.size(); i++)
    {
        cout << "route " << i << ": ";

        for (int j = 0; j < temporal_result_insertion[i].size(); j++)
        {
            cout << "node " << temporal_result_insertion[i][j].first << " with ";
            cout << "time " << temporal_result_insertion[i][j].second << " ";
        }
        cout << endl;
    } */

    cout << "Temporal information initialization after inserting new routes done." << "\n" << endl;

    return temporal_result_insertion;
}