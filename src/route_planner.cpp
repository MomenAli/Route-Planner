#include "route_planner.h"
#include <algorithm>




RoutePlanner::RoutePlanner(RouteModel &model, float start_x, float start_y, float end_x, float end_y): m_Model(model) {
    // Convert inputs to percentage:
    start_x *= 0.01;
    start_y *= 0.01;
    end_x *= 0.01;
    end_y *= 0.01;

    // Use the m_Model.FindClosestNode method to find the closest nodes to the starting and ending coordinates.
    // Store the nodes you find in the RoutePlanner's start_node and end_node attributes.
    RoutePlanner::start_node = &m_Model.FindClosestNode(start_x,start_y);
    RoutePlanner::end_node = &m_Model.FindClosestNode(end_x,end_y);
}


// the CalculateHValue method.
float RoutePlanner::CalculateHValue(RouteModel::Node const *node) {
    return node->distance(*RoutePlanner::end_node);
}


// the AddNeighbors method to expand the current node by adding all unvisited neighbors to the open list.
void RoutePlanner::AddNeighbors(RouteModel::Node *current_node) {
    // get current_node.neighbors vector with all the neighbors
    current_node->FindNeighbors();
    // looping each neighbors node
    for (auto &node: current_node->neighbors)
    {
        // set the parent, the h_value, the g_value. 
        node->parent = current_node;
        // calculate the total distance between this node and the start point, by adding the previous distance with the distance to the neighbor
        node->g_value = current_node->g_value + node->distance(*current_node); 
        // h Value calculation.
        node->h_value = CalculateHValue(node);
        node->visited = true;
        RoutePlanner::open_list.push_back(node);
    }

}

bool compare_nodes(const RouteModel::Node *node1,const RouteModel::Node *node2)
{
    return (node1->g_value + node1->h_value) < (node2->g_value + node2->h_value);
}

// the NextNode method to sort the open list and return the next node.
// - Sort the open_list according to the sum of the h value and g value.
// - Create a pointer to the node in the list with the lowest sum.
// - Remove that node from the open_list.
// - Return the pointer.

RouteModel::Node *RoutePlanner::NextNode() {

    std::sort(RoutePlanner::open_list.begin(),RoutePlanner::open_list.end(),&compare_nodes);
    RouteModel::Node * node_ptr;
    node_ptr = RoutePlanner::open_list[0];
    RoutePlanner::open_list.erase(RoutePlanner::open_list.begin());
    return node_ptr;
}


// the ConstructFinalPath method to return the final path found from your A* search.


std::vector<RouteModel::Node> RoutePlanner::ConstructFinalPath(RouteModel::Node *current_node) {
    // Create path_found vector
    distance = 0.0f;
    std::vector<RouteModel::Node> path_found;

    // insert the end point
    path_found.insert(path_found.begin(), *current_node);
    distance += current_node->distance(*(current_node->parent));
    // insert the reset of the points
    do
    {
        /* code */

        current_node = current_node->parent;
        if(current_node == nullptr)break;
        path_found.insert(path_found.begin(), *current_node);
        if(current_node->parent != nullptr)
        {
            distance += current_node->distance(*(current_node->parent));
        }
        
    } while (current_node != RoutePlanner::start_node );
    

    distance *= m_Model.MetricScale(); // Multiply the distance by the scale of the map to get meters.
    return path_found;

}

// Write the A* Search algorithm here.

void RoutePlanner::AStarSearch() {
    RouteModel::Node *current_node = nullptr;
    // start from start node
    current_node = RoutePlanner::start_node;
    // no parent for the start point
    current_node->parent = nullptr;
    // mark the start point as visited
    current_node->visited = true;
    // start search
    do
    {
        /* code */
        RoutePlanner::AddNeighbors(current_node);    
        current_node = RoutePlanner::NextNode();

    } while ( current_node != RoutePlanner::end_node );

    // construct the final path
    m_Model.path = RoutePlanner::ConstructFinalPath(current_node);

}