#include "route_planner.h"

#include <algorithm>

using std::cout;
using std::endl;
using std::sort;
using std::string;
using std::vector;

RoutePlanner::RoutePlanner(RouteModel &model, float start_x, float start_y, float end_x, float end_y) : m_Model(model)
{
    // Convert inputs to percentage:
    start_x *= 0.01;
    start_y *= 0.01;
    end_x *= 0.01;
    end_y *= 0.01;

    // Use the m_Model.FindClosestNode method to find the closest nodes to the starting and ending coordinates.
    // Store the nodes you find in the RoutePlanner's start_node and end_node attributes.
    m_Model = model;
    start_node = &(m_Model.FindClosestNode(start_x, start_y));
    end_node = &(m_Model.FindClosestNode(end_x, end_y));
}

// CalculateHValue method.
// - Use the distance to the end_node for the h value.
// - Node objects have a distance method to determine the distance to another node.
float RoutePlanner::CalculateHValue(RouteModel::Node const *node)
{
    return node->distance(*this->end_node);
}

// AddNeighbors method to expand the current node by adding all unvisited neighbors to the open list.
void RoutePlanner::AddNeighbors(RouteModel::Node *current_node)
{
    // Use the FindNeighbors() method of the current_node to populate current_node.neighbors vector with all the neighbors.
    current_node->FindNeighbors();

    // For each node in current_node.neighbors, set the parent, the h_value, the g_value.
    for (auto n_node : current_node->neighbors)
    {

        n_node->parent = current_node;
        // Use CalculateHValue below to implement the h-Value calculation.
        n_node->h_value = this->CalculateHValue(n_node);
        n_node->g_value = current_node->g_value + current_node->distance(*n_node);
        // For each node in current_node.neighbors, add the neighbor to open_list
        this->open_list.push_back(n_node);
        // Set the node's visited attribute to true.
        n_node->visited = true;
    }
}

// NextNode method to sort the open list and return the next node.
RouteModel::Node *RoutePlanner::NextNode()
{
    // Sort the open_list according to the sum of the h value and g value.
    sort(this->open_list.begin(), this->open_list.end(), [ ](const RouteModel::Node *a, const RouteModel::Node *b)
    {
        float f1 = a->g_value + a->h_value;
        float f2 = b->g_value + b->h_value;
        return f1 < f2;
    });
    // Create a pointer to the node in the list with the lowest sum.
    RouteModel::Node *nody = this->open_list.front();
    // Remove that node from the open_list.
    this->open_list.erase(this->open_list.begin());
    // Return the pointer.
    return nody;
}

// ConstructFinalPath method to return the final path found from your A* search.
std::vector<RouteModel::Node> RoutePlanner::ConstructFinalPath(RouteModel::Node *current_node)
{
    distance = 0.0f;
    // Create path_found vector
    std::vector<RouteModel::Node> path_found;
    path_found.push_back(*current_node);
    //iteratively follow the chain of parents of nodes until the starting node is found.
    while (current_node!=nullptr && current_node->parent!=nullptr)
    {
        #if defined(DEBUG)
        cout << current_node->g_value << "|" << current_node->h_value << "=" << (current_node->h_value + current_node->g_value) << endl;
        #endif
        path_found.push_back(*(current_node->parent));
        // For each node in the chain, add the distance from the node to its parent to the distance variable.
        distance += current_node->distance(*(current_node->parent));
        // set current_node to parent of previous current_node
        current_node = current_node->parent;

        if (current_node == start_node)
        {
            break;
        }
    }
    // The returned vector should be in the correct order: the start node should be the first element
    // of the vector, the end node should be the last element.    
    // reverse the final path to achieve it
    std::reverse(std::begin(path_found), std::end(path_found));
    // Multiply the distance by the scale of the map to get meters.
    distance *= m_Model.MetricScale();
    return path_found;
}

// Write the A* Search algorithm here.
void RoutePlanner::AStarSearch()
{
    RouteModel::Node *current_node = start_node;
    current_node->visited = true;

    do
    {
        // Use the AddNeighbors method to add all of the neighbors of the current node to the open_list.        
        AddNeighbors(current_node);
        // Use the NextNode() method to sort the open_list and return the next node.
        current_node = NextNode();
        // When the search has reached the end_node
        if (current_node == end_node){
            // use the ConstructFinalPath method to return the final path that was found.
            auto duda = ConstructFinalPath(current_node);
            // Store the final path in the m_Model.path attribute before the method exits. This path will then be displayed on the map tile.
            m_Model.path = duda;
            break;
        }

    } while (open_list.size() > 0);
}