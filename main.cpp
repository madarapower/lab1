#include <iostream>
#include <queue>
#include <set>
#include <map>
#include "structs.h"
#include "utils.h"
#include <chrono>
#include <cmath>


class BFS //breadth-first-search
{
public:
    Result find_path(Node start, Node goal, Map grid)
    {
        auto time_now = std::chrono::high_resolution_clock::now();
        Result result;
        int steps = 0;
        start.g = 0;
        std::list<Node> OPEN;
        OPEN.push_back(start);
        std::set<Node> CLOSED;
        CLOSED.insert(start);
        bool pathfound = false;
        while(!OPEN.empty() && !pathfound)
        {
           Node current = OPEN.front();
           OPEN.pop_front();
           steps++;
           auto neighbors = grid.get_neighbors(current,4);
           for(auto n:neighbors) {
               if (CLOSED.find(n) == CLOSED.end())
               {
                   n.g = current.g + 1;
                   n.parent = &(*CLOSED.find(current));
                   OPEN.push_back(n);
                   CLOSED.insert(n);
                   if(n == goal) {
                       result.path = reconstruct_path(n);
                       result.cost = n.g;
                       pathfound = true;
                       break;
                    }
                }
            }
        }
        result.steps = steps;
        result.nodes_created = CLOSED.size();
        result.runtime = (std::chrono::high_resolution_clock::now() - time_now).count()/1e+9;
        return result;
    }
    std::list<Node> reconstruct_path(Node n)
    {
        std::list<Node> path;
        while(n.parent != nullptr)
        {
            path.push_front(n);
            n = *n.parent;
        }
        path.push_front(n);
        return path;
    }
};

class AStar
{
public:
    Result find_path(Node start, Node goal, Map grid, std::string metrictype, int connections, double hweight)
    {
        auto time_now = std::chrono::high_resolution_clock::now();
        Result result;
        int steps = 0;
        int w_value = 1;
        start.g = 0;
        start.h = count_h_value(start, goal, metrictype, hweight,w_value);
        start.f = start.g + start.h;
        std::priority_queue<Node, std::vector<Node>,CompareHeuristic> OPEN;
        OPEN.push(start);
        std::set<Node> CLOSED;
        std::map<Node, double> COST;
        COST[start] = 0;
        CLOSED.insert(start);
        bool pathfound = false;
        while (!OPEN.empty() && !pathfound)
        {
            Node current = OPEN.top();
            OPEN.pop();
            steps++;
            auto neighbors = grid.get_neighbors(current,connections);
            for (auto n : neighbors) {
                n.g = current.g + grid.get_cost(current, n, hweight);
                if (COST.find(n) == COST.end() || n.g < COST[n]) {
                    COST[n] = n.g;
                    n.h = count_h_value(current, goal, metrictype, hweight, w_value);
                    n.f = n.g + n.h;
                    n.parent = &(*CLOSED.find(current));
                    OPEN.push(n);
                    CLOSED.insert(n);           
                }
                if (n == goal) {
                    result.path = reconstruct_path(n);
                    result.cost = n.g;
                    pathfound = true;
                    break;
                }
                //}
            }
        }
        result.steps = steps;
        result.nodes_created = CLOSED.size();
        result.runtime = (std::chrono::high_resolution_clock::now() - time_now).count() / 1e+9;
        return result;
    }
    double count_h_value(Node current, Node goal, std::string metrictype, int hweight, int w_value)
    {
        if (metrictype == "Manhattan") {
            return w_value*abs(goal.i - current.i) + abs(goal.j - current.j);
        }
        else if (metrictype == "Octile") {
            return w_value*abs(abs(goal.i - current.i) - abs(goal.j - current.j)) + sqrt(2*hweight) * fmin(abs(goal.i - current.i), abs(goal.j - current.j));
        }
        else if (metrictype == "Euclidean") {
            return w_value*sqrt((goal.i - current.i) * (goal.i - current.i) + (goal.j - current.j) * (goal.j - current.j));
        }
    }
    std::list<Node> reconstruct_path(Node n)
    {
        std::list<Node> path;
        while (n.parent != nullptr)
        {   
            path.push_front(n);
            n = *n.parent;
        }
        path.push_front(n);
        return path;
    }
};

class Dijkstra
{
public:
    Result find_path(Node start, Node goal, Map grid)
    {
        auto time_now = std::chrono::high_resolution_clock::now();
        Result result;
        int steps = 0;
        start.g = 0;
        start.f = 0;
        std::priority_queue<Node, std::vector<Node>, CompareHeuristic> OPEN;
        OPEN.push(start);
        std::set<Node> CLOSED;
        CLOSED.insert(start);
        std::map<Node, double> COST;
        COST[start] = 0;
        bool pathfound = false;
        while (!OPEN.empty() && !pathfound)
        {
            Node current = OPEN.top();
            OPEN.pop();
            steps++;
            auto neighbors = grid.get_neighbors(current,4);
            for (auto n : neighbors) {
                n.g = current.g + 1;
                if (COST.find(n) == COST.end() || n.g < COST[n]) {
                    COST[n] = n.g;
                    n.f = n.g;
                    n.parent = &(*CLOSED.find(current));
                    OPEN.push(n);
                    CLOSED.insert(n);
                }
                if (n == goal) {
                    result.path = reconstruct_path(n);
                    result.cost = n.g;
                    pathfound = true;
                    break;
                }
            }
        }
        result.steps = steps;
        result.nodes_created = CLOSED.size();
        result.runtime = (std::chrono::high_resolution_clock::now() - time_now).count() / 1e+9;
        return result;
    }
    
    std::list<Node> reconstruct_path(Node n)
    {
        std::list<Node> path;
        while (n.parent != nullptr)
        {
            path.push_front(n);
            n = *n.parent;
        }
        path.push_front(n);
        return path;
    }
};

int main(int argc, char* argv[]) //argc - argumnet counter, argv - argument values
{
    
    for(int i=0; i<argc; i++)
        std::cout<<argv[i]<<"\n";
    if(argc<2)
    {
        std::cout << "Name of the input XML file is not specified."<<std::endl;
        return 1;
    }
    Loader loader;
    loader.load_instance(argv[1]);
    Result result;
    if(loader.algorithm == "BFS")
    {
        std::cout << "This is BFS" << std::endl;
        BFS bfs;
        result = bfs.find_path(loader.start, loader.goal, loader.grid);
    }

    else if (loader.algorithm == "Dijkstra") {
        std::cout << "This is Dijkstra" << std::endl;
        Dijkstra d;
        result = d.find_path(loader.start, loader.goal, loader.grid);
    }
    else  if (loader.algorithm == "AStar") {
        std::cout << "This is AStar" << std::endl;
        AStar astar;
        result = astar.find_path(loader.start, loader.goal, loader.grid, loader.metrictype, loader.connections, loader.hweight);
    }
    loader.grid.print(result.path);
    std::cout<<"Cost: "<<result.cost<<"\nRuntime: "<<result.runtime
    <<"\nSteps: "<<result.steps<<"\nNodes created: "<<result.nodes_created<<std::endl;
    return 0;
}