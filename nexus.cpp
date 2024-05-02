#include <iostream>
#include <vector>
#include <queue>
#include <limits>
using namespace std;

class Node
{
private:
    int id;
    vector<pair<Node *, int>> edges;

public:
    Node(int id)
    {
        this->id = id;
    }

    void addEdge(Node *neighbor, int weight)
    {

        edges.emplace_back(neighbor, weight);
    }

    vector<pair<Node *, int>> &getEdges()
    {
        return edges;
    }

    int getId()
    {
        return id;
    }
    int getNeighborweight(int i)
    {
        return edges[i].second;
    }
    int NoOfNeighbors()
    {
        return edges.size();
    }
    Node *getNeighbor(int i)
    {
        return edges[i].first;
    }
    vector<Node *> getNeighbor()
    {
        vector<Node *> r;
        for (int i = 0; i < edges.size(); i++)
        {
            r.push_back(edges[i].first);
        }
        return r;
    }
};

class Graph
{
private:
public:
    vector<Node *> nodes;
    void addNode()
    {
        nodes.push_back(new Node(nodes.size())); // Assign  IDs = No of nodes (unique)
    }

    // Undirected graph
    void addEdge(Node *start, Node *end, int weight)
    {
        start->addEdge(end, weight);
        end->addEdge(start, weight);
    }

    vector<Node *> &getNodes()
    {
        return nodes;
    }

    int TotalNodes()
    {
        return nodes.size();
    }
    Node *getNode(int i)
    {
        for (Node *node : nodes)
        {
            if (node->getId() == i)
            {
                return node;
            }
        }
        return nullptr;
    }
    ~Graph()
    {
        // Clean up dynamically allocated nodes
        for (Node* node : nodes)
        {
            delete node;
        }
    }
};

class Algorithm
{
protected:
    Graph &graph; // Reference to the Graph object
    const int totalNodes;
    vector<bool> vis;
public:

   Algorithm(Graph& graph) : graph(graph), totalNodes(graph.TotalNodes()), vis(totalNodes, false)
    {
    }

    void resetVisited()
    {
        for (int i = 0; i < this->totalNodes; i++)
        {
            vis[i] = false;
        }
    }
    virtual void execute(Node *startNode) = 0;
    virtual ~Algorithm() {} // virtual destructor because it will ensure proper cleanup of any derived classes
};

class DFS : public Algorithm
{
    // vector<bool> vis;

    
public:
    DFS(Graph &graph) : Algorithm(graph)
    {
    }

    void execute(Node *startNode) override
    {
        // impement dfs
        DFS_implementation(startNode);
        cout << endl;
        // make the visited vector false again
        resetVisited();
    }

    bool search_by_dfs(Node *startNode, int s)
    {
        // reset the visited vector
        resetVisited();

        // impement dfs
        bool res = DFS_implementation_forSearch(startNode, s);

        // make the visited vector false again
        resetVisited();

        return res;
    }
    void DFS_implementation(Node *startNode)
    {
        // DFS implementation
        int curr = startNode->getId();
        if (vis[curr])
        {
            return;
        }
        cout << curr << " ";
        vis[curr] = true;
        for (int i = 0; i < startNode->NoOfNeighbors(); i++)
        {
            Node *nextNode = startNode->getNeighbor(i);
            DFS_implementation(nextNode);
        }
    }
    int no_of_paths_to(Node *startNode, Node *endNode)
    {
        int paths = 0;
        vis[startNode->getId()] = true;
        no_of_paths_to(startNode, endNode, paths);
        resetVisited();
        return paths;
    }

    void no_of_paths_to(Node *startNode, Node *endNode, int &paths)
    {
        if (startNode->getId() == endNode->getId())
        {
            ++paths;
            return;
        }
        for (int i = 0; i < startNode->NoOfNeighbors(); i++)
        {
            Node *nextNode = startNode->getNeighbor(i);
            int curr = nextNode->getId();
            if (!vis[curr])
            {
                vis[curr] = true;
                no_of_paths_to(nextNode, endNode, paths);
                vis[curr] = false;
            }
        }
        return;
    }

    bool DFS_implementation_forSearch(Node *startNode, int search, bool state = false)
    { // int search  instead of int we can use template
        // DFS implementation
        int curr = startNode->getId();
        if (vis[curr])
        {
            return false;
        }
        if (curr == search)
        {
            return true;
        }
        vis[curr] = true;
        for (int i = 0; i < startNode->NoOfNeighbors(); i++)
        {
            Node *nextNode = startNode->getNeighbor(i);
            state = state || DFS_implementation_forSearch(nextNode, search, state);
        }

        return state;
    }
};

class BFS : public Algorithm
{
public:
    BFS(Graph &graph) : Algorithm(graph)
    {
    }

    void execute(Node *startNode) override
    {
        // BFS implementation
        queue<Node *> q;
        q.push(startNode);

        while (!q.empty())
        {
            Node *removed = q.front();
            q.pop();
            int curr = removed->getId();
            if (vis[curr] == false)
            {
                cout << curr << " ";
                vis[curr] = true;

                for (int i = 0; i < removed->NoOfNeighbors(); i++)
                {
                    Node *nextNode = removed->getNeighbor(i);
                    q.push(nextNode);
                }
            }
        }
        cout << endl;
        // make the visited vector false again
        resetVisited();
    }

    bool search_by_bfs(Node *startNode, int search)
    {
        queue<Node *> q;
        q.push(startNode);

        while (!q.empty())
        {
            Node *removed = q.front();
            q.pop();
            int curr = removed->getId();
            if (vis[curr] == false)
            {
                if (curr == search)
                {
                    return true;
                }
                vis[curr] = true;

                for (int i = 0; i < removed->NoOfNeighbors(); i++)
                {
                    Node *nextNode = removed->getNeighbor(i);
                    q.push(nextNode);
                }
            }
        }

        // make the visited vector false again
        resetVisited();

        return true;
    }

    int shortestUnweightedPathBFS(Node *startNode, Node *endNode = nullptr)
    {
        queue<Node *> q;
        vector<int> distance(this->totalNodes, -1); // Initialize distances to -1 (unvisited)

        q.push(startNode);
        distance[startNode->getId()] = 0; // Distance to startNode is 0

        while (!q.empty())
        {
            Node *curr = q.front();
            q.pop();

            for (Node *neighbor : curr->getNeighbor())
            {
                if (distance[neighbor->getId()] == -1)
                { // If neighbor is not visited
                    q.push(neighbor);
                    distance[neighbor->getId()] = distance[curr->getId()] + 1; // Update distance
                    if (endNode == neighbor)
                    {
                        return distance[neighbor->getId()];
                    }
                }
            }
        }

        // Print shortest path distances from startNode to all nodes
        cout << "Shortest path distances from node " << startNode->getId() << ":\n";
        for (size_t i = 0; i < this->totalNodes; ++i)
        {
            cout << "Node " << i << ": " << distance[i] << " edges away\n";
        }
        return -1;
    }
};

class Dijkstra : public Algorithm
{
public:
    vector<int> shortest_distance;
    vector<int> prevnode;

    Dijkstra(Graph &graph) : Algorithm(graph)
    {
        shortest_distance.resize(totalNodes, numeric_limits<int>::max());
        prevnode.resize(totalNodes, -1);
    }
    void execute(Node *startNode) override
    {
        resetVisited();
        vector<int> distances(graph.getNodes().size(), numeric_limits<int>::max()); // Initialize distances to infinity
        distances[startNode->getId()] = 0;                                          // Distance to starting node is 0

        priority_queue<pair<int, Node *>, vector<pair<int, Node *>>, greater<pair<int, Node *>>> pq;
        pq.push({0, graph.getNode(startNode->getId())}); // Push the starting node into the priority queue

        while (!pq.empty())
        {
            Node *currNode = pq.top().second;
            int currDistance = pq.top().first;
            pq.pop();

            for (const auto &edge : currNode->getEdges())
            {
                Node *neighbor = edge.first;
                int weight = edge.second;
                int newDistance = currDistance + weight;
                if (newDistance < distances[neighbor->getId()])
                {
                    distances[neighbor->getId()] = newDistance;
                    pq.push({newDistance, neighbor});
                }
            }
        }

        // Print the shortest distances to all nodes
        cout << "Shortest distances from " << startNode->getId() << ":" << endl;
        for (size_t i = 0; i < distances.size(); ++i)
        {
            cout << "To node " << i << ": " << (distances[i] == numeric_limits<int>::max() ? "unreachable" : to_string(distances[i])) << endl;
        }
        resetVisited();
    }
    vector<int> findshortestpath(Node *startNode, Node *endNode)
    {
        resetVisited();
        shortest_distance[startNode->getId()] = 0;
        vis[startNode->getId()] = true;
        prevnode[startNode->getId()] = startNode->getId();
        return findpath(startNode, endNode);
    }

    vector<int> findpath(Node *startNode, Node *endNode)
    {
        if (startNode->getId() == endNode->getId())
        {
            vector<int> path;
            int i = endNode->getId();

            while (prevnode[i] != i)
            {
                // cout << i << " ";
                path.push_back(i);
                i = prevnode[i];
            }
            path.push_back(i);
            return path;
        }
        int NoOfNeighbours = startNode->NoOfNeighbors();
        int closest_node;
        bool flag = false;
        int min = numeric_limits<int>::max();
        for (int i = 0; i < NoOfNeighbours; i++)
        {
            if (vis[startNode->getNeighbor(i)->getId()])
            {
                continue;
            }
            if (shortest_distance[startNode->getNeighbor(i)->getId()] > (shortest_distance[startNode->getId()] + startNode->getNeighborweight(i)))
            {
                shortest_distance[startNode->getNeighbor(i)->getId()] = shortest_distance[startNode->getId()] + startNode->getNeighborweight(i);
                prevnode[startNode->getNeighbor(i)->getId()] = startNode->getId();
            }
            if (shortest_distance[startNode->getNeighbor(i)->getId()] < min)
            {
                closest_node = i;
                flag = true;
                min = shortest_distance[startNode->getNeighbor(i)->getId()];
            }
        }
        if (flag == false)
        {
            vis[startNode->getId()] = true;
            startNode = graph.getNode(prevnode[startNode->getId()]);
            return findpath(startNode, endNode);
        }
        vis[startNode->getId()] = true;
        prevnode[startNode->getNeighbor(closest_node)->getId()] = startNode->getId();
        startNode = startNode->getNeighbor(closest_node);
        return findpath(startNode, endNode);
    }
};

class DetectCycle : public Algorithm
{
    Node *parentNode = nullptr;
public:
    DetectCycle(Graph &graph) : Algorithm(graph)
    {
    }

    void execute(Node *startNode = nullptr) override
    {
        if (startNode == nullptr)
        {
            if (!graph.getNodes().empty())
            {
                startNode = graph.getNodes()[0]; // Default to the first node in the graph
            }
            else
            {
                std::cerr << "Graph is empty! Unable to execute DetectCycle algorithm.\n";
                return;
            }
        }
        if (is_cyclic())
        {
            cout << "Yes this graph is Cyclic!\n";
            return;
        }
        else
        {
            cout << "This graph is not cyclic!\n";
        }
    }
    bool is_cyclic(Node *startNode = nullptr)
    {
        if (startNode == nullptr)
        {
            if (!graph.getNodes().empty())
            {
                startNode = graph.getNodes()[0]; // Default to the first node in the graph
            }
            else
            {
                std::cerr << "Graph is empty! Unable to execute DetectCycle algorithm.\n";
                return false;
            }
        }

        vis[startNode->getId()] = true;

        for (Node *neighbor : startNode->getNeighbor())
        {
            if (vis[neighbor->getId()] && neighbor != parentNode)
            {
                return true;
            }
            else if (!vis[neighbor->getId()])
            {
                parentNode = startNode;
                if (is_cyclic(neighbor))
                {
                    return true;
                }
            }
        }
        resetVisited();
        return false;
    }
};

class Bridge : public Algorithm
{
public:
    Bridge(Graph &graph) : Algorithm(graph)
    {
    }
    void execute(Node *startNode = nullptr)
    {
        if (startNode == nullptr)
        {
            if (!graph.getNodes().empty())
            {
                startNode = graph.getNodes()[0]; // Default to the first node in the graph
            }
            else
            {
                std::cerr << "Graph is empty! Unable to execute DetectCycle algorithm.\n";
                return;
            }
        }
        int t = totalNodes;
        int dt[t];
        int low[t];
        int time = 0;
        bool vis[t];
        for (int i = 0; i < t; i++)
        {
            if (!vis[i])
            {
                dfs(startNode, nullptr, dt, low, time);
            }
        }
    }

    void dfs(Node *curr, Node *parent, int dt[], int low[], int time)
    {
        vis[curr->getId()] = true;
        dt[curr->getId()] = low[curr->getId()] = ++time;
        for (Node *neighbor : curr->getNeighbor())
        {

            if (neighbor == parent)
                continue;
            if (vis[neighbor->getId()])
            {
                low[curr->getId()] = min(low[curr->getId()], dt[neighbor->getId()]);
            }
            else
            {
                dfs(neighbor, curr, dt, low, time);
                low[curr->getId()] = min(low[curr->getId()], low[neighbor->getId()]);
                if (dt[curr->getId()] < low[neighbor->getId()])
                {
                    cout << "BRIDGE : " << curr->getId() << "---" << neighbor->getId() << endl;
                }
            }
        }
    }
};

class Tarjan : public Algorithm
{
public:
    Tarjan(Graph &graph) : Algorithm(graph)
    {
    }
    void execute(Node *startNode) override
    {
        resetVisited();
        // implement tarjan
        vector<int> disc(totalNodes, -1);
        vector<int> low(totalNodes, -1);
        vector<int> parent(totalNodes, -1);
        vector<bool> articulation_points(totalNodes, false);
        // resetVisited();
        tarjan_implementation(startNode, disc, low, parent, articulation_points);

        cout << "articulation_points: ";
        for (int i = 0; i < articulation_points.size(); i++)
        {
            if (articulation_points[i])
            {
                cout << i << " ";
            }
        }
        resetVisited();

    }
    void explore(Node *currentNode)
    {
        if (vis[currentNode->getId()])
        {
            return;
        }
        vis[currentNode->getId()] = true;
        for (int i = 0; i < currentNode->NoOfNeighbors(); i++)
        {
            if (vis[currentNode->getNeighbor(i)->getId()])
            {
                continue;
            }
            explore(currentNode->getNeighbor(i));
        }
    }
    void tarjan_implementation(Node *startNode, vector<int> &disc, vector<int> &low, vector<int> &parent, vector<bool> &articulation_points, int time = 0)
    {
        if (vis[startNode->getId()])
        {
            return;
        }
        disc[startNode->getId()] = low[startNode->getId()] = time++;
        vis[startNode->getId()] = true;
        int children = 0;
        // cout << "Hello" << endl;
        for (int i = 0; i < startNode->NoOfNeighbors(); i++)
        {
            // cout << "Bye " << endl;
            int neighborId = startNode->getNeighbor(i)->getId();
            if (parent[startNode->getId()] == neighborId)
            {
                continue;
            }
            if (!vis[neighborId])
            {
                children++;
                parent[neighborId] = startNode->getId();
                tarjan_implementation(startNode->getNeighbor(i), disc, low, parent, articulation_points, time);
                low[startNode->getId()] = min(low[startNode->getId()], low[neighborId]);
                if (parent[startNode->getId()] != -1 && low[neighborId] >= disc[startNode->getId()])
                {
                    articulation_points[startNode->getId()] = true;
                }
            }
            else
            {
                low[startNode->getId()] = min(low[startNode->getId()], disc[neighborId]);
            }
        }
        // cout << ""
        if (parent[startNode->getId()] == -1 && children > 1)
        {
            articulation_points[startNode->getId()] = true;
        }
        return;
    }
};

class nexus
{
public:
    Graph *graph;
    nexus()
    {
        graph = new Graph(); // Create an instance of Graph
    }

    void addNode()
    {
        graph->addNode();
        return;
    }
    void addEdge(int s, int e, int w)
    {
        graph->addEdge(graph->nodes[s], graph->nodes[e], w);
        return;
    }
    vector<Node *> &getNodes()
    {
        return graph->getNodes();
    }
    int TotalNodes()
    {
        return graph->TotalNodes();
    }

    //--------------DFS----------------------------------------------------
    void execute_dfs(int start)
    {
        DFS *dfs = new DFS(*graph);

        dfs->execute(graph->nodes[start]);

        delete (dfs);
        return;
    }
    bool dfsearch(int s)
    {
        DFS *dfs = new DFS(*graph);
        bool search = dfs->search_by_dfs(graph->nodes[0], s);
        delete (dfs);
        return search;
    }
    int no_of_paths_byDFS(int start, int end)
    {
        DFS *dfs = new DFS(*graph);
        int numPaths = dfs->no_of_paths_to(graph->nodes[start], graph->nodes[end]);
        delete dfs;
        return numPaths;
    }

    //--------------BFS-------------------
    void execute_bfs(int start)
    {
        BFS *bfs = new BFS(*graph);

        bfs->execute(graph->nodes[start]);

        delete (bfs);
    }
    bool bfsearch(int s, int e = -1)
    {
        BFS *bfs = new BFS(*graph);
        bool search = bfs->search_by_bfs(graph->nodes[0], s);
        delete (bfs);
        return search;
    }
    int shortest_unweightedpath_bfs(int start, int end = -1)
    {
        BFS *bfs = new BFS(*graph);
        int dist;
        if (end == -1)
        {
            dist = bfs->shortestUnweightedPathBFS(graph->nodes[start]);
        }
        else
        {
            dist = bfs->shortestUnweightedPathBFS(graph->nodes[start], graph->nodes[end]);
        }
        delete (bfs);
        return dist;
    }
    //-------------Dj----------
    void execute_dj()
    {
        Dijkstra *dj = new Dijkstra(*graph);
        dj->execute(graph->nodes[0]);
        delete (dj);
    }
    vector<int> find_short_path(int s, int e){
        Dijkstra *dj = new Dijkstra(*graph);
        vector<int> ans = dj->findshortestpath(graph->nodes[s], graph->nodes[e]);
        delete(dj);
        return ans;
    }
    //------------DetectCycle------------------
    void execute_CycleDetection()
    {
        DetectCycle *dt = new DetectCycle(*graph);

        dt->execute();

        delete (dt);
    }
    bool isCyclic()
    {
        DetectCycle *dt = new DetectCycle(*graph);
        bool t = dt->is_cyclic();
        delete (dt);
        return t;
    }

    //--------------Bridge---------------
    void execute_Bridege(int start = 0)
    {
        Bridge *bge = new Bridge(*graph);

        bge->execute(graph->nodes[start]);

        delete (bge);
    }
    //--------Tarjan------------------
    void excute_tarjan()
    {

        Tarjan *dj = new Tarjan(*graph);
        dj->execute(graph->nodes[0]);
        delete (dj);
    }
};

int main()
{

    nexus Nexus;
    Nexus.addNode();
    Nexus.addNode();
    Nexus.addNode();
    Nexus.addNode();
    Nexus.addNode();
    Nexus.addNode();
    Nexus.addNode();
    // Add more nodes and edges
    Nexus.addEdge(0, 1, 2);
    Nexus.addEdge(0, 2, 4);
    Nexus.addEdge(1, 2, 1);
    Nexus.addEdge(1, 3, 3);
    Nexus.addEdge(1, 6, 3);
    Nexus.addEdge(2, 4, 2);
    Nexus.addEdge(2, 3, 2);
    Nexus.addEdge(3, 5, 2);

    //     4
    //     |
    //     2 -----
    //    /  \    \
    //   0 -- 1 -- 3 -- 5
    //       /
    //      6

   
    Nexus.execute_dfs(0);
   
    Nexus.execute_dfs(2);
    bool find5 = Nexus.dfsearch(5); 
    cout << find5 << endl;
    cout << Nexus.no_of_paths_byDFS(0, 3)<<endl; 


    bool find3 = Nexus.bfsearch(3); 
    cout << find3 << endl;
    int sp0 = Nexus.shortest_unweightedpath_bfs(0);
    cout << sp0 << endl;
    int sp1to4 = Nexus.shortest_unweightedpath_bfs(1, 4); 
    cout << "shortest distance form node 1 to 4: " << sp1to4 << " edges" << endl;

  
    Nexus.execute_CycleDetection();
    
    cout << Nexus.isCyclic() << endl;

    
    Nexus.execute_Bridege();
    
    Nexus.execute_dj();
    cout<<endl;
    vector<int> a =Nexus.find_short_path(0, 2);
    for(int i=0; i<a.size(); i++){cout<<a[i]<<"<--";}
    cout<<endl;
    cout<<endl;
    Nexus.excute_tarjan();
    return 0;
}

//      ______                                                            _______
//     | Node |                                                          | nexus |
//     |______|                                                          |_______|
//
//                     _______
//                    | Graph |
//                    |_______|           ___________
//                                       | Algorithm |
//                                       |___________|
//                                             |
//                                             |
//                                       All Algorithms
