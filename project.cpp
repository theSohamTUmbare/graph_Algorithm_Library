#include<iostream>
using namespace std;
#include <vector>
#include <queue>
#include <limits>
using namespace std;

class Node {
private:
    int id;
    vector<pair<Node*, int>> edges;

public:
    Node(int id){
        this->id = id;
    }

    void addEdge(Node* neighbor, int weight) {
        
        edges.emplace_back(neighbor, weight);
    }

    vector<pair<Node*, int>>& getEdges() {
        return edges;
    }

    int getId() {
        return id;
    }
    int NoOfNeighbors(){
        return edges.size();
    }
    Node* getNeighbor(int i=0){
        return edges[i].first;

    }
};

class Edge {
private:
    Node* start;
    Node* end;
    int weight;

public:
    Edge(Node* start, Node* end, int weight) : start(start), end(end), weight(weight) {}

    Node* getStart() const {
        return start;
    }

    Node* getEnd() const {
        return end;
    }

    int getWeight() const {
        return weight;
    }
};

class Graph {
private:

public:
    vector<Node*> nodes;
    void addNode() {
        nodes.push_back(new Node(nodes.size())); // Assign  IDs = No of nodes (unique)
    }

    // Undirected graph
    void addEdge(Node* start, Node* end, int weight) {
        start->addEdge(end, weight);
        end->addEdge(start, weight); 
    }

    vector<Node*>& getNodes() {
        return nodes;
    }
    
    int TotalNodes(){
        return nodes.size();
    }
};

class Algorithm {
public:
    vector<bool> vis;
    int totalNodes;
    Algorithm(int totalNodes){
        this->totalNodes = totalNodes;
        for(int i=0; i<totalNodes; i++){
            vis.push_back(false);
        }
    }
    virtual void execute(Node* startNode) = 0;
    virtual ~Algorithm() {}     // virtual destructor because it will ensure proper cleanup of any derived classes
};

class DFS : public Algorithm {
    // vector<bool> vis;

    void resetVisited(){
        for(int i=0; i<this->totalNodes; i++){
            vis[i] = false;
        }
    }

public:
    DFS(int totalNodes) : Algorithm(totalNodes){    
        
    }

    void execute(Node* startNode) override {
        // impement dfs
        DFS_implementation(startNode);
        cout<<endl;
        // make the visited vector false again
        resetVisited();
    }
    
    bool search_by_dfs(Node* startNode, int s){
        // impement dfs
        bool res = DFS_implementation_forSearch(startNode, s);

        // make the visited vector false again
        resetVisited();
        
        return res;
    }
    void DFS_implementation(Node* startNode){
        // DFS implementation
        int curr = startNode->getId();
        if(vis[curr]){
            return;
        }
        cout<<curr<<" ";
        vis[curr] = true;
        for(int i=0; i<startNode->NoOfNeighbors(); i++){
            Node * nextNode = startNode->getNeighbor(i);
            DFS_implementation(nextNode);
        }
    }
    int no_of_paths_to(Node* startNode, Node* endNode){
        int paths =0;
        vis[startNode->getId()] = true;
        no_of_paths_to(startNode, endNode, paths);
        resetVisited();
        return paths;
    }

    void no_of_paths_to(Node* startNode, Node* endNode, int &paths){
        if(startNode->getId() == endNode->getId()){
            ++paths;
            return;
        }
        for(int i=0; i<startNode->NoOfNeighbors(); i++){
            Node * nextNode = startNode->getNeighbor(i);
            int curr = nextNode->getId();
            if(!vis[curr]){
                vis[curr] = true;
                no_of_paths_to(nextNode, endNode, paths);
                vis[curr] = false;
            }
        }
        return;
    }

    bool DFS_implementation_forSearch(Node* startNode, int search, bool state=false){ // int search  instead of int we can use template 
        // DFS implementation
        int curr = startNode->getId();
        if(vis[curr]){
            return false;
        }
        if(curr == search){
            return true;
        }
        vis[curr] = true;
        for(int i=0; i<startNode->NoOfNeighbors(); i++){
            Node * nextNode = startNode->getNeighbor(i);
            state = state || DFS_implementation_forSearch(nextNode, search, state);
        }

        return state;

    }

};

class BFS : public Algorithm {
    void resetVisited(){
        for(int i=0; i<this->totalNodes; i++){
            vis[i] = false;
        }
    }

public:
    BFS(int totalNodes) : Algorithm(totalNodes){
       
    }

    void execute(Node* startNode) override {
        // BFS implementation
        queue<Node*> q;
        q.push(startNode);

        while(!q.empty()){
            Node *removed = q.front();
            q.pop();
            int curr = removed->getId();
            if(vis[curr] == false){
                cout<<curr<<" ";
                vis[curr] = true;

                for(int i=0; i<removed->NoOfNeighbors(); i++){
                    Node * nextNode = removed->getNeighbor(i);
                    q.push(nextNode);
                    
                }       
            }
        }
        cout<<endl;
        // make the visited vector false again
        resetVisited();
    }

    bool search_by_bfs(Node *startNode, int search){
        queue<Node*> q;
        q.push(startNode);

        while(!q.empty()){
            Node *removed = q.front();
            q.pop();
            int curr = removed->getId();
            if(vis[curr] == false){
                if(curr == search){
                    return true;
                }
                vis[curr] = true;

                for(int i=0; i<removed->NoOfNeighbors(); i++){
                    Node * nextNode = removed->getNeighbor(i);
                    q.push(nextNode);
                }       
            }
        }

        // make the visited vector false again
        resetVisited();   

        return true;
    }
};

class Dijkstra : public Algorithm {
public:
    void execute( Node* startNode) override {
       
    }
    
};

int main() {
    // Usage example
    Graph graph;
    graph.addNode();
    graph.addNode();
    graph.addNode();
    graph.addNode();
    graph.addNode();
    graph.addNode();
    graph.addNode();
    // Add more nodes and edges
    graph.addEdge(graph.nodes[0], graph.nodes[1], 2);              
    graph.addEdge(graph.nodes[0], graph.nodes[2], 4);
    graph.addEdge(graph.nodes[1], graph.nodes[2], 1);
    graph.addEdge(graph.nodes[1], graph.nodes[3], 3);
    graph.addEdge(graph.nodes[1], graph.nodes[6], 3);
    graph.addEdge(graph.nodes[2], graph.nodes[4], 2);
    graph.addEdge(graph.nodes[2], graph.nodes[3], 2);
    graph.addEdge(graph.nodes[3], graph.nodes[5], 2);


    //     4
    //     |
    //     2 -----
    //    /  \    \
    //   0 -- 1 -- 3 -- 5
    //       /
    //      6 


    int totalNodes = graph.TotalNodes();


    
    DFS dfs(totalNodes);
    dfs.execute(graph.getNodes()[0]);
    dfs.execute(graph.getNodes()[1]);
    bool find5 = dfs.search_by_dfs(graph.getNodes()[0], 5);
    cout<<find5<<endl;
    cout<<dfs.no_of_paths_to(graph.getNodes()[0], graph.getNodes()[3])<<endl;
    

    BFS bfs(totalNodes);
    bfs.execute(graph.getNodes()[0]);
    bool find3 = bfs.search_by_bfs(graph.getNodes()[6], 3);
    cout<<find3<<endl;
    
    // Dijkstra dijkstra;
    // dijkstra.execute(graph, graph.getNodes()[0]);

    return 0;
}
