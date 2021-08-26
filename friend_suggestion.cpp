// #include <bits/stdc++.h>
#include <cstdio>
#include <cassert>
#include <vector>
#include <queue>
#include <limits>
#include <utility>

using namespace std;

// External vector of size 2 - for forward and backward search.
// Internal 2-dimensional vector is vector of adjacency lists for each node.
typedef vector<vector<vector<int>>> Adj;

// Distances can grow out of int type
typedef long long Len;

// Vector of two priority queues - for forward and backward searches.
// Each priority queue stores the closest unprocessed node in its head.
typedef vector<priority_queue<pair<Len, int>,vector<pair<Len,int>>,greater<pair<Len,int>>>> Queue;

const Len INFINITY = numeric_limits<long long>::max()/4;

class Bidijkstra {
    // Number of nodes
    int n_;
    // Graph adj_[0] and cost_[0] correspond to the initial graph,
    // adj_[1] and cost_[1] correspond to the reversed graph.
    // Graphs are stored as vectors of adjacency lists corresponding
    // to nodes.
    // Adjacency list itself is stored in adj_, and the corresponding
    // edge costs are stored in cost_.
    Adj adj_;
    Adj cost_;
    // distance_[0] stores distances for the forward search,
    // and distance_[1] stores distances for the backward search.
    vector<vector<Len>> distance_;
    // Stores all the nodes visited either by forward or backward search.
    vector<int> workset_;
    // Stores a flag for each node which is True iff the node was visited
    // either by forward or backward search.
    vector<bool> visited_;

public:
    Bidijkstra(int n, Adj adj, Adj cost)
        : n_(n), adj_(adj), cost_(cost), distance_(2, vector<Len>(n, INFINITY)), visited_(n)
    { workset_.reserve(n); }

    // Initialize the data structures before new query,
    // clear the changes made by the previous query.
    void clear() {
        for (int i = 0; i < workset_.size(); ++i) {
            int v = workset_[i];
            distance_[0][v] = distance_[1][v] = INFINITY;
            visited_[v] = false;
        }
        workset_.clear();
    }

    // Processes visit of either forward or backward search 
    // (determined by value of side), to node v trying to
    // relax the current distance by dist.
    void visit(Queue& pq, int side, int v, Len dist) {
        // Implement this method yourself
        if(distance_[side][v] > dist){
            pq[side].push({dist, v});
            distance_[side][v] = dist;
            workset_.push_back(v);
        }
    }

    // Returns the distance from s to t in the graph.
    Len query(int s, int t) {
        clear();
        Queue pq(2);
        visit(pq, 0, s, 0);
        visit(pq, 1, t, 0);
        // Implement the rest of the algorithm yourself
        while(!pq[0].empty() && !pq[1].empty()){
            int forward = pq[0].top().second;
            pq[0].pop();
            if(!visited_[forward]){
                for(int i=0;i<adj_[0][forward].size();i++){
                    if(distance_[0][adj_[0][forward][i]] > distance_[0][forward] + cost_[0][forward][i]){
                        visit(pq, 0, adj_[0][forward][i], distance_[0][forward] + cost_[0][forward][i]);
                    }
                }
                visited_[forward] = 1;
            }
            else{
                Len ans=INFINITY;
                for(int u:workset_){
                    if(distance_[0][u] + distance_[1][u] < ans){
                        ans = distance_[0][u] + distance_[1][u];
                    }
                }
                if(ans==INFINITY) return -1;
                return ans;
            }

            int reverse = pq[1].top().second;
            pq[1].pop();
            if(!visited_[reverse]){
                for(int i=0;i<adj_[1][reverse].size();i++){
                    if(distance_[1][adj_[1][reverse][i]] > distance_[1][reverse] + cost_[1][reverse][i]){
                        visit(pq, 1, adj_[1][reverse][i], distance_[1][reverse] + cost_[1][reverse][i]);
                    }
                }
                visited_[reverse] = 1;
            }
            else{
                Len ans=INFINITY;
                for(int u:workset_){
                    if(distance_[0][u] + distance_[1][u] < ans){
                        ans = distance_[0][u] + distance_[1][u];
                    }
                }
                if(ans==INFINITY) return -1;
                return ans;
            }
        }
        return -1;
    }
};

int main() {
    int n, m;
    scanf("%d%d", &n, &m);
    Adj adj(2, vector<vector<int>>(n));
    Adj cost(2, vector<vector<int>>(n));
    for (int i=0; i<m; ++i) {
        int u, v, c;
        scanf("%d%d%d", &u, &v, &c);
        adj[0][u-1].push_back(v-1);
        cost[0][u-1].push_back(c);
        adj[1][v-1].push_back(u-1);
        cost[1][v-1].push_back(c);
    }

    Bidijkstra bidij(n, adj, cost);

    int t;
    scanf("%d", &t);
    for (int i=0; i<t; ++i) {
        int u, v;
        scanf("%d%d", &u, &v);
        printf("%lld\n%lld\n", bidij.query(u-1, v-1), INFINITY);
    }
}
