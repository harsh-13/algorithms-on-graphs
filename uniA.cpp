#include <cstdio>
#include <cassert>
#include <vector>
#include <queue>
#include <limits>
#include <cmath>
#include <utility>

using namespace std;

// See the explanations of these typedefs and constants in the starter for friend_suggestion
typedef vector<vector<int>> Adj;
typedef long long Len;
typedef priority_queue<pair<Len, int>,vector<pair<Len,int>>,greater<pair<Len,int>>> Queue;

#define infinite  1e13

class AStar {
    // See the descriptions of these fields in the starter for friend_suggestion
    int n_;
    Adj adj_;
    Adj cost_;
    vector<Len> distance_;
    vector<Len> potentialDist_;
    vector<int> workset_;
    vector<bool> visited_;
    // Coordinates of the nodes
    std::vector<std::pair<Len,Len>> xy_;

public:
    AStar(int n, Adj adj, Adj cost, std::vector<std::pair<Len,Len>> xy)
        : n_(n), adj_(adj), cost_(cost), distance_(vector<Len>(n_, infinite)), potentialDist_(vector<Len>(n_, infinite)), visited_(n), xy_(xy)
    { workset_.reserve(n); }

    // See the description of this method in the starter for friend_suggestion
    void clear() {
        for (int i = 0; i < workset_.size(); ++i) {
            int v = workset_[i];
            distance_[v] = infinite;
            potentialDist_[v] = infinite;
            visited_[v] = false;
        }
        workset_.clear();
    }

    // See the description of this method in the starter for friend_suggestion
    void visit(Queue& pq, int v, Len dist, Len potenial) {
        // Implement this method yourself
        if(potentialDist_[v] > dist + potenial){
            pq.push({dist + potenial, v});
            distance_[v] = dist;
            potentialDist_[v] = dist + potenial;
            workset_.push_back(v);
        }
    }

    Len pot(Len x, Len t){
        return sqrt(pow(xy_[t].first - xy_[x].first, 2) + pow(xy_[t].second - xy_[x].second, 2));
    }

    // Returns the distance from s to t in the graph
    Len query(int s, int t) {
        clear();
        Queue pq;
        visit(pq, s, 0, pot(s,t));
        // Implement the rest of the algorithm yourself
        while(!pq.empty()){
            pair<Len, int> temp = pq.top();
            int v = temp.second;
            pq.pop();
            if(v==t){
                return distance_[t];
            }
            if(!visited_[v]){
                for(int i=0;i<adj_[v].size();i++){
                    if(!visited_[adj_[v][i]])
                        visit(pq, adj_[v][i], distance_[v] + cost_[v][i], pot(adj_[v][i], t));
                }
                visited_[v] = 1;
            }
        }
        return -1;
    }
};

int main() {
    int n, m;
    scanf("%d%d", &n, &m);
    std::vector<std::pair<Len,Len>> xy(n);
    for (int i=0;i<n;++i){
        int a, b;
        scanf("%d%d", &a, &b);
        xy[i] = make_pair(a,b);
    }
    vector<vector<int> >adj(n), cost(n);
    for (int i=0; i<m; ++i) {
        int u, v, c;
        scanf("%d%d%d", &u, &v, &c);
        adj[u-1].push_back(v-1);
        cost[u-1].push_back(c);
    }

    AStar astar(n, adj, cost, xy);

    int t;
    scanf("%d", &t);
    for (int i=0; i<t; ++i) {
        int u, v;
        scanf("%d%d", &u, &v);
        printf("%lld\n", astar.query(u-1, v-1));
    }
}
