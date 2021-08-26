#include <cstdio>
#include <cassert>
#include <vector>
#include <queue>
#include <limits>
#include <utility>
#include <cmath>

using namespace std;

// See the explanations of these typedefs and constants in the starter for friend_suggestion
typedef vector<vector<vector<int>>> Adj;
typedef long long Len;
typedef vector<priority_queue<pair<Len, int>,vector<pair<Len,int>>,greater<pair<Len,int>>>> Queue;

#define infinite  1e13

class AStar {
    // See the descriptions of these fields in the starter for friend_suggestion
    int n_;
    Adj adj_;
    Adj cost_;
    vector<vector<Len>> distance_;
    vector<vector<Len>> potentialDist_;
    vector<int> workset_;
    vector<vector<double>> euclid;
    vector<vector<bool>> visited_;
    // Coordinates of the nodes
    std::vector<std::pair<Len,Len>> xy_;

public:
    AStar(int n, Adj adj, Adj cost, std::vector<std::pair<Len,Len>> xy)
        : n_(n), adj_(adj), cost_(cost), distance_(2, vector<Len>(n_, infinite)), potentialDist_(2, vector<Len>(n_, infinite)), visited_(2, vector<bool>(n_, false)), xy_(xy)
    { workset_.reserve(n); euclid.resize(2, vector<double>(n, 0));}

    // See the description of this method in the starter for friend_suggestion
    void clear() {
        for (int i = 0; i < workset_.size(); ++i) {
            int v = workset_[i];
            distance_[0][v] = distance_[1][v] = infinite;
            potentialDist_[0][v] = potentialDist_[1][v] = infinite;
            visited_[0][v] = visited_[1][v] = false;
            euclid[0][v] = euclid[1][v] = 0;
        }
        workset_.clear();
    }

    Len pot(Len x, Len t){
        return sqrt(pow(xy_[t].first - xy_[x].first, 2) + pow(xy_[t].second - xy_[x].second, 2));
    }

    void euclidean_distance(int side, int s) {
        for (int i = 0;i < n_;i++) {
            double x_ = (xy_[s].first - xy_[i].first);
            double y_ = (xy_[s].second - xy_[i].second);
            double d = x_ * x_ + y_ * y_;
            d = sqrt(d);
            euclid[side][i] = d;
        }
    }

    void balance_potential() {
        for (int i = 0;i < n_;i++) {
            euclid[0][i] = ((euclid[0][i] - euclid[1][i]) / 2);
            euclid[1][i] = -(euclid[0][i]);
        }
    }

    // See the description of this method in the starter for friend_suggestion
    void visit(Queue& pq, int side, int v, Len dist, Len potential) {
        // Implement this method yourself
        if(potentialDist_[side][v]>dist+potential){
            pq[side].push({dist+potential, v});
            distance_[side][v] = dist;
            potentialDist_[side][v] = dist + potential;
            workset_.push_back(v);
        }
    }

    // Returns the distance from s to t in the graph
    Len query(int s, int t) {
        clear();
        euclidean_distance(0, t); //O(n*(O(sqrt())
        euclidean_distance(1, s);//O(n*(O(sqrt())
        balance_potential(); //O(n)
        Queue pq(2);
        visit(pq, 0, s, 0, euclid[0][s]);
        visit(pq, 1, t, 0, euclid[1][t]);
        // Implement the rest of the algorithm yourself
        while(!pq[0].empty() && !pq[1].empty()){
            int vforward = pq[0].top().second;
            pq[0].pop();
            if(!visited_[1][vforward]){
                for(int i=0;i<adj_[0][vforward].size();i++){
                    visit(pq, 0, adj_[0][vforward][i], distance_[0][vforward] + cost_[0][vforward][i], euclid[0][adj_[0][vforward][i]]);
                }
                visited_[0][vforward] = 1;
            }
            else{
                Len ans=infinite;
                for(int u:workset_){
                    if(distance_[0][u] + distance_[1][u] < ans){
                        ans = distance_[0][u] + distance_[1][u];
                    }
                }
                if(ans==infinite) return -1;
                return ans;
            }

            int vreverse = pq[1].top().second;
            pq[1].pop();
            if(!visited_[0][vreverse]){
                for(int i=0;i<adj_[1][vreverse].size();i++){
                    visit(pq, 1, adj_[1][vreverse][i], distance_[1][vreverse] + cost_[1][vreverse][i], euclid[1][adj_[1][vreverse][i]]);
                }
                visited_[1][vreverse] = 1;
            }
            else{
                Len ans=infinite;
                for(int u:workset_){
                    if(distance_[0][u] + distance_[1][u] < ans){
                        ans = distance_[0][u] + distance_[1][u];
                    }
                }
                if(ans==infinite) return -1;
                return ans;
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

    AStar astar(n, adj, cost, xy);

    int t;
    scanf("%d", &t);
    for (int i=0; i<t; ++i) {
        int u, v;
        scanf("%d%d", &u, &v);
        printf("%lld\n", astar.query(u-1, v-1));
    }
}
