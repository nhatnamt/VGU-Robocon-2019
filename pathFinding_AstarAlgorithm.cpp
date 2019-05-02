#include <bits/stdc++.h>
using namespace std;
using namespace this_thread;
using namespace chrono;
typedef pair<int, int> pi;
typedef pair<double, pi> pii;
const int inf = 1e9;
const int dx[4] = {0, -1, 0, 1};
const int dy[4] = {-1, 0, 1, 0};
const int N = 111;

int n, m, g[N][N];
bool vst[N][N];
char a[N][N], b[N][N];
pi S, T, trace[N][N];

double heuristic (int x, int y) {
  double manhattan=(x-T.first)*(x-T.first)+(y-T.second)*(y-T.second);
  double euclidean=sqrt(pow(x-T.first, 2.0)+pow(y-T.second, 2.0));
  return (manhattan+euclidean)/2.0;
}

void pathTracing(pi T) {
  ofstream cout("output.txt", ofstream::out);
  while (trace[T.first][T.second].first != -1) {
    pi t = trace[T.first][T.second];
    a[t.first][t.second] = '#';
    T = t;
  }
  for (int i = 0; i < n; ++i) {
    for (int j = 0; j < m; ++j) {
      cout << a[i][j];
      a[i][j] = b[i][j];
    }
    cout << endl;
  }
  sleep_for(nanoseconds(10000000));
  cout.close();
}

void AstarSearch() {
  priority_queue<pii, vector<pii>, greater<pii>> Q;
  for (int i = 0; i < n; ++i)
    for (int j = 0; j < m; ++j) 
      g[i][j] = inf;
  g[S.first][S.second] = 0;
  Q.push({0, S});
  trace[S.first][S.second] = {-1, -1};
  while (!Q.empty()) {
    pi u = Q.top().second;
    Q.pop();
    if (u == T) break;
    pathTracing(u);
    vst[u.first][u.second] = true;
    for (int i = 0; i < 4; ++i) {
      int X = u.first + dx[i];
      int Y = u.second + dy[i];
      if (X < 0 || X >= n || Y < 0 || Y >= m || a[X][Y] == '*') continue;
      int newCost = g[u.first][u.second] + 1;
      if (!vst[X][Y] && newCost < g[X][Y]) {
        g[X][Y] = newCost;
        double F = newCost*1.0 + heuristic(X, Y);
        trace[X][Y] = u;
        Q.push({F, pi(X, Y)});
      }
    }
  }
}

int main() {
  //ios::sync_with_stdio(0), cin.tie(0), cout.tie(0);
  freopen("input.txt", "r", stdin);
  cin >> n >> m;
  for (int i = 0; i < n; ++i)
    for (int j = 0; j < m; ++j) {
      cin >> a[i][j];
      b[i][j] = a[i][j];
      if (a[i][j] == 'B') 
        S = {i, j};
      else if (a[i][j] == 'C')
        T = {i, j};
    }
  AstarSearch();
}