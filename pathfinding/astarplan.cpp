#include <bits/stdc++.h>

using namespace std;

typedef pair <int, int> coordinates;
typedef pair <double, pair <coordinates, coordinates>> heap_data;

const int x_bound = 300; /// contest bounds: 300
const int y_bound = 300; /// contest bounds: 300
const int dir_to_storage = -1; /// -1: storage in left side; 1: storage in right side
const int safe_diameter = 8; /// safe diameter from the center point of the object
const int Xeps = 5; /// X-coor epsilon to push object

coordinates autobot;
vector <coordinates> object;
bool locked[x_bound + 15][y_bound + 15];
char matrix[x_bound + 15][y_bound + 15];
coordinates trace[x_bound + 15][y_bound + 15];

void get_data() {
    for (int i = 0; i <= x_bound; ++i)
        for (int j = 0; j <= y_bound; ++j) {
            cin >> matrix[i][j];
            if (matrix[i][j] == 'A')
                autobot.first = i,
                autobot.second = j;
            if (matrix[i][j] == 'O')
                object.push_back({i, j}),
                locked[i][j] = true;
        }
}

void update() {
    memset(locked, false, sizeof locked);
    for (coordinates obj: object)
        locked[obj.first][obj.second] = true;
}

bool pushable(int X, int Y) {
    for (coordinates obj: object) {
        int dX = obj.first - X;
        int dY = obj.second - Y;
        if (abs(dX) <= Xeps && dY * dir_to_storage > 0 && hypot(dX, dY) >= safe_diameter)
            return true;
    }
    return false;
}

bool object_clear(coordinates a, coordinates b) {
    for (coordinates obj: object) {
        double AOx = obj.first - a.first;
        double AOy = obj.second - a.second;
        double BOx = obj.first - b.first;
        double BOy = obj.second - b.second;
        double S = AOx * BOy - AOy * BOx; S = max(S, -S);
        double AB = hypot(a.first - b.first, a.second - b.second);
        if (2 * S <= AB * safe_diameter) return false;
    }
    return true;
}

double heuristic(int X, int Y) {
    return hypot(X - autobot.first, Y - autobot.second);
}

int sqr(int x) {return x * x;}

void output_path() {
    while (trace[autobot.first][autobot.second].first != -1)
        cout << autobot.first << ' ' << autobot.second << '\n',
        autobot = trace[autobot.first][autobot.second];
    cout << autobot.first << ' ' << autobot.second << '\n';
}

void push_object() {
    cout << autobot.first << ' ' << safe_diameter << '\n';
}

void return_home() {
    cout << ... << ' ' << ... << '\n';
}

void A_star() {
    priority_queue <heap_data, vector <heap_data>, greater <heap_data>> pos;
    while (!pos.empty()) pos.pop();
    for (int i = 0; i <= x_bound; ++i)
        for (int j = 0; j <= y_bound; ++j)
            if (!locked[i][j] && pushable(i, j))
                pos.push(make_pair(heuristic(i, j) + j, make_pair(make_pair(i, j), make_pair(-1, -1))));
    bool no_move = false;
    while (!pos.empty()) {
        coordinates cur = pos.top().second.first;
        coordinates pre = pos.top().second.second;
        double len = pos.top().first - heuristic(cur.first, cur.second);
        pos.pop();
        if (locked[cur.first][cur.second]) continue;
        locked[cur.first][cur.second] = true;
        trace[cur.first][cur.second] = pre;

        if (cur == autobot) {
            output_path();
            no_move = true;

            /// DEBUG
            matrix[cur.first][cur.second] = '.';
            matrix[pre.first][pre.second] = 'A';
            autobot = pre;
            ///

            break;
        }
        for (int i = 0; i <= x_bound; ++i)
            for (int j = 0; j <= y_bound; ++j)
                if (!locked[i][j] && object_clear(cur, make_pair(i, j)))
                    pos.push(make_pair(len + hypot(i - cur.first, j - cur.second) + heuristic(i, j), make_pair(make_pair(i, j), cur)));
    }
    if (no_move) return_home();
    else push_object();
}

int main() {
    ios::sync_with_stdio(0); cin.tie(0);
    freopen("data.txt", "r", stdin);
    freopen("path.txt", "w", stdout);
    get_data();
    if (pushable(autobot.x, autobot.y))
        push_object();
    else A_star();
}
