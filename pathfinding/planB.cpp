#include <bits/stdc++.h>

using namespace std;

typedef pair <int, int> coordinates;
typedef pair <double, pair <coordinates, coordinates>> heap_data;

const int stepX[4] = {-1, 0, 1, 0};
const int stepY[4] = {0, -1, 0, 1};
const int x_bound = 9; /// contest bounds: 300
const int y_bound = 9; /// contest bounds: 300
const int dir_to_storage = -1; /// -1: storage in left side; 1: storage in right side
const int process_radius = 3; /// radius of zone that robot works on before getting new data

coordinates autobot;
vector <coordinates> object;
bool locked[x_bound + 15][y_bound + 15];
char matrix[x_bound + 15][y_bound + 15];

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

void debug() {
    for (int i = 0; i <= x_bound; ++i) {
        for (int j = 0; j <= y_bound; ++j)
            cout << matrix[i][j];
        cout << '\n';
    }
    cout << '\n';
}

void update() {
    memset(locked, false, sizeof locked);
    for (coordinates obj: object)
        locked[obj.first][obj.second] = true;
}

bool pushable(int X, int Y) {
    for (coordinates obj: object)
        if (obj.first == X && obj.second - Y == dir_to_storage)
            return true;
    return false;
}

bool object_clear(coordinates a, coordinates b) {
    for (coordinates obj: object)
        if ((obj.first - a.first) * (obj.second - b.second) == (obj.first - b.first) * (obj.second - a.second))
            return false;
    return true;
}

double heuristic(int X, int Y) {
    return hypot(X - autobot.first, Y - autobot.second);
}

int sqr(int x) {return x * x;}

void output_next_move(coordinates pos) {
}

void A_star() {
    priority_queue <heap_data, vector <heap_data>, greater <heap_data>> pos;
    while (!pos.empty()) pos.pop();
    for (int i = 0; i <= x_bound; ++i)
        for (int j = 0; j <= y_bound; ++j)
            if (!locked[i][j] && pushable(i, j))
                pos.push(make_pair(heuristic(i, j) /** + distance_to_storage**/, make_pair(make_pair(i, j), make_pair(-1, -1))));
    while ("robocon" != "easy") {
        coordinates cur = pos.top().second.first;
        coordinates pre = pos.top().second.second;
        double len = pos.top().first - heuristic(cur.first, cur.second);
        pos.pop();
        if (locked[cur.first][cur.second]) continue;
        locked[cur.first][cur.second] = true;

        if (cur == autobot) {
            output_next_move(pre);

            /// DEBUG
            matrix[cur.first][cur.second] = '.';
            matrix[pre.first][pre.second] = 'A';
            autobot = pre;
            ///

            return;
        }

        int process_x1 = max(0, cur.first - process_radius);
        int process_x2 = min(x_bound, cur.first + process_radius);
        for (int i = process_x1; i <= process_x2; ++i) {
            int process_y1 = max(0, cur.second - int(sqrt(sqr(process_radius) - sqr(i - cur.first))));
            int process_y2 = min(y_bound, cur.second + int(sqrt(sqr(process_radius) - sqr(i - cur.first))));
            for (int j = process_y1; j <= process_y2; ++j)
                if (!locked[i][j] && object_clear(cur, make_pair(i, j)))
                    pos.push(make_pair(len + hypot(i - cur.first, j - cur.second) + heuristic(i, j), make_pair(make_pair(i, j), cur)));
        }
    }
}

void push_object() {
}

int main() {
    ios::sync_with_stdio(0); cin.tie(0);
    freopen("data.txt", "r", stdin);
    freopen("out.txt", "w", stdout);
    get_data();
    do {
        debug();
        if (pushable(autobot.first, autobot.second))
            return 0;
        update(); A_star();
    } while ("robocon" != "easy");

/**
    get_data();
    if (pushable(autobot.x, autobot.y))
        push_object();
    else A_star();
**/
}
