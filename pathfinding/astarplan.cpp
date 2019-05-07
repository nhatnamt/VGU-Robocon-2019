#include <bits/stdc++.h>

using namespace std;

typedef pair <int, int> coordinates;
typedef pair <double, pair <coordinates, coordinates>> heap_data;

const string myautobot = "auto_1"; /// our autobot name from server data
const string opmanualbot = "manual_2"; /// opponent's manualbot name from server data
const int x_bound = 300; /// contest bounds: 300
const int y_bound = 300; /// contest bounds: 300
const int dir_to_storage = -1; /// -1: storage in left side; 1: storage in right side
const int safe_radius = 15; /// safe radius from the center point of the object
const int Yeps = 5; /// Y-coor epsilon to push object
const bool debug_mode = true; /// true to watch progress

coordinates autobot, manualbot; /// auto bot from our team, manual bot from opponent's team
vector <coordinates> object;
bool locked[x_bound + 15][y_bound + 15];
char matrix[x_bound + 15][y_bound + 15];
coordinates trace[x_bound + 15][y_bound + 15];

void get_data() {
    do {
        string s, name = ""; getline(cin, s);
        if (s == "") return;
        int i = 0, x = 0, y = 0;
        for (; s[i] != ' '; name += s[i++]);
        for (i += 2; s[i] != ','; x = x * 10 + (s[i++] - 48));
        for (i += 2; s[i] != ']'; y = y * 10 + (s[i++] - 48));
        if (name == myautobot) autobot = make_pair(x, y);
        else if (name == opmanualbot) manualbot = make_pair(x, y);
        else if (name[0] == 'o') object.push_back(make_pair(x, y));
        getline(cin, s); getline(cin, s);
    } while ("robocon" != "easy");
}

bool pushable(coordinates pos) {
    bool check1 = false, check2 = true;
    for (coordinates obj: object) {
        int dX = obj.first - pos.first, dY = obj.second - pos.second;
        check1 |= dX * dir_to_storage > 0 && abs(dY) <= Yeps && obj.first > safe_radius;
        check2 &= hypot(dX, dY) >= safe_radius;
    }
    return check1 & check2;
}

bool object_clear(coordinates a, coordinates b) {
    for (coordinates obj: object) {
        double AOx = obj.first - a.first;
        double AOy = obj.second - a.second;
        double BOx = obj.first - b.first;
        double BOy = obj.second - b.second;
        double S = AOx * BOy - AOy * BOx; S = max(S, -S);
        double AB = hypot(a.first - b.first, a.second - b.second);
        if (S <= AB * safe_radius) return false;
    }
    return true;
}

double heuristic(int X, int Y) {
    double h = 1500000; /// just random big number
    for (coordinates obj: object)
        h = min(h, X + hypot(X - obj.first, Y - obj.second));
    return h;
}

int sqr(int x) {return x * x;}

void push_object(coordinates pos) {
    cout << safe_radius * 2 << ' ' << pos.second << '\n';
}

void trace_path(coordinates pos) {
    if (pos.first == -1) return;
    trace_path(trace[pos.first][pos.second]);
    cout << pos.first << ' ' << pos.second << '\n';
}

void panic_plan() {
    if (debug_mode) cerr << "PANICING!!!\n";
    cout << autobot.first << ' ' << autobot.second << '\n';
    cout << manualbot.first << ' ' << manualbot.second << '\n';
}

void A_star() {
    priority_queue <heap_data, vector <heap_data>, greater <heap_data>> pos;
    while (!pos.empty()) pos.pop();
    pos.push(make_pair(heuristic(autobot.first, autobot.second), make_pair(autobot, make_pair(-1, -1))));
    memset(locked, false, sizeof locked);
    bool patient = false;
    for (int i = 0; i <= x_bound; ++i)
        for (int j = 0; j <= y_bound; ++j)
            if (pushable(make_pair(i, j))) {
                patient = true; break;
            }
    if (!patient) {panic_plan(); return;}

    if (debug_mode) cerr << "init data done\n";

    while (!pos.empty()) {
        coordinates cur = pos.top().second.first;
        coordinates pre = pos.top().second.second;
        double len = pos.top().first - heuristic(cur.first, cur.second);
        pos.pop();
        if (locked[cur.first][cur.second]) continue;
        locked[cur.first][cur.second] = true;
        trace[cur.first][cur.second] = pre;

        if (debug_mode) cerr << "point " << cur.first << ' ' << cur.second << '\n';

        if (pushable(cur)) {
            if (debug_mode) cerr << "path found\n";
            trace_path(cur); push_object(cur);
            if (debug_mode) cerr << "output done\n";
            return;
        }

        for (int i = 0; i <= x_bound; ++i)
            for (int j = 0; j <= y_bound; ++j)
                if (!locked[i][j] && object_clear(cur, make_pair(i, j)))
                    pos.push(make_pair(len + hypot(i - cur.first, j - cur.second) + heuristic(i, j), make_pair(make_pair(i, j), cur)));
    }
    panic_plan();
}

int main() {
    ios::sync_with_stdio(0); cin.tie(0);
    freopen("data.txt", "r", stdin);
    freopen("path.txt", "w", stdout);
    get_data();
    if (debug_mode) cerr << "get data done\n";
    A_star();
    if (debug_mode) cerr << "A* done\n";
}
