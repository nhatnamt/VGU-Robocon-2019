#include "ros/ros.h"
#include "std_msgs/String.h"
#include <bits/stdc++.h>
using namespace ros;
using namespace std;

typedef pair <int, int> coordinates;
typedef pair <double, pair <coordinates, coordinates>> heap_data;

const string myautobot = "auto_1"; /// our autobot name from server data
const string opmanualbot = "manual_2"; /// opponent's manualbot name from server data
const int x_bound = 1000; /// contest bounds: 300
const int y_bound = 1000; /// contest bounds: 300
const int dir_to_storage = -1; /// -1: storage in left side; 1: storage in right side
const int safe_radius = 15; /// safe radius from the center point of the object
const int Yeps = 5; /// Y-coor epsilon to push object
const bool debug_mode = false; /// true to watch progress

coordinates autobot, manualbot; /// auto bot from our team, manual bot from opponent's team
vector <coordinates> object;
bool locked[x_bound + 15][y_bound + 15];
char matrix[x_bound + 15][y_bound + 15];
coordinates trace[x_bound + 15][y_bound + 15];

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
    cerr << safe_radius * 2 << ' ' << pos.second << '\n';
}

void trace_path(coordinates pos) {
    if (pos.first == -1) return;
    trace_path(trace[pos.first][pos.second]);
    cerr << pos.first << ' ' << pos.second << '\n';
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
            trace_path(cur); 
            push_object(cur);
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

void calculate(const std_msgs::String::ConstPtr& msg)
{
  object.clear();
  string test = msg->data;
  string buffer = "";
  for (int i = 0; i < (int)test.size(); ++i) {
    if (test[i] == '{' || test[i] == '"' || test[i] == '['
      || test[i] == ':' || test[i] == ',' || test[i] == '}'
      || test[i] == ']') continue;
    buffer += test[i];
  }
  stringstream input(buffer);
  string dim, name, nameBot, pos;
  int dx, dy, x, y;
  for (int i = 0; i < 3; ++i) input >> name;
  for (int i = 0; i < 28; ++i) {
    input >> dim >> dx >> dy >> name >> nameBot >> pos >> x >> y;
    //cerr << "Dimension: " << dx << ' ' << dy << '\n';
    //cerr << "Name: " << nameBot << '\n';
    //cerr << "Position: " << x << ' ' << y << '\n';
    //cerr << '\n';
    x = min(300, x);
    y = min(300, y);
    if (nameBot == myautobot) autobot = make_pair(x, y);
      else if (nameBot == opmanualbot) manualbot = make_pair(x, y);
        else if (nameBot[0] == 'o') object.push_back(make_pair(x, y));
  }
  A_star();
}

int main(int argc, char **argv)
{
  init(argc, argv, "listener");
  NodeHandle n;
  Subscriber sub = n.subscribe("chatter", 3, calculate);
  spin();
  return 0;
}

//{"time": 1557486887879241501, "data": 
//[{"dimension": [-20, -30], "name": "manual_1", "position": [1190, 292]},
// {"dimension": [-39, -49], "name": "auto_1", "position": [470, 992]}, 
// {"dimension": [26, 0], "name": "manual_2", "position": [190, 842]}, 
// {"dimension": [15, -20], "name": "auto_2", "position": [305, 306]}, 
// {"dimension": [-36, -70], "name": "object_1", "position": [571, 1353]}, 
// {"dimension": [51, 41], "name": "object_2", "position": [1173, 686]}, 
// {"dimension": [-13, 24], "name": "object_3", "position": [986, 457]}, 
// {"dimension": [26, 6], "name": "object_4", "position": [1131, 84]}, 
// {"dimension": [25, -69], "name": "object_5", "position": [109, 609]}, 
// {"dimension": [32, -3], "name": "object_6", "position": [1150, 1226]}, 
// {"dimension": [-9, 6], "name": "object_7", "position": [601, 738]}, 
// {"dimension": [-29, 60], "name": "object_8", "position": [346, 494]}, 
// {"dimension": [-37, 21], "name": "object_9", "position": [742, 1227]}, 
// {"dimension": [-41, 21], "name": "object_10", "position": [259, 795]}, 
// {"dimension": [87, -62], "name": "object_11", "position": [312, 170]}, 
// {"dimension": [26, 42], "name": "object_12", "position": [66, 244]}, 
// {"dimension": [57, -6], "name": "object_13", "position": [886, 985]}, 
// {"dimension": [-79, -83], "name": "object_14", "position": [1364, 291]}, 
// {"dimension": [-28, -59], "name": "object_15", "position": [1128, 1021]}, 
// {"dimension": [-83, -46], "name": "object_16", "position": [160, 902]}, 
// {"dimension": [-68, -60], "name": "object_17", "position": [1056, 523]}, 
// {"dimension": [1, -69], "name": "object_18", "position": [312, 973]}, 
// {"dimension": [30, 29], "name": "object_19", "position": [439, 408]}, 
// {"dimension": [71, 38], "name": "object_20", "position": [801, 212]}, 
// {"dimension": [-68, -48], "name": "object_21", "position": [683, 47]}, 
// {"dimension": [58, 73], "name": "object_22", "position": [318, 279]}, 
// {"dimension": [-52, 84], "name": "object_23", "position": [884, 602]},
// {"dimension": [-6, -83], "name": "object_24", "position": [1072, 534]}]}