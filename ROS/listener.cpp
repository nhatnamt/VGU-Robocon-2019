#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float32.h"
#include <bits/stdc++.h>
using namespace ros;
using namespace std;
using namespace std_msgs;

typedef pair<float, float> point;
typedef pair <int, int> coordinates;
typedef pair <double, pair <coordinates, coordinates>> heap_data;

const string myautobot = "auto_2"; /// our autobot name from server data
const string opmanualbot = "manual_2"; /// opponent's manualbot name from server data
const int x_bound = 300; /// contest bounds: 300
const int y_bound = 300; /// contest bounds: 300
const int dir_to_storage = -1; /// -1: storage in left side; 1: storage in right side
const int safe_radius = 15; /// safe radius from the center point of the object
const int Yeps = 5; /// Y-coor epsilon to push object
const bool debug_mode = false; /// true to watch progress
int counter = 0;

coordinates autobot, manualbot; /// auto bot from our team, manual bot from opponent's team
vector <coordinates> object;
vector <point> path;
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

int sqr(int x) {return x * x;}

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

int heuristic(int X, int Y) {
    int h = 1e9; /// just random big number
    for (coordinates obj: object)
        h = min(h, sqr(obj.first) + sqr(X - obj.first) + sqr(Y - obj.second));
    return h;
}

void push_object(coordinates pos) {
    cout << safe_radius * 2 << ' ' << pos.second << '\n';
}

void trace_path(coordinates pos) {
    if (pos.first == -1) return;
    trace_path(trace[pos.first][pos.second]);
    path.push_back(point(pos.first*1.0, pos.second*1.0));
}

void panic_plan() {
    if (debug_mode) cerr << "PANICING!!!\n";
    path.push_back(point(autobot.first*1.0, autobot.second*1.0));
    path.push_back(point(manualbot.first*1.0, manualbot.second*1.0));
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

vector<point> smooth (const vector<point> &A, float weight_data, float weight_smooth, float tolerance) {
  // HYPERPARAMETER 
  // weight_data = 0.5
  // weight_smooth = 0.1
  // tolerance = 0.000001
  vector<point> P = A;
  float df = tolerance;
  while (df >= tolerance) {
    df = 0.0;
    for (int i = 1; i < (int)P.size() - 1; ++i) {
      float x, y, yPrev, yNext, ty;
      
      x = A[i].first;
      y = P[i].first, yPrev = P[i - 1].first, yNext = P[i + 1].first;
      ty = y;
      y += weight_data * (x - y) + weight_smooth * (yPrev + yNext - 2.0 * y);
      P[i].first = y;
      df += fabs(ty - y);

      x = A[i].second;
      y = P[i].second, yPrev = P[i - 1].second, yNext = P[i + 1].second;
      ty = y;
      y += weight_data * (x - y) + weight_smooth * (yPrev + yNext - 2.0 * y);
      P[i].second = y;
      df += fabs(ty - y);
    }
  }
  return P;
}

const float eps=1e-7;
struct vec {
    float x, y;
    vec() { x=0.0, y=0.0; }
    vec(float xx, float yy) {
        this->x=xx;
        this->y=yy;
    }
};

vec tovec(const point &A, const point &B) {
    return vec(B.first-A.first, B.second-A.second);
}
float cross(const vec &A, const vec &B) {
    return A.x*B.y-B.x*A.y;
}
float dist(const point &A, const point &B) {
  return sqrt((A.first-B.first)*(A.first-B.first)+(A.second-B.second)*(A.second-B.second));
}
bool collinear(const vec &A, const vec &B) {
    return fabs(cross(A, B))<eps;
}


vector<point> optimize;

void pathReprocessing() {
    int N=(int)path.size();
    int i=0;
    optimize.clear();
    optimize.push_back(path[0]);
    //Eliminate unnecessary points
    while (i<N-1) {
        int j=i+1;
        vec base=tovec(path[i], path[j]);
        while (j+1<N&&collinear(base, tovec(path[i], path[j+1]))) ++j;
        optimize.push_back(path[j]);
        i=j;
    }
    //Smooth new path
    optimize=smooth(optimize, 0.5, 0.1, 0.000001);
}

vec vecDir;

void calculate(const String::ConstPtr& msg)
{
  object.clear();
  path.clear();
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
    if (nameBot == myautobot) { 
        autobot = make_pair(x, y);
        vecDir=tovec(point(autobot.first, autobot.second), point(dx, dy));
      }
      else if (nameBot == opmanualbot) manualbot = make_pair(x, y);
        else if (nameBot[0] == 'o') object.push_back(make_pair(x, y));
  }
}

int main(int argc, char **argv)
{
  
  init(argc, argv, "listener");
  NodeHandle n;

  Subscriber sub = n.subscribe("chatter", 1, calculate);
  Publisher fine_path=n.advertise<Float32>("fine_path", 10);
  
  Rate loop_rate(10);

  while (ok()) {
    A_star();
    pathReprocessing();
    spinOnce();
    
    //Calculate angle error
    int l=0, r=0;
    float minDist=1e9;
    point currentPos=point(autobot.first*1.0, autobot.second*1.0);
    for (int i=0;i<(int)optimize.size()-1;++i) {
      //Cross(OA,OB)/|AB| = dist from O to AB
      float D=fabs(cross(tovec(currentPos, optimize[i]), tovec(currentPos, optimize[i+1])))/dist(optimize[i], optimize[i+1]);
      if (D-minDist<-eps) {
        l=i, r=i+1;
        minDist=D;
      }
    }
    vec alpha=tovec(optimize[l], optimize[r]);
    
    //Publish angle to PID
    Float32 angle;
    angle.data=cross(vecDir, alpha);
    fine_path.publish(angle);
    loop_rate.sleep();
  }
  return 0;
}