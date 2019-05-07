#include <bits/stdc++.h>
using namespace std;
typedef pair<float, float> point;

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

int main() {
}