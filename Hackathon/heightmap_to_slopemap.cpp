#include <bits/stdc++.h>
using namespace std;

int main() {
    freopen("input_heights.txt", "r", stdin);
    freopen("slopemap.txt", "w", stdout);

    int n, m;
    cin >> n >> m;

    vector<vector<double>> h(n, vector<double>(m));
    vector<vector<double>> slope(n, vector<double>(m, 0));

    for (int i = 0; i < n; i++) {
        for (int j = 0; j < m; j++) {
            cin >> h[i][j];
        }
    }

    for (int i = 0; i < n; i++) {
        for (int j = 0; j < m; j++) {
            double max_diff = 0;
            if (i > 0) max_diff = max(max_diff, fabs(h[i][j] - h[i - 1][j]));
            if (i < n - 1) max_diff = max(max_diff, fabs(h[i][j] - h[i + 1][j]));
            if (j > 0) max_diff = max(max_diff, fabs(h[i][j] - h[i][j - 1]));
            if (j < m - 1) max_diff = max(max_diff, fabs(h[i][j] - h[i][j + 1]));
            slope[i][j] = max_diff;
        }
    }

    cout << n << " " << m << endl;
    for (int i = 0; i < n; i++) {
        for (int j = 0; j < m; j++) {
            cout << (int)round(slope[i][j]) << " ";
        }
        cout << "\n";
    }

    return 0;
}