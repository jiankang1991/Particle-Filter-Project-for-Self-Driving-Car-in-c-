#include <vector>
#include <iostream>
#include <numeric>
#include <algorithm>

using std::vector;
using std::iota;
using std::sort;

template <typename T>
vector<size_t> sort_indexes(const vector<T> &v) {

  // initialize original index locations
  vector<size_t> idx(v.size());
  iota(idx.begin(), idx.end(), 0);

  // sort indexes based on comparing values in v
  sort(idx.begin(), idx.end(),
       [&v](size_t i1, size_t i2) {return v[i1] < v[i2];});

  return idx;
}


int main(int argc, char const *argv[])
{
    /* code */

    vector<double> vec{1.12, 3.45, 8.0, 0.9, 1.0};

    vector<size_t> sort_idx;

    sort_idx = sort_indexes(vec);

    for (int i=0; i<sort_idx.size(); i++) {

        std::cout << vec[sort_idx[i]] << std::endl;
    }


    return 0;
}

















