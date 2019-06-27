
#ifndef STATISTICS_HPP_
#define STATISTICS_HPP_

// STL
#include <algorithm>
#include <cmath>
#include <cstddef>
#include <vector>

#include "macros.h"

template<typename T>
T vector_median(const std::vector<T>& v) {
    const std::size_t N = v.size();
    RUNTIME_ASSERT(N > 1);

    if (N == 1) return v[0];

    const std::size_t m = v.size() / 2;

    std::vector<T> v_ = v;
    std::nth_element(v_.begin(), v_.begin() + m, v_.end());

    if (N & 1)
        return v[m];
    else {
        const T elem1 = *std::max_element(v_.begin(), v_.begin() + m); 
        const T elem2 = v[m];
        return (elem1 + elem2) / static_cast<T>(2.0);
    }
}

template<typename T>
T vector_mad(const std::vector<T>& v) {
    const std::size_t N = v.size();
    RUNTIME_ASSERT(N > 1);

    const T m = vector_median(v);

    std::vector<T> abs_dev(N);
    for (std::size_t i = 0; i < N; ++i) {
        abs_dev[i] = std::abs(v[i] - m);
    }

    return vector_median(abs_dev);
}

template<typename T>
T vector_mean(const std::vector<T>& v) {
    RUNTIME_ASSERT(v.size() > 1);

    T mean = 0.0;
    for (const T& elem : v)
        mean += elem;
    return mean / static_cast<T>(v.size());
}

template<typename T>
T vector_rms(const std::vector<T>& v) {
    std::vector<T> v2(v.size());
    for (std::size_t i = 0; i < v.size(); ++i) {
        v2[i] = v[i] * v[i];
    }

    const T mean = vector_mean(v2);
    return std::sqrt(mean);
}

template<typename T>
T vector_stdv(const std::vector<T>& v) {
    RUNTIME_ASSERT(v.size() > 1);

    T s = 0.0;
    const T mean = vector_mean(v);
    for (const T& elem : v)
        s += (elem - mean)*(elem - mean);
    return std::sqrt(s / static_cast<T>(v.size()));
}

template<typename T>
T vector_stdv(const std::vector<T>& v, const T mean) {
    RUNTIME_ASSERT(v.size() > 1);

    T s = 0.0;
    for (const T& elem : v)
        s += (elem - mean)*(elem - mean);
    return std::sqrt(s / static_cast<T>(v.size()));
}

#endif // STATISTICS_HPP_
