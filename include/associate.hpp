
#ifndef ASSOCIATE_HPP_
#define ASSOCIATE_HPP_

// STL
#include <algorithm>
#include <cstddef>
#include <iterator>
#include <limits>
#include <vector>
#include <utility>

#include "macros.h"

template<class A, typename T = double>
struct Timestamp_comparator {
    bool operator()(const A& lhs, const T& rhs) {
        return lhs.timestamp < rhs;
    }
};

template<typename T>
T abs_diff(const T a, const T b) {
    if (a < b) return (b - a);
    else return (a - b);
}

template<class A, typename T = double, class Comparator = Timestamp_comparator<A, T>>
typename std::vector<A>::const_iterator find_closest(const std::vector<A>& v, const T& value) {
    if (v.empty()) return v.end(); 

    typename std::vector<A>::const_iterator v_cit_next = std::lower_bound(v.begin(), v.end(), value, Comparator());
    if (v_cit_next == v.end() && v_cit_next != v.begin()) return std::prev(v_cit_next);

    typename std::vector<A>::const_iterator v_cit;
    if (v_cit_next != v.begin()) {
        typename std::vector<A>::const_iterator v_cit_prev = std::prev(v_cit_next);

        v_cit = (abs_diff<T>(value, v_cit_next->timestamp)
                  <
                 abs_diff<T>(value, v_cit_prev->timestamp))
                  ? v_cit_next : v_cit_prev;
    } else
        v_cit = v_cit_next;

    return v_cit;
}

template<class A, class B, typename T = double, class Comparator = Timestamp_comparator<B, T>>
std::vector<std::pair<std::size_t, std::size_t>> associate(const std::vector<A>& u, const std::vector<B>& v, const T max_diff) {

    RUNTIME_ASSERT(max_diff > 0.0);

    std::vector<T> errors(v.size(), std::numeric_limits<T>::max());
    std::vector<std::size_t> associations(v.size(), std::numeric_limits<std::size_t>::max());
    for (typename std::vector<A>::const_iterator u_cit = u.begin(); u_cit != u.end(); ++u_cit) {
/*
        typename std::vector<B>::const_iterator v_cit_next = std::lower_bound(v.begin(), v.end(), u_cit->timestamp, Comparator());
        if (v_cit_next == v.end()) continue;

        typename std::vector<B>::const_iterator v_cit;
        if (v_cit_next != v.begin()) {
            typename std::vector<B>::const_iterator v_cit_prev = std::prev(v_cit_next);

            v_cit = (abs_diff<T>(u_cit->timestamp, v_cit_next->timestamp)
                      <
                     abs_diff<T>(u_cit->timestamp, v_cit_prev->timestamp))
                      ? v_cit_next : v_cit_prev;
        } else
            v_cit = v_cit_next;
*/
        typename std::vector<B>::const_iterator v_cit = find_closest<B, T, Comparator>(v, u_cit->timestamp);
        if (v_cit == v.end()) continue;

        const T error = abs_diff<T>(u_cit->timestamp, v_cit->timestamp);
        if (error > max_diff)
            continue;

        std::size_t i = std::distance(u.begin(), u_cit),
                    j = std::distance(v.begin(), v_cit);

        if (error < errors.at(j)) {
            associations.at(j) = i;
            errors.at(j) = error;
        }
    }

    std::vector<std::pair<std::size_t, std::size_t>> pairs;
    for (std::size_t j = 0; j < associations.size(); ++j) {
        const std::size_t i = associations.at(j);
        if (i != std::numeric_limits<std::size_t>::max()) pairs.emplace_back(std::make_pair(i, j));
    }

    return pairs;
}

#endif // ASSOCIATE_HPP_
