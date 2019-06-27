
#ifndef DECIMATE_HPP_
#define DECIMATE_HPP_

// STL
#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>

template<class Container>
Container decimate_sequence(const Container& full_sequence, std::size_t offset = 0, std::size_t step = 0, std::size_t nmax = 0) {

    if (nmax == 0)
        nmax = std::numeric_limits<std::size_t>::max();

    if (offset >= full_sequence.size())
        return Container();

    std::size_t n = std::min(nmax, static_cast<std::size_t>(std::ceil(static_cast<double>(full_sequence.size() - offset) / static_cast<double>(step + 1))));

    Container decimated_sequence(n);

    std::size_t i = offset;
    for (std::size_t ctr = 0; ctr < n; ++ctr, i += (step + 1))
        decimated_sequence.at(ctr) = full_sequence.at(i);

    return decimated_sequence;
}

#endif // DECIMATE_HPP_
