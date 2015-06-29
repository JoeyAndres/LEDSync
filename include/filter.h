//
// Created by joey on 28/06/15.
//

#ifndef LEDSYNC_RCFILTER_H
#define LEDSYNC_RCFILTER_H


#include <SFML/Audio.hpp>
#include <vector>

template <typename T,
        template <typename D, typename Allocator = std::allocator<D>>
        class C = std::vector>
C<T> low_pass(const C<T> &input, double dt, double rc) {
    C<T> output(input.size());
    double step_size = dt/(rc + dt);

    output[0] = input[0];
    for (size_t i = 1; i < input.size(); i++) {
        output[i] = output[i-1] + step_size*(input[i] - output[i-1]);
    }

    return output;
}

template <typename T,
        template <typename D, typename Allocator = std::allocator<D>>
        class C = std::vector>
C<T> high_pass(const C<T> &input, double dt, double rc) {
    C<T> output(input.size());
    double step_size = dt/(rc + dt);

    output[0] = input[0];
    for (size_t i = 1; i < input.size(); i++) {
        output[i] = step_size*(output[i-1] + input[i] - input[i-1]);
    }

    return output;
}



#endif //LEDSYNC_RCFILTER_H
