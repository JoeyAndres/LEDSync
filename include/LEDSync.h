/**
 * LEDSync.h
 */
#include <SFML/Audio.hpp>
#include <cstdint>
#include <vector>
#include <string>
#include <cmath>
#include <iostream>

inline long map(long x, long in_min, long in_max, long out_min, long out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

inline void plot(long averageAmplitude, long maxBarVal = 80) {
    long val = std::abs(map(averageAmplitude, -32768, 32767, -maxBarVal, maxBarVal));
    std::string output = "";
    for (size_t i = 0; i < val; i++) {
        output += "#";
    }
    std::cout << output << std::endl;
}

class LEDSync {
public:
    LEDSync(const std::string &audioFile);

    std::vector<int16_t> getSampledAmplitude(uint16_t ms, const sf::SoundBuffer &sb) const;

    void play(uint16_t ms, bool visualize = false, int maxBarVal = 80) const;

protected:
    sf::SoundBuffer _buffer;
    mutable sf::Sound _sound;
};
