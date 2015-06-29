/**
 * LEDSync.cpp
 */
#include <exception>

#include <unistd.h>

#include "LEDSync.h"
#include "filter.h"
#include "Serial.h"

LEDSync::LEDSync(const std::string &audioFile) {
    if (!_buffer.loadFromFile(audioFile))
        throw "Loading " + audioFile + " failed.";

    _sound.setBuffer(_buffer);
}

std::vector<int16_t> LEDSync::getSampledAmplitude(uint16_t ms, const sf::SoundBuffer &sb) const {
    std::vector<int16_t> ampSampling;
    double ampAcc = 0;
    uint16_t originalSamplingDivisor = (ms / 1000.0F) * sb.getSampleRate();
    for (size_t i = 0; i < sb.getSampleCount(); i++) {

        ampAcc += *(sb.getSamples() + i);
        if (i > 0 && (i % originalSamplingDivisor == 0)) {
            ampSampling.push_back(ampAcc / originalSamplingDivisor);
            ampAcc = 0.0f;
        }
    }

    return ampSampling;
}

void LEDSync::play(uint16_t ms, bool visualize, int maxBarVal) const {
    auto sampleAmplitude = this->getSampledAmplitude(ms, _buffer);
    uint16_t originalSamplingDivisor = (ms / 1000.0F) * _buffer.getSampleRate();

    auto acousticVector = low_pass(std::vector<sf::Int16>(
            _buffer.getSamples(),
            _buffer.getSamples() + _buffer.getSampleCount()), 0.1f, 0.001f);
    acousticVector = high_pass(acousticVector, 0.1, 0.002f);
    sf::SoundBuffer acousticBuffer;
    acousticBuffer.loadFromSamples(
            &acousticVector[0], acousticVector.size(),
            _buffer.getChannelCount(), _buffer.getSampleRate());
    auto baseAmplitude = this->getSampledAmplitude(ms, acousticBuffer);

    _sound.play();
    size_t lastIndex = 0;

    // TEST
    Serial serial("/dev/ttyACM0");
    // END TEST

    while (_sound.getStatus() == sf::SoundSource::Status::Playing) {
        if (_sound.getPlayingOffset().asMilliseconds() <= 0.0f) { continue; }

        size_t sampleIndex =
                (static_cast<double>(_sound.getPlayingOffset().asMilliseconds()) /
                 static_cast<double>(_buffer.getDuration().asMilliseconds())) *
                (_buffer.getSampleCount() / originalSamplingDivisor);
        if (lastIndex == sampleIndex) continue;
        if (visualize) plot(sampleAmplitude[sampleIndex], maxBarVal);

        uint8_t drum_val = std::abs(map(std::abs(baseAmplitude[sampleIndex]), 0, 32767, 0, 255));
        uint8_t normal_val = std::abs(map(std::abs(sampleAmplitude[sampleIndex]), 0, 32767, 0, 255));
        //std::cout << (int) drum_val << std::endl;

        uint8_t data[6] = {
                0,
                0,
                0,
                drum_val,
                drum_val,
                drum_val
        };
        data[sampleIndex % 3] = (char) normal_val;
        serial.write(std::vector<uint8_t>(data, data + sizeof(data)));

        lastIndex = sampleIndex;
    }
}
