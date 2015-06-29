/**
 * LEDSync.cpp
 */
#include <exception>
#include <cstring>

//#include <errno.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include "LEDSync.h"
#include "RCFilter.h"

int set_interface_attribs(int fd, int speed, int parity) {
    struct termios tty;
    memset(&tty, 0, sizeof tty);
    if (tcgetattr(fd, &tty) != 0) {
        //error_message ("error %d from tcgetattr", errno);
        return -1;
    }

    cfsetospeed(&tty, speed);
    cfsetispeed(&tty, speed);

    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;     // 8-bit chars
    // disable IGNBRK for mismatched speed tests; otherwise receive break
    // as \000 chars
    tty.c_iflag &= ~IGNBRK;         // disable break processing
    tty.c_lflag = 0;                // no signaling chars, no echo,
    // no canonical processing
    tty.c_oflag = 0;                // no remapping, no delays
    tty.c_cc[VMIN] = 0;            // read doesn't block
    tty.c_cc[VTIME] = 5;            // 0.5 seconds read timeout

    tty.c_iflag &= ~(IXON | IXOFF | IXANY); // shut off xon/xoff ctrl

    tty.c_cflag |= (CLOCAL | CREAD);// ignore modem controls,
    // enable reading
    tty.c_cflag &= ~(PARENB | PARODD);      // shut off parity
    tty.c_cflag |= parity;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CRTSCTS;

    if (tcsetattr(fd, TCSANOW, &tty) != 0) {
        //error_message ("error %d from tcsetattr", errno);
        return -1;
    }
    return 0;
}

void set_blocking(int fd, int should_block) {
    struct termios tty;
    memset(&tty, 0, sizeof tty);
    if (tcgetattr(fd, &tty) != 0) {
        //error_message ("error %d from tggetattr", errno);
        return;
    }

    tty.c_cc[VMIN] = should_block ? 1 : 0;
    tty.c_cc[VTIME] = 5;            // 0.5 seconds read timeout

    if (tcsetattr(fd, TCSANOW, &tty) != 0) { }
    //error_message ("error %d setting term attributes", errno);
}


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
    const char *portname = "/dev/ttyACM1";
    int fd = open(portname, O_RDWR | O_NOCTTY | O_SYNC);
    if (fd < 0) {
        //error_message ("error %d opening %s: %s", errno, portname, strerror (errno));
        return;
    }

    set_interface_attribs(fd, B115200, 0);  // set speed to 115,200 bps, 8n1 (no parity)
    set_blocking(fd, 0);                // set no blocking
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
        write(fd, data, 6);           // send 7 character greeting

        lastIndex = sampleIndex;

        usleep((6 + 25) * 100);             // sleep enough to transmit the 6 plus
    }
}
