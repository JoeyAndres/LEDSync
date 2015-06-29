//
// Created by joey on 29/06/15.
//

#ifndef LEDSYNC_SERIAL_H
#define LEDSYNC_SERIAL_H

#include <string>
#include <cstdint>
#include <vector>

#include <termios.h>

class Serial {
public:
    Serial(const std::string& port_name);
    Serial(const std::string& port_name, int baud);
    void write(const std::vector<uint8_t>& data);

private:
    int set_interface_attribs(int fd, int speed, int parity);
    void set_blocking(int fd, int should_block);
private:
    std::string m_port_name;
    int m_fd;
    int m_baud = B115200;
};

#endif //LEDSYNC_SERIAL_H
