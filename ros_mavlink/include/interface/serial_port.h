#ifndef SERIAL_PORT_H
#define SERIAL_PORT_H

#include <serial/serial.h>
#include <iostream>
#include "mavlink_messages.h"

using std::string;

using serial::Serial;

class SerialPort
{
    friend class serial::Serial;
    public:

    SerialPort() = delete;

    SerialPort(const string &port_name, int baudrate);

    ~SerialPort();

    int read_message(mavlink_message_t *message);

    int write_message(const mavlink_message_t *message);

    void start();

    private:

    int fd_;
    mavlink_status_t last_status_;

    string port_name_;
    int baudrate_;

    Serial serial_;

};


#endif // SERIAL_PORT_H