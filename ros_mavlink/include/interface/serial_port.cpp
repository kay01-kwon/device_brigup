#include "serial_port.h"

SerialPort::SerialPort(const string &port_name, int baudrate)
: port_name_(port_name), baudrate_(baudrate)
{

}

SerialPort::~SerialPort()
{
    serial_.close();
    std::cout << "Serial port closed" << std::endl;
}

int SerialPort::read_message(mavlink_message_t *message)
{
    uint8_t cp;
    mavlink_status_t status;
    uint8_t msgReceived = false;

    // int result = 0;

    if(serial_.available()>0)
    {
        serial_.read(&cp, 1);
        msgReceived = mavlink_parse_char(MAVLINK_COMM_0, 
                                         cp, 
                                         message, 
                                         &status);
    }

    return msgReceived;
}

int SerialPort::write_message(const mavlink_message_t *message)
{
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];

    uint16_t len = 
    mavlink_msg_to_send_buffer(buf,
                               message);

    size_t bytes_sent;

    bytes_sent = serial_.write(buf, len);


    return bytes_sent;
}

void SerialPort::start()
{
    try{
        serial_.setPort(port_name_);
        serial_.setBaudrate(baudrate_);
        serial::Timeout timeout = serial::Timeout::simpleTimeout(1000);
        serial_.setTimeout(timeout);
    }
    catch(serial::IOException& e)
    {
        std::cout << "ERROR: Could not open serial port" 
        << std::endl;
    }

    serial_.open();

}
