#include <unistd.h>
#include <iostream>
#include <boost/asio.hpp>
#include <boost/asio/serial_port.hpp>
#include <boost/system/error_code.hpp>
#include <boost/system/system_error.hpp>
#include <boost/bind.hpp>
#include <boost/thread.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/thread/thread.hpp>
#include <thread>
#include <boost/asio/serial_port_base.hpp>
#include <boost/algorithm/string.hpp>
#include <iomanip>
#include <iostream>
#include <fstream>
#include <curses.h>

// Compile usign -l -lboost_system

using namespace std;

class SerialIO
{
  private:
    boost::asio::io_service io_svc;
    boost::asio::serial_port m_port;

  public:
    SerialIO(std::string portDir) : m_port(io_svc, portDir) {}

    void set_baud(int b_rate)
    {
        m_port.set_option(boost::asio::serial_port_base::baud_rate(b_rate));
    }

    void write(string input)
    {
        const int BUFFSIZE = 200;
        boost::array<char, BUFFSIZE> buf;

        if (sizeof(input) > 0)
        {
            for (int i = 0; i < sizeof(input) / 8; i++)
            {
                buf[i] = input[i];
            }
            buf[sizeof(input) / 8] = 0;
            m_port.write_some(boost::asio::buffer(buf, BUFFSIZE));
        }
    }

    void send_commands()
    {
        initscr();   // Opens new window to get commands
        timeout(-1); // Never close window

        char a;

        // Get automatically one char and write it to serial
        while ((a = getch()) != '\n')
        {
            string str(1, a);
            write(str);
        }
    }
};

int main()
{
    SerialIO s("/dev/cu.usbmodem1441");

    s.set_baud(115200);

    s.send_commands();

    return 0;
}

// Compile:

// g++ src/pc/rx/serial_boost.cpp -o serial_boost -lboost_system -lcurses

// Run:

// ./serial_boost