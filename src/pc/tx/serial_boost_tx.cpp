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

#define WHEELS_DISTANCE_MM 335
#define WHEELS_PERIMETER_MM 470
#define CPR 40000
#define COMPUTATION_PERIOD 80 // im milliseconds
#define IMPULSE_PER_PERIOD_ONE_SECOND (CPR / 1000 * COMPUTATION_PERIOD)
#define M_PER_S_TO_IMPULSES_IN_PERIOD (1000 * IMPULSE_PER_PERIOD_ONE_SECOND / WHEELS_PERIMETER_MM)

// Compile usign -l -lboost_system

using namespace std;

class SerialIO
{
  private:
    boost::asio::io_service io_svc;
    boost::asio::serial_port m_port;
    int left_sp;  // Left set-point
    int right_sp; // Right set-point

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

    void get_keyboard_command()
    {
        initscr();   // Opens new window to get commands
        timeout(-1); // Never close window
        char command;
        while ((command = getch()) != '\n')
        {
            switch (command)
            {
            case 'W':
            case 'w':
                speed_to_sp(0.1, 0);
                break;
            case 'S':
            case 's':
                speed_to_sp(-0.1, 0);
                break;
            case 'R':
            case 'r':
                speed_to_sp(0, 0.1);
                break;
            default:
                speed_to_sp(0, 0);
                break;
            }
        }
    }

    // linear_speed [m/s], angular_speed [rad/s]
    void speed_to_sp(double linear_speed, double angular_speed)
    {
        double l_linear_speed = linear_speed - angular_speed * WHEELS_DISTANCE_MM / 2;
        double r_linear_speed = linear_speed + angular_speed * WHEELS_DISTANCE_MM / 2;

        left_sp = l_linear_speed * M_PER_S_TO_IMPULSES_IN_PERIOD;
        right_sp = r_linear_speed * M_PER_S_TO_IMPULSES_IN_PERIOD;

        string str;
        str += boost::lexical_cast<string>(left_sp);
        str += " ";
        str += boost::lexical_cast<string>(right_sp);

        cout << str << endl;

        write(str);
    }
};

int main()
{
    SerialIO s("/dev/cu.usbmodem1431");

    s.set_baud(115200);

    s.send_commands();
    // s.get_keyboard_command();

    return 0;
}

// Compile:

// g++ src/pc/rx/serial_boost.cpp -o serial_boost -lboost_system -lcurses

// Run:

// ./serial_boost