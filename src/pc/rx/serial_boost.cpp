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

// Compile usign -l -lboost_system

using namespace std;

class SerialIO
{
  private:
    boost::asio::io_service io_svc;
    boost::asio::serial_port m_port;
    float x;
    float y;

  public:
    SerialIO(std::string portDir) : m_port(io_svc, portDir)
    {
        x = 0;
        y = 0;
    }

    void set_baud(int b_rate)
    {
        m_port.set_option(boost::asio::serial_port_base::baud_rate(b_rate));
    }

    void write(string input)
    {
        const int BUFFSIZE = 100;
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

    void read()
    {
        const int BUFFSIZE = 100;
        char read_msg_[BUFFSIZE];
        int read_size = 0;
        stringstream ss;

        while (read_msg_[read_size - 1] != '\n')
        {
            read_msg_[read_size] = 0;
            ss << read_msg_;
            read_size = m_port.read_some(boost::asio::buffer(read_msg_, BUFFSIZE));
        }
        // cout << ss.str();

        istringstream iss(ss.str());

        iss >> x >> y;
        cout << "x = " << fixed << setprecision(3) << x << ", y = " << fixed << setprecision(3) << y << endl;
    }

    void write_file()
    {
        ofstream out_file("example.txt", fstream::in | fstream::out | fstream::app);
        if (out_file.is_open())
        {
            out_file << fixed << setprecision(10) << x << " " << fixed << setprecision(10) << y << endl;
            cout << "line written" << endl;
        }
        else
            cout << "Unable to open file";
    }

};

int main()
{
    SerialIO s("/dev/cu.usbmodem1441");

    s.set_baud(115200);

    while (true)
    {
        // string str = "ON";
        // string str = "OFF";
        // string str;
        // cout << "Enter input: ";
        // cin >> str;
        // s.write(str);
        s.read();
        // s.write_file();
    }

    return 0;
}

// Compile:

// g++ /Users/lorenzo/Documents/Documents/UniBG/Terzo\ anno/Tesi/Bruga/Serial/serial/serial_boost.cpp -o ser -lboost_system

// Run:

// ./ser