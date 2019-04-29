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
    float x;
    float y;
    float imp_l;
    float imp_r;
    float theta;

  public:
    SerialIO(std::string portDir) : m_port(io_svc, portDir)
    {
        x = 0;
        y = 0;
        imp_l = 0;
        imp_r = 0;
        theta = 0;
    }

    void set_baud(int b_rate)
    {
        m_port.set_option(boost::asio::serial_port_base::baud_rate(b_rate));
    }

    void read_coordinates()
    {
        const int BUFFSIZE = 200;
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

    void read_impulses()
    {
        const int BUFFSIZE = 200;
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
        cout << "left: " << x << ", right: " << y << endl;
    }

    void read_all()
    {
        const int BUFFSIZE = 200;
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

        iss >> imp_r >> imp_l >> x >> y >> theta;
        cout << "imp right: " << imp_r << ", imp left: " << imp_l << ", x: " << fixed << setprecision(3) << x << ", y: " << fixed << setprecision(3) << y << ", theta: " << fixed << setprecision(3) << theta << endl;
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
        s.read_all();
        // s.read_coordinates();
        // s.read_impulses();
    }

    return 0;
}

// Compile:

// g++ g++ src/pc/rx/serial_boost.cpp -o serial_boost -lboost_system -lcurses

// Run:

// ./serial_boost