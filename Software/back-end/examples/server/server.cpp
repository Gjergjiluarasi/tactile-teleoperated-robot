#include "server.hh"

using namespace std;

bool serverExample()
{
    UdpClientServer::UdpServer server(IP, PORT);
    double message[6];
    cout << "Starting server " << server.get_addr() << ':' << server.get_port() << endl;
    auto startlooptime = chrono::system_clock::now();

    while (true)
    {
        startlooptime = chrono::system_clock::now();

        if (server.recv(message, MAX_SIZE))
        {
            cout << "0: " << message[0] << ", 1: " << message[1]
                 << "2: " << message[2] << ", 3: " << message[3]
                 << ", 4: " << message[4] << ", 5: " << message[5] << "\n";
        }
        this_thread::sleep_until(startlooptime + chrono::microseconds(1000));
    }

    return true;
}