#include "client.hh"

using namespace std;

bool clientExample()
{
    UdpClientServer::UdpClient client(IP, PORT);
    double message[6];
    cout << "Starting client " << client.get_addr() << ':' << client.get_port() << endl;
    auto startlooptime = chrono::system_clock::now();

    while (true)
    {
        startlooptime = chrono::system_clock::now();
        message[0] = (float)(((int)message[0] + 1) % 100);
        message[1] = (float)(((int)message[1] + 2) % 100);
        message[2] = (float)(((int)message[2] + 3) % 100);
        message[3] = (float)(((int)message[3] + 4) % 100);
        message[4] = (float)(((int)message[4] + 5) % 100);
        message[5] = (float)(((int)message[5] + 6) % 100);
        // cout << "0: " << message[0] << ", 1: " << message[1]
        //      << "2: " << message[2] << ", 3: " << message[3]
        //      << ", 4: " << message[4] << ", 5: " << message[5] << "\n";
        client.send(message, MAX_SIZE);
        this_thread::sleep_until(startlooptime + chrono::microseconds(1000));
    }

    return true;
}