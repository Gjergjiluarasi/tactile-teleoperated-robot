#define USE_TERMIOS 1

#if USE_TERMIOS

#include <iostream>
#include <fstream>
#include <string>
#include <cstring>
#include <termios.h> // for serial port configuration

using namespace std;

int main() {
    string usb_port = "/dev/ttyUSB0"; // Replace with the actual USB port name
    int baudrate = B115200; // Set the baudrate to 115200
    ifstream serial(usb_port.c_str()); // Open the USB port
    if (!serial.is_open()) { // Check if the port is opened successfully
        cerr << "Failed to open " << usb_port << endl;
        return 1;
    }

    // Set the baudrate
    termios options;
    tcgetattr(serial.fd(), &options);
    cfsetispeed(&options, baudrate);
    cfsetospeed(&options, baudrate);
    tcsetattr(serial.fd(), TCSANOW, &options);

    char buffer[1024]; // Buffer for storing the received data
    memset(buffer, 0, sizeof(buffer)); // Initialize the buffer

    while (true) {
        serial.read(buffer, sizeof(buffer)); // Read data from the USB port
        cout << buffer; // Print the received data to the console
        memset(buffer, 0, sizeof(buffer)); // Clear the buffer
    }

    serial.close(); // Close the USB port
    return 0;
}

#else

#include <iostream>
#include <fstream>
#include <string>
#include <cstring>

using namespace std;

int main() {
    string usb_port = "/dev/ttyUSB0"; // Replace with the actual USB port name
    ifstream serial(usb_port.c_str()); // Open the USB port
    if (!serial.is_open()) { // Check if the port is opened successfully
        cerr << "Failed to open " << usb_port << endl;
        return 1;
    }

    char buffer[1024]; // Buffer for storing the received data
    memset(buffer, 0, sizeof(buffer)); // Initialize the buffer

    while (true) {
        serial.read(buffer, sizeof(buffer)); // Read data from the USB port
        cout << buffer; // Print the received data to the console
        memset(buffer, 0, sizeof(buffer)); // Clear the buffer
    }

    serial.close(); // Close the USB port
    return 0;
}

#endif
