// UDP Client Server -- send/receive UDP packets
// Copyright (C) 2013  Made to Order Software Corp.
//
// This program is free software; you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation; either version 2 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program; if not, write to the Free Software
// Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
#ifndef UDPUTILS_HH
#define UDPUTILS_HH

#include <sys/types.h>
#include <sys/socket.h>
#include <netdb.h>
#include <stdexcept>

namespace UdpClientServer
{

    class UdpClientServerRuntimeError : public std::runtime_error
    {
    public:
        UdpClientServerRuntimeError(const char *w) : std::runtime_error(w) {}
    };

    class UdpClient
    {
    public:
        UdpClient(const std::string &addr, int port);
        ~UdpClient();

        int get_socket() const;
        int get_port() const;
        std::string get_addr() const;

        int send(const void *msg, size_t size);

    private:
        int f_socket;
        int f_port;
        std::string f_addr;
        struct addrinfo *f_addrinfo;
    };

    class UdpServer
    {
    public:
        UdpServer(const std::string &addr, int port);
        ~UdpServer();

        int get_socket() const;
        int get_port() const;
        std::string get_addr() const;

        int recv(void *msg, size_t max_size);
        int timed_recv(void *msg, size_t max_size, int max_wait_ms);

    private:
        int f_socket;
        int f_port;
        std::string f_addr;
        struct addrinfo *f_addrinfo;
    };

} // namespace UdpClientServer

#endif
// UDPUTILS_HH
// vim: ts=4 sw=4 et