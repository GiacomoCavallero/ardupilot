#include <AP_HAL/AP_HAL.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_LINUX

#include <stdio.h>
#include <unistd.h>
#include <errno.h>
#include <stdlib.h>

#include "TCPClientDevice.h"

extern const AP_HAL::HAL& hal;

TCPClientDevice::TCPClientDevice(const char *ip, uint16_t port):
    _ip(ip),
    _port(port),
    _reopened(0)
{
}

TCPClientDevice::~TCPClientDevice()
{
    if (sock != NULL) {
        delete sock;
        sock = NULL;
    }
}

ssize_t TCPClientDevice::write(const uint8_t *buf, uint16_t n)
{
    if (sock == NULL) {
        return -1;
    }
    return sock->send(buf, n);
}

/*
  when we try to read we accept new connections if one isn't already
  established
 */
ssize_t TCPClientDevice::read(uint8_t *buf, uint16_t n)
{
    uint32_t mnow = AP_HAL::millis();
    if (sock == NULL) {
        if ((mnow - _reopened) > 5000) {
            open();
            _reopened = mnow;
        }
        return -1;
    }

    //FIXME: Why are we waiting 5 seconds after reopening the connection?
    if (_reopened) {
        if ((mnow - _reopened) < 5000)
            return -1;
    }

    ssize_t ret = sock->recv(buf, n, 1);

    if (ret == 0) {
        //::printf("socket has closed\n");
        delete sock;
        sock = NULL;
        return -1;
    }
    return ret;
}

bool TCPClientDevice::open()
{
    if (sock != NULL)
        return true;

    sock = new SocketAPM(false);
    if (sock != NULL) {
        if (!sock->connect(_ip, _port))
        {
            //::printf("connect failed on %s port %u - %s\n",
            //         _ip,
            //         _port,
            //         strerror(errno));
            delete sock;
            sock = NULL;
            return false;
        }
    } else {
        return false;
    }

    return true;
}

bool TCPClientDevice::close()
{
    if (sock != NULL) {
        delete sock;
        sock = NULL;
    }
    return true;
}

void TCPClientDevice::set_blocking(bool blocking)
{
    //_blocking = blocking;
    //if (sock != NULL)
    //    sock->set_blocking(_blocking);
}

void TCPClientDevice::set_speed(uint32_t speed)
{
}

#endif
