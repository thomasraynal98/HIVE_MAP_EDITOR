#define ASIO_STANDALONE 
#define SIO_TLS

#include <iomanip>
#include <sstream>
#include <iostream>
#include <thread>
#include <chrono>
#include <vector>
#include <sio_client.h>

#include <string.h>
#include <math.h>
#include <fstream>
#include <cstdlib>
#include <unistd.h>

std::thread thread_event;
std::thread thread_server;

sio::client h;

void f_thread_server()
{
    while(true)
    {
        if(!h.opened())
        {
            usleep(500000);
            h.connect("https://api.hiverobotics.fr");
            usleep(10000);
        }
        if(h.opened())
        {
            usleep(10000);
            h.socket()->emit("NEW_MAP_UPDATE");
            return;
        }
    }
}

int main(int argc, char *argv[])
{
    thread_server = std::thread(&f_thread_server);

    thread_server.join();

    return 0;
}