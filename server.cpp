

#include <stdio.h>
#include <stdlib.h>

#include "LidarFindObject.h"

#include <iostream>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <string>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <cstring>

// Server configuration constants
static const int SERVER_PORT = 12345;
static const int BUFFER_SIZE = 1024;
static const int SOCKET_TIMEOUT_SEC = 1;

static std::mutex mtx;
static std::condition_variable cv;
static bool shutdown_requested = false;
static std::string data_to_send;


// Non-blocking function to send data to client
// Returns true if data was queued for sending, false if send in progress, no client, or server shutting down
bool send_server_data(const std::string& data) 
{
    // Try to get mutex lock - if mutex is locked return false (non-blocking)
    if (!mtx.try_lock()) return false;  // Failed to acquire lock immediately
    std::lock_guard<std::mutex> lock(mtx, std::adopt_lock);  // Adopt the already-locked mutex
    if (shutdown_requested) return false;  // Check after acquiring lock
    data_to_send = data;  // deep copy
    cv.notify_all();  // Notify server thread that new data is available
    return true;  // Data queued successfully
}

// Function to check if server is running
bool is_server_running() {
    std::lock_guard<std::mutex> lock(mtx);
    return !shutdown_requested;
}

// Function to send cleanup signal to server
void cleanup_server() 
{
    {
        std::lock_guard<std::mutex> lock(mtx);
        shutdown_requested = true;
    }
    cv.notify_all();  // Wake up waiting server thread
}

static void server_thread(void)
{
    int sock = -1;
    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_port = htons(SERVER_PORT);
    sockaddr_in client_addr{};  // Add client address storage
    socklen_t client_len = sizeof(client_addr);  // Add client length
    char buffer[BUFFER_SIZE];

    sock = socket(AF_INET, SOCK_DGRAM, 0);
    if (sock < 0) 
    {
        printf("Failed to create socket\n");
        return;
    }

    addr.sin_addr.s_addr = INADDR_ANY;
    if (bind(sock, (sockaddr*)&addr, sizeof(addr)) < 0) 
    {
        printf("Failed to bind to port %d\n", SERVER_PORT);
        close(sock);
        return;
    }
    
    printf("Server started on port %d\n", SERVER_PORT);

    while (!shutdown_requested) 
    {
        // Set socket timeout to allow periodic shutdown checks
        struct timeval timeout;
        timeout.tv_sec = SOCKET_TIMEOUT_SEC;  // Use constant
        timeout.tv_usec = 0;
        if (setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout)) < 0) {
            perror("setsockopt failed");
        }
        
        ssize_t len = recvfrom(sock, buffer, sizeof(buffer), 0, (sockaddr*)&client_addr, &client_len);
        if (len > 0) 
        {
            if (!shutdown_requested) 
            {
                // Send the message
                if (client_addr.sin_family == AF_INET) 
                {
                    // wait for signal from send_server_data
                    std::unique_lock<std::mutex> lock(mtx);
                    while (data_to_send.empty() && !shutdown_requested) {
                        cv.wait(lock);
                    }
                    if (!shutdown_requested && !data_to_send.empty()) {
                        ssize_t sent = sendto(sock, data_to_send.c_str(), data_to_send.size(), 0, (sockaddr*)&client_addr, sizeof(client_addr));
                        if (sent < 0) {
                            perror("sendto failed");
                        } else {
                            printf("Sent %zd bytes to client\n", sent);
                        }
                        data_to_send.clear();  // Clear after sending to prevent re-sending
                    }
                }
            }
        }
        // len <= 0 could be timeout or error - continue if not shutting down
    }

    close(sock);
}


int start_server() 
{
    // Create server thread
    std::thread srv_thread(server_thread);
    srv_thread.detach();  // Detach to run independently
    printf("Server threads started, returning from start_server()\n");
    return 0;  // Return immediately, threads run in background
}
