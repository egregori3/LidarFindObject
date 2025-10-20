

#include <stdio.h>
#include <stdlib.h>
#include <signal.h>

#include "LidarFindObject.h"

#include <iostream>
#include <thread>
#include <queue>
#include <mutex>
#include <condition_variable>
#include <string>
#include <chrono>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <cstring>

struct SharedData {
    std::queue<std::string> recv_queue;
    std::queue<std::string> send_queue;
    std::mutex mtx;
    std::condition_variable cv;
    int sock;
    sockaddr_in last_client;
    bool shutdown_requested;
    std::thread* rx_thread;
    std::thread* tx_thread;
};

// Global pointer to shared data for cleanup access
static SharedData* g_server_data = nullptr;

// Non-blocking function to get data from receive queue
// Returns true if data was retrieved, false if queue is empty
bool get_server_data(std::string& data_out) {
    if (!g_server_data) {
        return false;  // Server not initialized
    }
    
    std::lock_guard<std::mutex> lock(g_server_data->mtx);
    if (g_server_data->recv_queue.empty()) {
        return false;  // No data available
    }
    
    data_out = g_server_data->recv_queue.front();
    g_server_data->recv_queue.pop();
    return true;  // Data retrieved successfully
}

// Non-blocking function to send data to client
// Returns true if data was queued for sending, false if no client connected
bool send_server_data(const std::string& data) {
    if (!g_server_data) {
        return false;  // Server not initialized
    }
    
    std::lock_guard<std::mutex> lock(g_server_data->mtx);
    if (g_server_data->last_client.sin_family != AF_INET) {
        return false;  // No client connected
    }
    
    g_server_data->send_queue.push(data);
    g_server_data->cv.notify_one();
    return true;  // Data queued for sending
}

// Function to check if server is running
bool is_server_running() {
    return (g_server_data != nullptr && !g_server_data->shutdown_requested);
}

static void rx_thread(SharedData& data) {
    char buffer[1024];
    sockaddr_in client_addr{};
    socklen_t client_len = sizeof(client_addr);

    while (!data.shutdown_requested) {
        // Set socket timeout to allow periodic shutdown checks
        struct timeval timeout;
        timeout.tv_sec = 1;  // 1 second timeout
        timeout.tv_usec = 0;
        setsockopt(data.sock, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));
        
        ssize_t len = recvfrom(data.sock, buffer, sizeof(buffer), 0, (sockaddr*)&client_addr, &client_len);
        if (len > 0) {
            std::lock_guard<std::mutex> lock(data.mtx);
            if (!data.shutdown_requested) {
                data.recv_queue.push(std::string(buffer, len));
                data.last_client = client_addr;
                data.cv.notify_one();
            }
        }
        // len <= 0 could be timeout or error - continue if not shutting down
    }
}

static void tx_thread(SharedData& data) {
    while (!data.shutdown_requested) {
        std::unique_lock<std::mutex> lock(data.mtx);
        // Wait with timeout to allow periodic shutdown checks
        data.cv.wait_for(lock, std::chrono::seconds(1), [&data]{ 
            return !data.send_queue.empty() || data.shutdown_requested; 
        });
        
        if (data.shutdown_requested) break;
        
        if (!data.send_queue.empty()) {
            std::string msg = data.send_queue.front();
            data.send_queue.pop();
            lock.unlock();

            if (data.last_client.sin_family == AF_INET) {
                sendto(data.sock, msg.c_str(), msg.size(), 0, (sockaddr*)&data.last_client, sizeof(data.last_client));
            }
        }
    }
}



// Function to send cleanup signal to server
void cleanup_server() 
{
    if (g_server_data) {
        printf("Initiating server cleanup...\n");
        
        // Signal shutdown to threads
        {
            std::lock_guard<std::mutex> lock(g_server_data->mtx);
            g_server_data->shutdown_requested = true;
        }
        g_server_data->cv.notify_all();
        
        // Wait for threads to finish (with timeout)
        if (g_server_data->rx_thread && g_server_data->rx_thread->joinable()) {
            printf("Waiting for receive thread to finish...\n");
            g_server_data->rx_thread->join();
            delete g_server_data->rx_thread;
            g_server_data->rx_thread = nullptr;
        }
        if (g_server_data->tx_thread && g_server_data->tx_thread->joinable()) {
            printf("Waiting for transmit thread to finish...\n");
            g_server_data->tx_thread->join();
            delete g_server_data->tx_thread;
            g_server_data->tx_thread = nullptr;
        }
        
        // Close socket
        if (g_server_data->sock >= 0) {
            printf("Closing server socket...\n");
            close(g_server_data->sock);
            g_server_data->sock = -1;
        }
        
        // Clear global pointer
        g_server_data = nullptr;
        
        printf("Server cleanup completed.\n");
    } else {
        printf("Server was not running.\n");
    }
}

int start_server() 
{
    static SharedData data{};  // Make it static so it persists
    g_server_data = &data;     // Set global pointer
    int port = 12345;
    
    // Initialize shutdown flag
    data.shutdown_requested = false;
    data.sock = -1;  // Initialize socket to invalid value
    data.rx_thread = nullptr;  // Initialize thread pointers
    data.tx_thread = nullptr;
        
    data.sock = socket(AF_INET, SOCK_DGRAM, 0);
    if (data.sock < 0) {
        printf("Failed to create socket\n");
        g_server_data = nullptr;
        return 1;
    }

    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_port = htons(port);
    addr.sin_addr.s_addr = INADDR_ANY;
    if (bind(data.sock, (sockaddr*)&addr, sizeof(addr)) < 0) {
        printf("Failed to bind to port %d\n", port);
        close(data.sock);
        g_server_data = nullptr;
        return 1;
    }
    
    printf("Server started on port %d\n", port);

    // Create threads using regular pointers (C++11 compatible)
    data.rx_thread = new std::thread(rx_thread, std::ref(data));
    data.tx_thread = new std::thread(tx_thread, std::ref(data));

    printf("Server threads started, returning from start_server()\n");
    return 0;  // Return immediately, threads run in background
}
