#include "ugv/robot_tools.hpp"
#include <iostream>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>

RobotTools::RobotTools() : serial_fd_(-1) {}

RobotTools::~RobotTools() {
    close_serial();
}

bool RobotTools::open_serial(const std::string& port, int baud_rate_val) {
    // Open for non-blocking reads and writes
    serial_fd_ = open(port.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (serial_fd_ == -1) {
        std::cerr << "Error opening serial port " << port << std::endl;
        return false;
    }

    struct termios options;
    tcgetattr(serial_fd_, &options);

    // Set Baud Rate
    speed_t baud_rate_const;
    switch (baud_rate_val) {
        case 9600:   baud_rate_const = B9600;   break;
        case 19200:  baud_rate_const = B19200;  break;
        case 38400:  baud_rate_const = B38400;  break;
        case 57600:  baud_rate_const = B57600;  break;
        case 115200: baud_rate_const = B115200; break;
        default:
            std::cerr << "Unsupported baud rate: " << baud_rate_val << std::endl;
            close(serial_fd_);
            serial_fd_ = -1;
            return false;
    }
    cfsetispeed(&options, baud_rate_const);
    cfsetospeed(&options, baud_rate_const);

    options.c_cflag |= (CLOCAL | CREAD); // Enable receiver, local mode
    options.c_cflag &= ~CSIZE;           // Clear data size bits
    options.c_cflag |= CS8;              // 8 data bits
    options.c_cflag &= ~PARENB;          // No parity
    options.c_cflag &= ~CSTOPB;          // 1 stop bit

    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG); // Raw input
    options.c_iflag &= ~(IXON | IXOFF | IXANY); // No software flow control
    options.c_oflag &= ~OPOST;           // Raw output

    // Configure for non-blocking reads
    options.c_cc[VMIN] = 0;
    options.c_cc[VTIME] = 0;

    tcsetattr(serial_fd_, TCSANOW, &options);

    // Flush any existing data in the serial buffer
    tcflush(serial_fd_, TCIOFLUSH);

    return true;
}

void RobotTools::close_serial() {
    if (is_open()) {
        close(serial_fd_);
        serial_fd_ = -1;
    }
}

bool RobotTools::is_open() const {
    return serial_fd_ != -1;
}

void RobotTools::setup_robot() {
    // robot type = 0
    nlohmann::json cmd1;
    cmd1["T"] = "4";
    cmd1["cmd"] = 0;
    send_command(cmd1);
    
    // set streaming UART
    nlohmann::json cmd2;
    cmd2["T"] = "131";
    cmd2["cmd"] = 1;
    send_command(cmd2);
}

void RobotTools::reset_robot() {
    setup_robot();
    nlohmann::json zero_vel_cmd;
    zero_vel_cmd["T"] = "13";
    zero_vel_cmd["X"] = 0.0;
    zero_vel_cmd["Z"] = 0.0;
    for (int i = 0; i < 20; ++i) {
        send_command(zero_vel_cmd);
    }
}

void RobotTools::send_command(const nlohmann::json& command) {
    if (!is_open()) {
        std::cerr << "Serial port not open." << std::endl;
        return;
    }
    std::string message = command.dump() + "\n";
    write(serial_fd_, message.c_str(), message.length());
}

std::string RobotTools::read_line() {
    if (!is_open()) {
        std::cerr << "Serial port not open." << std::endl;
        return "";
    }
    
    // This loop continuously tries to find and extract a valid JSON line
    while (true) {
        // --- 1. Read more data from serial port (blocks for up to 1 second) ---
        char temp_buf[2048];
        ssize_t bytes_read = read(serial_fd_, temp_buf, sizeof(temp_buf));

        if (bytes_read > 0) {
            read_buffer_.insert(read_buffer_.end(), temp_buf, temp_buf + bytes_read);
            // Debug print of buffer update
            // std::cerr << "[read_line DEBUG] Read " << bytes_read << " bytes. Buffer size: " << read_buffer_.size() << ". Content (first 50): '"
            //           << std::string(read_buffer_.begin(), read_buffer_.size() > 50 ? read_buffer_.begin() + 50 : read_buffer_.end())
            //           << "'" << std::endl;
        } else if (bytes_read == 0) { // Timeout occurred, no more data for now
            // If we have some data in buffer, but no complete JSON, keep it.
            // Only return empty string if buffer is also empty, to prevent busy looping on empty buffer.
            if (read_buffer_.empty()) {
                // std::cerr << "[read_line DEBUG] Read timeout (0 bytes read) and buffer empty, returning empty string." << std::endl;
                return "";
            }
            // If buffer has data, don't return immediately, let the JSON finding logic below run.
            // If no JSON found, it will fall through and return "" at the end of loop.
            //std::cerr << "[read_line DEBUG] Read timeout (0 bytes read), but buffer has data. Checking buffer." << std::endl;
        } else if (bytes_read == -1 && errno == EAGAIN) { // No data currently available
            // Similar to bytes_read == 0 for practical purposes with VTIME=10
            if (read_buffer_.empty()) {
                // std::cerr << "[read_line DEBUG] No data available (EAGAIN) and buffer empty, returning empty string." << std::endl;
                return "";
            }
            // std::cerr << "[read_line DEBUG] No data available (EAGAIN), but buffer has data. Checking buffer." << std::endl;
        } else {
            // A real read error occurred
            perror("read");
            return "";
        }

        // --- 2. Now, try to find a complete JSON object within the read_buffer_ ---
        auto start_pos = std::find(read_buffer_.begin(), read_buffer_.end(), '{');

        if (start_pos != read_buffer_.end()) {
            // Discard any data before the opening brace '{'
            if (start_pos != read_buffer_.begin()) {
                //std::cerr << "[read_line DEBUG] Discarding leading non-JSON data." << std::endl;
                read_buffer_.erase(read_buffer_.begin(), start_pos);
                start_pos = read_buffer_.begin(); // Reset start_pos after erase
            }

            // Now search for the matching '}'
            auto end_brace_pos = std::find(start_pos, read_buffer_.end(), '}');
            if (end_brace_pos != read_buffer_.end()) {
                // Found a closing brace, now search for a newline after it
                auto newline_pos = std::find(end_brace_pos, read_buffer_.end(), '\n');
                if (newline_pos != read_buffer_.end()) {
                    // Found a complete JSON object including its newline
                    std::string line(start_pos, newline_pos); // Extract from '{' to '\n' (excluding '\n')
                    read_buffer_.erase(read_buffer_.begin(), newline_pos + 1); // Erase the extracted line + newline
                    
                    // Remove trailing carriage return if present
                    if (!line.empty() && line.back() == '\r') {
                        //std::cerr << "[read_line DEBUG] Stripped \\r from line." << std::endl;
                        line.pop_back();
                    }
                    //std::cerr << "[read_line DEBUG] Returning JSON line: '" << line << "'" << std::endl;
                    return line; // Return the valid JSON line
                }
            }
        }
        
        // --- 3. Safeguard against unbounded buffer growth with corrupted stream ---
        if (read_buffer_.size() > 4096) { // Arbitrary limit
            //std::cerr << "[read_line DEBUG] Read buffer exceeded limit (" << read_buffer_.size() << " bytes), clearing to resynchronize." << std::endl;
            read_buffer_.clear(); // Clear the buffer to resynchronize
            return ""; // Cleared, nothing returned for this call
        }

        // If no complete JSON object was found after processing new data,
        // and read() might have timed out, or no new data arrived, then the loop will
        // continue to try reading more data in the next iteration of the while(true) loop.
        // This is where the blocking aspect comes into play. If bytes_read was 0 or EAGAIN,
        // we keep the loop going to check the buffer again, and then read more.

    } // End of while(true) loop
}

void RobotTools::clear_read_buffer() {
    read_buffer_.clear();
}

// --- libudev implementation for finding serial port ---
#include <libudev.h>
#include <string.h>

std::string RobotTools::find_robot_tty(const std::string& vendor_id, const std::string& product_id) {
    struct udev* udev = udev_new();
    if (!udev) {
        std::cerr << "Failed to create udev context." << std::endl;
        return "";
    }

    struct udev_enumerate* enumerate = udev_enumerate_new(udev);
    if (!enumerate) {
        std::cerr << "Failed to create udev enumerate object." << std::endl;
        udev_unref(udev);
        return "";
    }

    // Add match criteria: "usb-serial" subsystem for USB-to-serial converters
    // or "tty" subsystem and filter by USB device type if not a dedicated converter
    udev_enumerate_add_match_subsystem(enumerate, "tty");
    // You might also need to add properties like "ID_BUS=usb" to filter tty devices further.

    udev_enumerate_scan_devices(enumerate);
    struct udev_list_entry* devices = udev_enumerate_get_list_entry(enumerate);
    struct udev_list_entry* dev_list_entry;

    std::string found_tty_path = "";

    udev_list_entry_foreach(dev_list_entry, devices) {
        const char* path = udev_list_entry_get_name(dev_list_entry);
        struct udev_device* dev = udev_device_new_from_syspath(udev, path);
        
        // Check if it's a serial device and get its parent USB device
        struct udev_device* usb_dev = udev_device_get_parent_with_subsystem_devtype(
            dev, "usb", "usb_device");

        if (usb_dev) {
            const char* dev_vendor_id = udev_device_get_sysattr_value(usb_dev, "idVendor");
            const char* dev_product_id = udev_device_get_sysattr_value(usb_dev, "idProduct");

            if (dev_vendor_id && dev_product_id) {
                if (vendor_id == dev_vendor_id && product_id == dev_product_id) {
                    // This is our robot! Get the device node (e.g., /dev/ttyACM0)
                    const char* tty_path = udev_device_get_devnode(dev);
                    if (tty_path) {
                        found_tty_path = tty_path;
                        udev_device_unref(dev);
                        break; // Found it, exit loop
                    }
                }
            }
        }
        udev_device_unref(dev);
    }

    udev_enumerate_unref(enumerate);
    udev_unref(udev);

    if (found_tty_path.empty()) {
        std::cerr << "Robot with Vendor ID: " << vendor_id << " and Product ID: " << product_id << " not found." << std::endl;
    } else {
        std::cout << "Found robot at: " << found_tty_path << std::endl;
    }
    return found_tty_path;
}
