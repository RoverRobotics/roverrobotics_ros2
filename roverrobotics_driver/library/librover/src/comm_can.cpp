#include "comm_can.hpp"

namespace RoverRobotics 
{
    CommCan::CommCan(const char *device,std::function<void(std::vector<uint8_t>)> parsefunction,std::vector<uint8_t> setting)
    : is_connected_(false) 
    {
        if ((fd = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0) 
        {
            // failed to create socket
            throw(-1);
        }
        strcpy(ifr.ifr_name, device);
        ioctl(fd, SIOCGIFINDEX, &ifr);
        addr.can_family = AF_CAN;
        addr.can_ifindex = ifr.ifr_ifindex;

        if (bind(fd, (struct sockaddr *)&addr, sizeof(addr)) < 0) 
        {
            std::cerr << "error in socket bind" << std::endl;
            throw(-2);
        }
        // start read thread
        Can_read_thread_ = std::thread([this, parsefunction]() { this->read_device_loop(parsefunction); });
    }

    void CommCan::write_to_device(std::vector<uint8_t> msg) 
    {
        Can_write_mutex_.lock();
        if (msg.size() == CAN_MSG_SIZE_) 
        {
            // convert msg to frame
            frame.can_id = static_cast<uint32_t>((msg[0] << 24) + (msg[1] << 16) + (msg[2] << 8) + msg[3]);
            frame.can_dlc = msg[4];
            frame.data[0] = msg[5];
            frame.data[1] = msg[6];
            frame.data[2] = msg[7];
            frame.data[3] = msg[8];
            write(fd, &frame, sizeof(struct can_frame));
        }
        Can_write_mutex_.unlock();
    }

    void CommCan::read_device_loop(std::function<void(std::vector<uint8_t>)> parsefunction) 
    {
        std::chrono::milliseconds time_last = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch());
        
        while (true) 
        {
            int num_bytes = read(fd, &robot_frame, sizeof(robot_frame));
            std::chrono::milliseconds time_now = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch());
            
            if (num_bytes <= 0) 
            {
                if ((time_now - time_last).count() > TIMEOUT_MS_) 
                {
                    is_connected_ = false;
                }
                continue;
            }
            is_connected_ = true;
            time_last = time_now;
            std::vector<uint8_t> msg;
            
            msg.push_back(robot_frame.can_id >> 24);
            msg.push_back(robot_frame.can_id >> 16);
            msg.push_back(robot_frame.can_id >> 8);
            msg.push_back(robot_frame.can_id);
            msg.push_back(robot_frame.can_dlc);

            for (int i = 0; i < sizeof(robot_frame.data); i++) 
            {
                msg.push_back(robot_frame.data[i]);
            }

            parsefunction(msg);
            msg.clear();

        }
    }

    bool CommCan::is_connected() { return (is_connected_); }

}  // namespace RoverRobotics
