#include "vesc.hpp"
#include <iostream>

namespace vesc {

    BridgedVescArray::BridgedVescArray(std::vector<uint8_t> vescIds) 
    {
        vescIds_ = vescIds;
        currentVoltage_ = 0.0;
        currentAmperage_ = 0.0;
    }

    vescChannelStatus BridgedVescArray::parseReceivedMessage(std::vector<uint8_t> robotmsg) 
    {

        auto full_msg = static_cast<uint32_t>((robotmsg[0] << 24) + (robotmsg[1] << 16) + (robotmsg[2] << 8) + robotmsg[3]);

        uint8_t vescId = full_msg & ID_MASK;
        uint8_t commandId = (full_msg & COMMAND_MASK) >> 8;

        // here we process the adequate status packets 
        if (commandId == STATUS_COMMAND_ID)
        {

            /* combine shifted byte values into a single rpm value */
            int32_t rpm_scaled = (robotmsg[5] << 24) | (robotmsg[6] << 16) | (robotmsg[7] << 8) | (robotmsg[8]);

            /* combine shifted byte values into a single current value */
            int16_t current_scaled = (robotmsg[9] << 8) | (robotmsg[10]);

            /* combine shifted byte values into a single duty value */
            int16_t duty_scaled = (robotmsg[11] << 8) | (robotmsg[12]);
            
            /* scale values per fixed-point vesc protocol */
            float rpm = ((float)rpm_scaled) * RPM_SCALING_FACTOR;
            float current = ((float)current_scaled) * CURRENT_SCALING_FACTOR;
            float duty = ((float)duty_scaled) * DUTY_SCALING_FACTOR;

            return (vescChannelStatus){.vescId = vescId,
                                    .current = current,
                                    .rpm = rpm,
                                    .duty = duty,
                                    .voltage = currentVoltage_,
                                    .current_in = currentAmperage_, 
                                    .dataValid = true};
            
        }
        else if (commandId == STATUS_COMMAND_ID_4)
        {
            float amperage_scaled = (robotmsg[9] << 8) | (robotmsg[10]);
            currentAmperage_ = ((float)amperage_scaled) * CURRENT_IN_SCALING_FACTOR;

            return (vescChannelStatus){
                .vescId = 0, 
                .current = 0, 
                .rpm = 0, 
                .duty = 0, 
                .voltage = 0,
                .current_in = 0, 
                .dataValid = false};
        }
        else if (commandId == STATUS_COMMAND_ID_5)
        {
            float voltage_scaled = (robotmsg[9] << 8) | (robotmsg[10]);
            currentVoltage_ = ((float)voltage_scaled) * VOLTAGE_SCALING_FACTOR;

            return (vescChannelStatus){
                .vescId = 0, 
                .current = 0, 
                .rpm = 0, 
                .duty = 0, 
                .voltage = 0,
                .current_in = 0, 
                .dataValid = false};
        }
    }

    std::vector<uint8_t> BridgedVescArray::buildCommandMessage(vesc::vescChannelCommand command) 
    {
        /* create a vector to hold the message */
        std::vector<uint8_t> write_buffer;

        /* build the message */
        switch (command.commandType) 
        {
            case (RPM):
                command.commandValue /= RPM_SCALING_FACTOR;
                break;
            case (CURRENT):
                command.commandValue /= CURRENT_SCALING_FACTOR;
                break;
            case (DUTY):
                command.commandValue *= DUTY_COMMAND_SCALING_FACTOR;
                break;
            default:
                std::cerr << "unknown command type" << std::endl;
                exit(-1);
        };

        auto casted_command = static_cast<int32_t>(command.commandValue);
        auto full_id = static_cast<uint32_t>(command.vescId | vescPacketFlags::PACKET_FLAG | command.commandType);

        write_buffer = {static_cast<uint8_t>((full_id >> 24) & 0xFF),
                        static_cast<uint8_t>((full_id >> 16) & 0xFF),
                        static_cast<uint8_t>((full_id >> 8) & 0xFF),
                        static_cast<uint8_t>(full_id & 0xFF),
                        SEND_MSG_LENGTH,
                        static_cast<uint8_t>((casted_command >> 24) & 0xFF),
                        static_cast<uint8_t>((casted_command >> 16) & 0xFF),
                        static_cast<uint8_t>((casted_command >> 8) & 0xFF),
                        static_cast<uint8_t>(casted_command & 0xFF)};

        return write_buffer;
    }

}  // namespace vesc
