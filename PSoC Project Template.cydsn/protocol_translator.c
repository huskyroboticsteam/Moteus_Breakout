/* ========================================
 *
 * Copyright YOUR COMPANY, THE YEAR
 * All Rights Reserved
 * UNPUBLISHED, LICENSED SOFTWARE.
 *
 * CONFIDENTIAL AND PROPRIETARY INFORMATION
 * WHICH IS THE PROPERTY OF your company.
 *
 * ========================================
*/

#include "protocol_translator.h"
#include "HindsightCAN/CANLibrary.h"

int ProcessMoteusToHindsight(const CANPacket* moteus_packet, CANPacket* hindsight_packet) {
    if (!moteus_packet || !hindsight_packet) return -1;
    
    MoteusCommand cmd;
    if (DecodeMoteusPacket(moteus_packet, &cmd)) {
        switch(cmd.mode) {
            case MOTEUS_MODE_POSITION:
                // Set mode to PID first
                hindsight_packet->id = ConstructCANID(PRIORITY_NORMAL, 
                                                    DEVICE_GROUP_MOTOR, 
                                                    getLocalDeviceSerial());
                hindsight_packet->dlc = 2;  // As per your spec
                hindsight_packet->data[0] = PACKET_MODE_SET;
                hindsight_packet->data[1] = MODE_PID;
                
                // Then send position target
                hindsight_packet->id = ConstructCANID(PRIORITY_NORMAL, 
                                                    DEVICE_GROUP_MOTOR, 
                                                    getLocalDeviceSerial());
                hindsight_packet->dlc = 5;  // B1-B4 for position data
                EncodePIDTargetPacket(hindsight_packet->data, 
                                    (int32_t)(cmd.position * 1000.0f));  // Convert to mdeg
                break;
                
            case MOTEUS_MODE_VOLTAGE:
                // Set mode to PWM first
                hindsight_packet->id = ConstructCANID(PRIORITY_NORMAL, 
                                                    DEVICE_GROUP_MOTOR, 
                                                    getLocalDeviceSerial());
                hindsight_packet->dlc = 2;
                hindsight_packet->data[0] = PACKET_MODE_SET;
                hindsight_packet->data[1] = MODE_PWM;
                
                // Then send PWM value
                hindsight_packet->id = ConstructCANID(PRIORITY_NORMAL, 
                                                    DEVICE_GROUP_MOTOR, 
                                                    getLocalDeviceSerial());
                hindsight_packet->dlc = 3;  // PWM uses 2 bytes for data
                EncodePWMPacket(hindsight_packet->data, 
                              (int16_t)(cmd.voltage * 32767.0f));
                break;
        }
        return 0;
    }
    return -1;
}

int ProcessHindsightToMoteus(const CANPacket* hindsight_packet, CANPacket* moteus_packet) {
    if (!hindsight_packet || !moteus_packet) return -1;
    
    switch(hindsight_packet->data[0]) {  // Packet ID
        case PACKET_MODE_SET: {
            MotorMode mode = DecodeModeSetPacket(hindsight_packet->data);
            // Translate to Moteus mode command
            if (mode == MODE_PWM) {
                EncodeMoteusMode(moteus_packet, MOTEUS_MODE_VOLTAGE);
            } else {
                EncodeMoteusMode(moteus_packet, MOTEUS_MODE_POSITION);
            }
            break;
        }
        
        case PACKET_PWM_DIR_SET: {
            int16_t pwm = DecodePWMPacket(hindsight_packet->data);
            float voltage = pwm / 32767.0f;
            EncodeMoteusVoltage(moteus_packet, voltage);
            break;
        }
        
        case PACKET_PID_POS_TARGET: {
            int32_t position_mdeg = DecodePIDTargetPacket(hindsight_packet->data);
            float position_rot = position_mdeg / 1000.0f / 360.0f;  // Convert mdeg to rotations
            EncodeMoteusPosition(moteus_packet, position_rot);
            break;
        }
        
        case PACKET_LIM_SWITCH: {
            uint8_t switches = DecodeLimitSwitchPacket(hindsight_packet->data);
            // Handle limit switch status for Moteus
            // This might need special handling depending on how Moteus handles limits
            break;
        }
    }
    return 0;
}

int IsMoteusCAN(const CANPacket* packet) {
    // Implement based on your Moteus ID range
    return (packet->id & 0x7FF) >= MOTEUS_ID_START && 
           (packet->id & 0x7FF) <= MOTEUS_ID_END;
}
/* [] END OF FILE */
