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
#pragma once
#include <stdbool.h>
#include <stdint.h>

// Motor Unit Packet Types
typedef enum {
    PACKET_MODE_SET = 0x00,
    PACKET_RESERVED_1 = 0x01,
    PACKET_RESERVED_2 = 0x02,
    PACKET_PWM_DIR_SET = 0x03,
    PACKET_PID_POS_TARGET = 0x04,
    PACKET_PID_P_SET = 0x05,
    PACKET_PID_I_SET = 0x06,
    PACKET_PID_D_SET = 0x07,
    PACKET_INITIALIZE = 0x08,
    PACKET_LIM_SWITCH = 0x09,
    PACKET_ENCODER_PPJ = 0x0A,
    PACKET_MAX_JOINT_REV = 0x0B,
    PACKET_ENCODER_INIT = 0x0C,
    PACKET_MAX_PID_PWM = 0x0D,
    PACKET_PCA_PWM = 0x0E,
    PACKET_POT_INIT_LOW = 0x0F,
    PACKET_POT_INIT_HIGH = 0x10
} MotorPacketType;

// Motor Unit Operation Modes
typedef enum {
    MODE_PWM = 0x00,
    MODE_PID = 0x01
} MotorMode;

// Encoder initialization bits
typedef struct {
    uint8_t use_potentiometer : 1;  // 0 = encoder, 1 = potentiometer
    uint8_t reverse_direction : 1;   // 0 = normal, 1 = reverse
    uint8_t zero_angle : 1;         // 1 = set current as zero
    uint8_t reserved : 5;           // Reserved bits
} EncoderInitBits;

// Function prototypes for encoding packets
void EncodeModeSetPacket(uint8_t* data, MotorMode mode);
void EncodePWMPacket(uint8_t* data, int16_t pwm);
void EncodePIDTargetPacket(uint8_t* data, int32_t position_mdeg);
void EncodePIDCoeffPacket(uint8_t* data, int32_t coeff);
void EncodeEncoderInitPacket(uint8_t* data, EncoderInitBits config);
void EncodeLimitSwitchPacket(uint8_t* data, uint8_t switches);
void EncodeEncoderPPJPacket(uint8_t* data, uint32_t pulses);
void EncodePotInitPacket(uint8_t* data, uint16_t adc_value, int32_t position_mdeg, bool is_high);

// Function prototypes for decoding packets
MotorMode DecodeModeSetPacket(const uint8_t* data);
int16_t DecodePWMPacket(const uint8_t* data);
int32_t DecodePIDTargetPacket(const uint8_t* data);
int32_t DecodePIDCoeffPacket(const uint8_t* data);
EncoderInitBits DecodeEncoderInitPacket(const uint8_t* data);
uint8_t DecodeLimitSwitchPacket(const uint8_t* data);
uint32_t DecodeEncoderPPJPacket(const uint8_t* data);
/* [] END OF FILE */
