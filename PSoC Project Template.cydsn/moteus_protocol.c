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
#include "moteus_protocol.h"
#include <string.h>

void EncodeModeSetPacket(uint8_t* data, MotorMode mode) {
    data[0] = PACKET_MODE_SET;
    data[1] = mode;
}

void EncodePWMPacket(uint8_t* data, int16_t pwm) {
    data[0] = PACKET_PWM_DIR_SET;
    // Pack MSB first
    data[1] = (pwm >> 8) & 0xFF;
    data[2] = pwm & 0xFF;
}

void EncodePIDTargetPacket(uint8_t* data, int32_t position_mdeg) {
    data[0] = PACKET_PID_POS_TARGET;
    // Pack MSB first
    data[1] = (position_mdeg >> 24) & 0xFF;
    data[2] = (position_mdeg >> 16) & 0xFF;
    data[3] = (position_mdeg >> 8) & 0xFF;
    data[4] = position_mdeg & 0xFF;
}

void EncodePIDCoeffPacket(uint8_t* data, int32_t coeff) {
    // Note: coefficient is multiplied by 10000 as per specification
    int32_t scaled_coeff = coeff * 10000;
    // Pack MSB first
    data[1] = (scaled_coeff >> 24) & 0xFF;
    data[2] = (scaled_coeff >> 16) & 0xFF;
    data[3] = (scaled_coeff >> 8) & 0xFF;
    data[4] = scaled_coeff & 0xFF;
}

void EncodeEncoderInitPacket(uint8_t* data, EncoderInitBits config) {
    data[0] = PACKET_ENCODER_INIT;
    data[1] = *((uint8_t*)&config); // Pack bits into byte
}

void EncodeLimitSwitchPacket(uint8_t* data, uint8_t switches) {
    data[0] = PACKET_LIM_SWITCH;
    data[1] = getLocalDeviceGroup();
    data[2] = getLocalDeviceSerial();
    data[3] = switches;
}

void EncodeEncoderPPJPacket(uint8_t* data, uint32_t pulses) {
    data[0] = PACKET_ENCODER_PPJ;
    // Pack MSB first
    data[1] = (pulses >> 24) & 0xFF;
    data[2] = (pulses >> 16) & 0xFF;
    data[3] = (pulses >> 8) & 0xFF;
    data[4] = pulses & 0xFF;
}

void EncodePotInitPacket(uint8_t* data, uint16_t adc_value, int32_t position_mdeg, bool is_high) {
    data[0] = is_high ? PACKET_POT_INIT_HIGH : PACKET_POT_INIT_LOW;
    // Pack ADC value
    data[1] = (adc_value >> 8) & 0xFF;
    data[2] = adc_value & 0xFF;
    // Pack position
    data[3] = (position_mdeg >> 24) & 0xFF;
    data[4] = (position_mdeg >> 16) & 0xFF;
    data[5] = (position_mdeg >> 8) & 0xFF;
    data[6] = position_mdeg & 0xFF;
}

// Decoding functions
MotorMode DecodeModeSetPacket(const uint8_t* data) {
    return (MotorMode)data[1];
}

int16_t DecodePWMPacket(const uint8_t* data) {
    return (int16_t)((data[1] << 8) | data[2]);
}

int32_t DecodePIDTargetPacket(const uint8_t* data) {
    return (int32_t)((data[1] << 24) | (data[2] << 16) | 
                     (data[3] << 8) | data[4]);
}

int32_t DecodePIDCoeffPacket(const uint8_t* data) {
    int32_t scaled_coeff = (data[1] << 24) | (data[2] << 16) | 
                          (data[3] << 8) | data[4];
    return scaled_coeff / 10000; // Convert back from fixed point
}

EncoderInitBits DecodeEncoderInitPacket(const uint8_t* data) {
    EncoderInitBits config;
    *((uint8_t*)&config) = data[1];
    return config;
}

uint8_t DecodeLimitSwitchPacket(const uint8_t* data) {
    return data[3];
}

uint32_t DecodeEncoderPPJPacket(const uint8_t* data) {
    return (uint32_t)((data[1] << 24) | (data[2] << 16) | 
                      (data[3] << 8) | data[4]);
}
/* [] END OF FILE */
