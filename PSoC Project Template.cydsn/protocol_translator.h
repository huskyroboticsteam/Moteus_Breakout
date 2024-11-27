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
// CAN ID bit positions as per specification
#pragma once
#include "CANPacket.h"
#include "moteus_protocol.h"

// CAN ID construction based on your specification
#define PRIORITY_HIGH    0
#define PRIORITY_NORMAL  1

// Device Groups from your spec
#define DEVICE_GROUP_BROADCAST  0x00
#define DEVICE_GROUP_MOTOR      0x04

// Serial Numbers
#define SERIAL_BROADCAST        0x00
#define GROUP_BROADCAST_SERIAL  0x3F

// Function prototypes
int ProcessMoteusToHindsight(const CANPacket* moteus_packet, CANPacket* hindsight_packet);
int ProcessHindsightToMoteus(const CANPacket* hindsight_packet, CANPacket* moteus_packet);
int IsMoteusCAN(const CANPacket* packet);
/* [] END OF FILE */
