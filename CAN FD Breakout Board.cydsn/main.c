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
#include <math.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h> // for memcpy
#include "main.h"
#include "cyapicallbacks.h"
#include "CAN_Stuff.h"
#include "FSM_Stuff.h"
#include "HindsightCAN/CANLibrary.h"
// #include "Button_1.h"
// #include "Button_1_aliases.h"

#define FD_MAX_PAYLOAD  64 // CAN_FD is 64 bytes (checked with Moteus fw)
#define CAN_MAX_PAYLOAD  8
#define CAN_ID_MASK      0x7FF  // 11-bit mask for our 11 bit address

// LED stuff
volatile uint8_t CAN_time_LED = 0;
volatile uint8_t ERROR_time_LED = 0;
volatile uint8_t DBG_1_time_LED = 0; // newly added
volatile uint8_t DBG_2_time_LED = 0; // newly added
volatile uint8_t DBG_3_time_LED = 0; // newly added
volatile uint8_t CAN_FD_time_LED = 0; // newly added

// UART stuff
char txData[TX_DATA_SIZE];

// CAN stuff
CANPacket can_recieve;
CANPacket can_send;
uint8 address = 0;

CY_ISR(Period_Reset_Handler) {
    CAN_time_LED++;
    ERROR_time_LED++;

    if (ERROR_time_LED >= 3) {
        LED_ERR_Write(OFF);
    }
    if (CAN_time_LED >= 3) {
        LED_CAN_Write(OFF);
    }
}

CY_ISR(Button_1_Handler) {
    LED_DBG1_Write(!LED_DBG1_Read());
    // LED_DBG2_Write(!LED_DBG2_Read());
    // LED_DBG3_Write(!LED_DBG3_Read());
}

uint16_t getMoteusRegister(uint8_t canId) {
    switch(canId) {
        case 0x00: return 0x000; // Mode Set to kMode
        case 0x03: return 0x01C; // Motor PWM & Direction to kCommandQCurrent
        case 0x04: return 0x020; // Motor PID Position Target to kCommandPosition
        case 0x05: return 0x030; // P Coefficient Set to kPositionKp
        case 0x06: return 0x031; // I Coefficient Set to kPositionKi
        case 0x07: return 0x032; // D Coefficient Set to kPositionKd
        case 0x08: return 0x000; // Initialize with Mode to kMode (fallback)
        case 0x09: return 0x05E; // Limit Switch Alert to kAux1GpioStatus
        case 0x0A: return 0x050; // Encoder Pulse Per Joint Revolution Set to kEncoder0Position
        case 0x0B: return 0x001; // Maximum Joint Revolutions Set to kPosition
        case 0x0C: return 0x132; // Initialize Encoder to kRequireReindex
        case 0x0D: return 0x025; // Max PWM for PID to kCommandPositionMaxTorque
        case 0x0E: return 0x060; // PCA PWM to kAux1AnalogIn1
        case 0x0F: return 0x060; // Pot Set Low to kAux1AnalogIn1
        case 0x10: return 0x060; // Pot Set High to kAux1AnalogIn1+
        case 0x11: return 0x060; // PCA Servo to kAux1AnalogIn1
        case 0x12: return 0x05C; // Set Limit Switch Encoder Bound to kAux1GpioCommand
        case 0x13: return 0x05C; // Set Peripherals to kAux1GpioCommand
        default:  return 0x000; // Default fallback to kMode
    }
}

// checking the mappings, seems to be not one to one
uint8_t getCanId(uint16_t registerValue) {
    switch(registerValue) {
        case 0x000: return 0x00; // kMode to Mode Set
        case 0x01C: return 0x03; // kCommandQCurrent to Motor PWM & Direction
        case 0x020: return 0x04; // kCommandPosition to Motor PID Position Target
        case 0x030: return 0x05; // kPositionKp to P Coefficient Set
        case 0x031: return 0x06; // kPositionKi to I Coefficient Set
        case 0x032: return 0x07; // kPositionKd to D Coefficient Set
        case 0x05E: return 0x09; // kAux1GpioStatus to Limit Switch Alert
        case 0x050: return 0x0A; // kEncoder0Position to Encoder Pulse Per Joint Revolution Set
        case 0x001: return 0x0B; // kPosition to Maximum Joint Revolutions Set
        case 0x132: return 0x0C; // kRequireReindex to Initialize Encoder
        case 0x025: return 0x0D; // kCommandPositionMaxTorque to Max PWM for PID
        case 0x060: return 0x0E; // kAux1AnalogIn1 to PCA PWM
        // case 0x060: return 0x0F; // kAux1AnalogIn1 to Pot Set Low
        // case 0x060: return 0x10; // kAux1AnalogIn1+ to Pot Set High
        // case 0x060: return 0x11; // kAux1AnalogIn1 to PCA Servo
        case 0x05C: return 0x12; // kAux1GpioCommand to Set Limit Switch Encoder Bound
        // case 0x05C: return 0x13; // kAux1GpioCommand to Set Peripherals
        default:  return 0xFF; // Default fallback (invalid register)
    }
}


int main(void)
{ 
    Initialize();
    int err;
    
    for(;;)
    {
        err = 0;
        switch(GetState()) {
            
            // ------------------------------
            // 1) System starts UNINIT
            // ------------------------------
            case(UNINIT):
                // After power‐on, go to a state where we check messages
                SetStateTo(CHECK_CAN);
                break;
            
            // ------------------------------
            // 2) CHECK_CAN => bridging + local processing
            // ------------------------------
            case(CHECK_CAN):
            {
                // (A) Poll CAN_2 (Moteus FD)
                
                // implement the PollANdReceiveCAN2Packet function next time
                if (!PollAndReceiveCAN2Packet(&can_recieve))
                {
                    // We have a CAN FD packet from Moteus
                    LED_DBG2_Write(ON);
                    CAN_FD_time_LED = 0; 
                    
                    // If needed, handle FD->Classical data differences:
                    // e.g. truncate data if FD has >8 bytes, or re‐map ID
                    // For now, if DLC <=8, do a direct copy:
                    // can_send = can_recieve; 
                    
                    // ConvertFDToCAN(&can_recieve, &can_send);
                    
                    // CAN_FD -> CAN, get CAN_FD packets
                    
                    uint8_t CAN_FD = getMoteusRegister(&can_send);
                    
                   
                    // Send that packet out on the Classical CAN to Jetson
                    SendCANPacket(&can_send);
                }
                
                // (B) Poll Classical CAN (Jetson)
                if (!PollAndReceiveCANPacket(&can_recieve))
                {
                    // We have a Classical CAN packet from Jetson
                    LED_CAN_Write(ON);
                    CAN_time_LED = 0;

                    // Optional: run your existing logic
                    err = ProcessCAN(&can_recieve, &can_send);
                    
                    // Forward it to Moteus (FD)
                    ConvertCANToFD(&can_recieve, &can_send);
                    SendCAN2Packet(&can_send);
                }

                
                // (C) Check if the local mode changed to MODE1
                // or remain bridging in CHECK_CAN otherwise
                if (GetMode() == MODE1)
                {
                    SetStateTo(DO_MODE1);
                }
                else
                {
                    SetStateTo(CHECK_CAN);
                }
                break;
            }
            
            // ------------------------------
            // 3) DO_MODE1 => do your mode tasks
            // ------------------------------
            case(DO_MODE1):
                // Perform mode 1 tasks, then go back to bridging
                // If you still want bridging in DO_MODE1,
                // replicate the bridging logic above or do a more advanced approach
                SetStateTo(CHECK_CAN);
                break;
            
            // ------------------------------
            // 4) Invalid state => fallback
            // ------------------------------
            default:
                err = ERROR_INVALID_STATE;
                SetStateTo(UNINIT);
                break;
        }
        
        // ------------------------------
        // Handle any errors
        // ------------------------------
        if (err) {
            DisplayErrorCode(err);
        }
        
        // ------------------------------
        // Debugging over UART
        // ------------------------------
        if (DBG_UART_SpiUartGetRxBufferSize()) {
            DebugPrint(DBG_UART_UartGetByte());
        }
        
        // Simple delay before looping again
        CyDelay(100);
    }
}

void Initialize(void) {
    CyGlobalIntEnable; /* Enable global interrupts. LED arrays need this first */
    
    address = getSerialAddress();
    
    DBG_UART_Start();
    sprintf(txData, "Dip Addr: %x \r\n", address);
    Print(txData);
    
    LED_DBG1_Write(0);
   
    InitCAN(0x4, (int)address);
    
    CyDelay(100); // delay between the two 
    
    LED_DBG2_Write(0);
    InitCAN2(0x4, (int)address);
    
    Timer_Period_Reset_Start();
    
    // isr_Button_1_StartEx(Button_1_Handler);
    // isr_Period_Reset_StartEx(Period_Reset_Handler);
}

void DebugPrint(char input) {
    switch(input) {
        case 'f':
            sprintf(txData, "Mode: %x State:%x \r\n", GetMode(), GetState());
            break;
        case 'x':
            sprintf(txData, "bruh\r\n");
            break;
        default:
            sprintf(txData, "what\r\n");
            break;
    }
    Print(txData);
}

int getSerialAddress() {
    int address = 0;
    
    if (DIP1_Read()==0) address += 1;
    if (DIP2_Read()==0) address += 2;
    if (DIP3_Read()==0) address += 4;
    if (DIP4_Read()==0) address += 8;
    
    if (address == 0)
        address = DEVICE_SERIAL_TELEM_LOCALIZATION;

    return address;
}

void DisplayErrorCode(uint8_t code) {    
    ERROR_time_LED = 0;
    LED_ERR_Write(ON);
    
    sprintf(txData, "Error %X\r\n", code);
    Print(txData);
    
    switch(code)
    {
        case ERROR_INVALID_TTC:
            Print("Cannot Send That Data Type!\n\r");
            break;
        default:
            //some error
            break;
    }
}


// write methods that takes care of different address bytes between CAN_FD and CAN over here.
// assuming that CAN_FD is 64 bytes with CAN is 8 bytes
void ConvertCANToFD(CANPacket *packetCAN, CANPacket *packetFD)
{
    if (!packetCAN || !packetFD) return;
    
    // 1) ID Assignment
    //    The 11‐bit ID from Classical CAN is placed in the lower bits.
    //    If FD needs extended ID usage, you can set a flag or place
    //    the 11 bits in some portion of a 29‐bit ID. This example just copies them directly.
    packetFD->id = (packetCAN->id & CAN_ID_MASK);

    // 2) DLC
    //    Classical CAN is max 8. FD can handle 0..64, so we can copy directly.
    packetFD->dlc = packetCAN->dlc; // not just copying over the CAN_FD
    
    
    // broadcast packet in CAN wiki 
    // research more on the moteus CAN_FD protocols, address bits and etc.
    // set speed packet, write out the list of useful moteus packet ids
    // 

    // 3) Data
    memcpy(packetFD->data, packetCAN->data, packetCAN->dlc);

    // 4) Clear out the remainder (optional)
    for (uint8_t i = packetCAN->dlc; i < FD_MAX_PAYLOAD; i++) {
        packetFD->data[i] = 0;
    }
}


void ConvertFDToCAN(CANPacket *packetFD, CANPacket *packetCAN)
{
    if (!packetFD || !packetCAN) return;

    // 1) ID Remapping
    //    Naive approach: just take the lower 11 bits of the FD ID
    //    (If FD uses extended IDs, bits above 11 are lost here unless you store them elsewhere)
    packetCAN->id = (packetFD->id & CAN_ID_MASK);

    // 2) DLC Truncation
    uint8_t truncatedDLC = packetFD->dlc;
    if (truncatedDLC > CAN_MAX_PAYLOAD) {
        truncatedDLC = CAN_MAX_PAYLOAD;
        // Optionally log or store that data was truncated
    }
    packetCAN->dlc = truncatedDLC;

    // 3) Copy the truncated bytes
    memcpy(packetCAN->data, packetFD->data, truncatedDLC);

    // 4) (Optional) Clear the remaining part of the CAN data array
    for (uint8_t i = truncatedDLC; i < CAN_MAX_PAYLOAD; i++) {
        packetCAN->data[i] = 0;
    }
}
/* [] END OF FILE */
