#ifndef MOTOR_CONFIG_H
#define MOTOR_CONFIG_H

#include <string>
#include <vector>

uint64_t keyaConfigID = 0x06000591;
uint8_t keyaEnterConfig[] = { 0xFA, 0xFA, 0x00, 0x00 };
uint8_t keyaSet5Amp[] = { 0xBB, 0xBB, 0x00, 0x00, 0x00, 0x03, 0x00, 0x05 }; // update parameter 3 with value 5 (Max Current)
uint8_t keyaStoreEEPROM[] = { 0xFA, 0xFA, 0x00, 0x08 };
uint8_t keyaExitConfig[] = { 0xFA, 0xFA, 0x00, 0xAA };

// template <size_t N>
// void keyaConfig(uint8_t(&command)[N])
// {
// 	if (keyaDetected)
// 	{
// 		CAN_message_t KeyaBusSendData;
// 		KeyaBusSendData.id = keyaConfigID;
// 		KeyaBusSendData.flags.extended = true;
// 		KeyaBusSendData.len = N;
// 		memcpy(KeyaBusSendData.buf, command, N);
// 		canbus3.write(KeyaBusSendData);
// 		delay(1000);
// 	}
// }
namespace MotorController {

    // Structure to represent a single parameter entry
    struct ParamEntry {
        int id;                 // No (Number column)
        std::string name;       // Name (Tag column)
        std::string rangeInfo;  // Setup Range (Param column)
    };

    // Constant list of all parameters parsed from the interface
    const std::vector<ParamEntry> PARAM_MAP = {
        {0, "TAG", "Non-Modified Content"},
        {1, "Motor Poles", "Even Number of 2-32"},
        {2, "Rated Speed", "80-3500"},
        {3, "Max Current", "10-500"},
        {4, "Encoder PPR", "1000-50000"},
        {5, "Current Kp", "0.001-2"},
        {6, "Current Ki", "0.001-2"},
        {7, "Speed Kp", "0.001-2"},
        {8, "Speed Ki", "0.001-2"},
        {9, "Position Kp", "0.001-2"},
        {10, "Position Ki", "0.001-2"},
        {11, "Position Kd", "0.001-2"},
        {12, "Position Kc", "0.001-2"},
        {13, "Acceleration Time", "1-200; 0.1s-20s"},
        {14, "Position", "Slow Down in Advance 20-100"},
        {15, "Magnetic", "0.001-0.999"},
        {18, "System", "CAN-ID (decimalism)"},
        {19, "Control Ways", "1-Analog 2-CAN 3-Serial Port 4-RC 5-CANOPEN"},
        {20, "Control Mode", "1-speed 2-Torque 3-Pos 4-Pos"},
        {21, "BPS", "CAN BPS 1-125K 2-250K 3-500K 4-1M"},
        {22, "Position", "1-Encoder 2-Hall 3-Magnetic"},
        {23, "Over Voltage", "Over Voltage Setting"},
        {24, "Less Voltage", "Less Voltage Setting"},
        {25, "Motor Temp", "Motor Temp Protection Setting"},
        {26, "Direction", "1-Motor 1 Reverse; 2-Motor 2"},
        {27, "Brake Time", "1-30; 0.1s-3s"},
        {28, "Over", "1-20; 1s-20s"},
        {29, "Hall status", "Hall reverse"},
        {30, "Deceleration time", "1-200; 0.1s-20s"},
        {31, "Spare", "Spare"},
        {32, "Spare", "Spare"}
    };

    // Enum for programmatic access to IDs
    enum ParamID {
        TAG = 0,
        MOTOR_POLES = 1,
        RATED_SPEED = 2,
        MAX_CURRENT = 3,
        ENCODER_PPR = 4,
        CURRENT_KP = 5,
        CURRENT_KI = 6,
        SPEED_KP = 7,
        SPEED_KI = 8,
        POSITION_KP = 9,
        POSITION_KI = 10,
        POSITION_KD = 11,
        POSITION_KC = 12,
        ACCEL_TIME = 13,
        POSITION_LIMIT = 14,
        MAGNETIC = 15,
        SYSTEM_ID = 18,
        CONTROL_WAYS = 19,
        CONTROL_MODE = 20,
        CAN_BPS = 21,
        POS_SENSE_TYPE = 22,
        OVER_VOLTAGE = 23,
        LESS_VOLTAGE = 24,
        MOTOR_TEMP = 25,
        DIRECTION = 26,
        BRAKE_TIME = 27,
        OVER_LOAD = 28,
        HALL_STATUS = 29,
        DECEL_TIME = 30
    };
}

#endif // MOTOR_CONFIG_H