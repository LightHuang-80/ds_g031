/*
 * dynamixel.h
 *
 *  Created on: 2026年9月20日
 *      Author: Huang
 */

#ifndef SRC_PROCESS_DYNAMIXEL_H_
#define SRC_PROCESS_DYNAMIXEL_H_

/*

AX-12:
https://emanual.robotis.com/docs/en/dxl/ax/ax-12a/

PROTOCOL v1.0 (used by AX-12):
https://emanual.robotis.com/docs/en/dxl/protocol1/

PROTOCOL v2.0:
https://emanual.robotis.com/docs/en/dxl/protocol2/


###############################################################################
###############################################################################

Control Table of EEPROM Area :
-------------------------------------------------------------------------------

0   2  Model Number           Model Number                        R   12
2   1  Firmware Version       Firmware Version                    R   -
3   1  ID                     DYNAMIXEL ID                        RW  1
4   1  Baud Rate              Communication Speed                 RW  1
5   1  Return Delay Time      Response Delay Time                 RW  250
6   2  CW Angle Limit         Clockwise Angle Limit               RW  0
8   2  CCW Angle Limit        Counter-Clockwise Angle Limit       RW  1023
11  1  Temperature Limit      Maximum Internal Temperature Limit  RW  70
12  1  Min Voltage Limit      Minimum Input Voltage Limit         RW  60
13  1  Max Voltage Limit      Maximum Input Voltage Limit         RW  140
14  2  Max Torque             Maximun Torque                      RW  1023
16  1  Status Return Level    Select Types of Status Return       RW  2
17  1  Alarm LED              LED for Alarm                       RW  36
18  1  Shutdown               Shutdown Error Information          RW  36

Control Table of RAM Area :
-------------------------------------------------------------------------------

24  1  Torque Enable          Motor Torque On/Off           RW  0
25  1  LED                    Status LED On/Off             RW  0
26  1  CW Compliance Margin   CW Compliance Margin          RW  1
27  1  CCW Compliance Margin  CCW Compliance Margin         RW  1
28  1  CW Compliance Slope    CW Compliance Slope           RW  32
29  1  CCW Compliance Slope   CCW Compliance Slope          RW  32
30  2  Goal Position          Target Position               RW  -
32  2  Moving Speed           Moving Speed                  RW  -
34  2  Torque Limit           Torque Limit                  RW  Max Torque
36  2  Present Position       Present Position              R   -
38  2  Present Speed          Present Speed                 R   -
40  2  Present Load           Present Load                  R   -
42  1  Present Voltage        Present Voltage               R   -
43  1  Present Temperature    Present Temperature           R   -
44  1  Registered             If Instruction is registered  R   0
46  1  Moving                 Movement Status               R   0
47  1  Lock                   Locking EEPROM                RW  0
48  2  Punch                  Minimum Current Threshold     RW  32

###############################################################################
###############################################################################

*/

#include "main.h"

/////////////////// MACROS ///////////////////

// Only necessary when using protocol v2.0
#define GET_INT16_LSB(bytes) (uint8_t)(((uint16_t)(bytes) >> 0) & 0xFF)
#define GET_INT16_MSB(bytes) (uint8_t)(((uint16_t)(bytes) >> 8) & 0xFF)

#define MAKE_UINT16(x, p_lsb, p_hsb) {x = p_hsb; x <<= 8; x &= 0xFF00; x |= (p_lsb & 0x00FF); }
/////////////////// INSTRUCTIONS ///////////////////

#define PING                  0x01
#define READ                  0x02
#define WRITE                 0x03
#define REG_WRITE             0x04
#define ACTION                0x05
#define FACTORY_RESET         0x06
#define REBOOT                0x08
#define CLEAR                 0x10  // Exists only in protocol v2.0
#define CONTROL_TABLE_BACKUP  0x20  // Exists only in protocol v2.0
#define STATUS_RETURN         0x55	// Status(Return)
#define SYNC_READ             0x82  // Exists only in protocol v2.0
#define SYNC_WRITE            0x83
#define FAST_SYNC_READ        0x8A  // Exists only in protocol v2.0
#define BULK_READ             0x92
#define BULK_WRITE            0x93  // Exists only in protocol v2.0
#define FAST_BULK_READ        0x9A  // Exists only in protocol v2.0

/////////////////// ERRORS ///////////////////

// Protocole v1.0
#define INPUT_VOLTAGE_ERROR   0x01
#define ANGLE_LIMIT_ERROR     0x02
#define OVERHEATING_ERROR     0x04
#define RANGE_ERROR           0x08
#define CHECKSUL_ERROR        0x10
#define OVERLOAD_ERROR        0x20
#define INSTRUCTION_ERROR_V1  0x40

// Protocole v2.0
#define RESULT_FAIL           0x01
#define INSTRUCTION_ERROR_V2  0x02
#define CRC_ERROR             0x03
#define DATA_RANGE_ERROR      0x04
#define DATA_LENGTH_ERROR     0x05
#define DATA_LIMIT_ERROR      0x06
#define ACCESS_ERROR          0x07

#define FATAL_ERROR_BIT_MASK  0x80
#define ERROR_BIT_MASK        0x7F

/////////////////// CONTROL TABLE ///////////////////

// Specific to AX-12

// EEPROM AREA
#define ID_ADDR                  7
#define BAUDRATE_ADDR            8
#define REG_INST_ADDR            69
#define HARDWARE_ERR_ADDR        70

#define RETURN_DELAY_TIME_ADDR   9

#define CW_ANGLE_LIMIT_ADDR      48
#define CCW_ANGLE_LIMIT_ADDR     52

#define MAX_POSITION_LIMIT_ADDR  CW_ANGLE_LIMIT_ADDR
#define MIN_POSITION_LIMIT_ADDR  CCW_ANGLE_LIMIT_ADDR

#define INPUT_VOL_ADDR           144

#define POSITION_MODE            3
#define VELOCITY_MODE            2

#define OPERATION_MODE_ADDR      11
#define HOME_OFFSET_ADDR         20

#define MIN_VOLTAGE_LIMIT_ADDR  12
#define MAX_VOLTAGE_LIMIT_ADDR  13
#define MAX_TORQUE_ADDR         14
#define STATUS_RET_LEVEL_ADDR   16
#define ALARM_LED_ADDR          17
#define SHUTDOWN_ADDR           18

// RAM AREA

#define TORQUE_ENABLE_ADDR      64
#define LED_ADDR                65
#define STARTUP_ADDR            60
#define CW_COMP_MARGIN_ADDR     26
#define CCW_COMP_MARGIN_ADDR    27
#define CW_COMP_SLOPE_ADDR      28
#define CCW_COMP_SLOPE_ADDR     29
#define DRIVE_MODE_ADDR         10

#define GOAL_POSITION_ADDR      116
#define GOAL_CURRENT_ADDR       102
#define GOAL_VELOCITY_ADDR      104

#define MOVING_SPEED_ADDR       32
#define TORQUE_LIMIT_ADDR       34
#define PRESENT_POSITION_ADDR   36
#define PRESENT_SPEED_ADDR      38
#define PRESENT_LOAD_ADDR       40
#define PRESENT_VOLTAGE_ADDR    42
#define PRESENT_TEMP_ADDR       43
#define REGISTERED_ADDR         44
#define MOVING_ADDR             46
#define LOCK_ADDR               47
#define PUNCH_ADDR              48

/////////////////// CONSTANTS ///////////////////

// Comm
#define DYNAMIXEL_ID          0x01
#define BROADCAST_ID          0xFE
#define DEFAULT_TIMEOUT        500
#define MAX_BAUD_RATE      1000000

// Data
#define MAX_TORQUE            1023
#define MAX_MOVING_SPEED      1023

#define LOCK                     1
#define JOINT_MODE               0
#define WHEEL_MODE               1

// action add
extern uint16_t   reg_action_addr;

#define SYNC_NONE              0
#define SYNC_READ_TYPE	       1
#define BULK_READ_TYPE         2
#define FAST_SYNC_READ_TYPE    3

#define SYNC_READ_WAIT         1
#define SYNC_READ_RESPOND      2
#define FAST_SYNC_READ_WAIT    3
#define FAST_SYNC_READ_RESPOND 4

#define BULK_READ_WAIT         0x11
#define BULK_READ_RESPOND      0x12

#define PARAMS_MAX_LENGTH       32

typedef struct sync_pkt_data_ {
	uint8_t  devices_num;
	uint8_t  devices_chain[31];
	uint16_t reg_addr;                // register address
	uint8_t  sync_type;               // sync type:  sync read, bulk read, fast sync read
	uint8_t  sync_state;              // sync state: wait or respond
	uint16_t data_len;				  // data length
	uint8_t  resp_index;               // device index in chain
	uint8_t  status_data[PARAMS_MAX_LENGTH];
}sync_pkt_data_t;

typedef struct reg_pkt_data_ {
	uint16_t reg_addr;
	uint16_t data_len;
	uint8_t  reg_data[8];
}reg_pkt_data_t;

extern sync_pkt_data_t   sync_pkt;
extern reg_pkt_data_t    reg_pkt;

/////////////////// COMMON FUNCTIONS ///////////////////

// Send dynamixel packet
void send_packet_v2(UART_HandleTypeDef *huart, uint8_t id, uint8_t inst, uint8_t *params, uint16_t params_len);

// Parse dynamixel packet
uint8_t parse_status_packet_v2(uint8_t *packet, uint32_t packet_len, uint8_t *id, uint8_t *params, uint16_t *params_len, uint8_t *error, uint8_t *crc_check);

// Calculate CRC
uint16_t update_crc(uint16_t crc_accum, uint8_t *data_blk_ptr, uint16_t data_blk_size);

// Sync Write into dynamixel registers
void dynamixel_sync_write_v2(UART_HandleTypeDef *huart, uint8_t *ids, uint16_t address, uint8_t *data, uint16_t data_len, uint8_t nb_ids);

/////////////////// SPECIFIC INSTRUCTION FUNCTIONS ///////////////////

// Ping
void dynamixel_ping(UART_HandleTypeDef *huart, uint8_t id);

uint8_t dynamixel_read_registered(UART_HandleTypeDef *huart, uint8_t id);

void dynamixel_read(UART_HandleTypeDef *huart, uint8_t* packet, uint16_t packet_len);
uint8_t dynamixel_write(UART_HandleTypeDef *huart, uint8_t* packet, uint16_t packet_len);
void dynamixel_regwrite(UART_HandleTypeDef *huart, uint8_t* packet, uint16_t packet_len);
void dynamixel_action(UART_HandleTypeDef *huart, uint8_t* packet, uint16_t packet_len);

void dynamixel_fast_status_resp(UART_HandleTypeDef *huart, uint16_t crc_sum);
void dynamixel_status_resp(UART_HandleTypeDef *huart, uint8_t resp_id);
void dynamixel_first_fast_status_resp(UART_HandleTypeDef *huart, uint8_t resp_id);

/////////////////// UTILITY FUNCTIONS ///////////////////
uint16_t verify_dxl_packet(uint8_t* pkt, uint16_t data_len, uint16_t* pkt_len);

uint8_t baudrate_to_value(uint32_t baudrate);
uint32_t value_to_baudrate(uint8_t value);
uint16_t pos_to_value(float pos_deg);
uint16_t vel_to_value(float speed_deg_s);
uint16_t vel_to_value_wheel_mode(float speed_pct);
float value_to_load(uint16_t value);

uint16_t update_crc(uint16_t crc_accum,
                    uint8_t *data_blk_ptr,
                    uint16_t data_blk_size);

#endif /* SRC_PROCESS_DYNAMIXEL_H_ */
