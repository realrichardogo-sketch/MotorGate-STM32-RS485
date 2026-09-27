#ifndef G1_MODBUS_H
#define G1_MODBUS_H
#include <stdint.h>
extern volatile uint32_t G1_frames_ok;
extern volatile uint32_t G1_frames_bad;
extern volatile uint32_t G1_timeouts;
extern volatile uint32_t G1_tx_ok;
extern volatile uint32_t G1_tx_errors;
extern volatile uint32_t G1_rx_bytes_last;
extern volatile uint16_t G1_reg[10];
/* One-shot command entry shared by the UART console and ST-LINK debug:
   1 ARM, 2 START, 3 STOP, 4 ARM+START.
   Defaults to zero after every reset; no automatic motion on power-up. */
extern volatile uint32_t G2_command_request;
extern volatile uint32_t G2_cmd_sent;
extern volatile uint32_t G2_cmd_acked;
extern volatile uint32_t G2_cmd_failed;
void G1_Log(const char *status);
void G3_SerialInit(void);
void G3_SerialRxIRQ(void);
void G3_SerialPoll(void);
void G1_PollMotor(void);
void G2_ProcessCommand(void);
#endif
