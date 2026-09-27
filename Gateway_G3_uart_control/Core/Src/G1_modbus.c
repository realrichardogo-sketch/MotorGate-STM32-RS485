#include "G1_modbus.h"
#include "usart.h"
#include <stdio.h>
#include <string.h>

/* USART2/RS485 talks to the M18 motor board; USART1 is the PC console. */
volatile uint32_t G1_frames_ok = 0U;
volatile uint32_t G1_frames_bad = 0U;
volatile uint32_t G1_timeouts = 0U;
volatile uint32_t G1_tx_ok = 0U;
volatile uint32_t G1_tx_errors = 0U;
volatile uint32_t G1_rx_bytes_last = 0U;
volatile uint16_t G1_reg[10] = {0};
volatile uint32_t G2_command_request = 0U;
volatile uint32_t G2_cmd_sent = 0U;
volatile uint32_t G2_cmd_acked = 0U;
volatile uint32_t G2_cmd_failed = 0U;

#define G2_REMOTE_ARM 0xA55AU
#define G2_REMOTE_START 0x5AA5U
#define G2_REMOTE_STOP 0x0000U
/* Pulse request advances only after the next successful FC04 read of ARMED. */
static uint8_t g2_pulse_armed = 0U;
static uint32_t g2_arm_tick = 0U;
static uint32_t g2_arm_frames = 0U;

/* USART1 RX is interrupt driven because an RS485 read can block the main loop.
   On overflow, discard through a line ending so a truncated command cannot run. */
#define G3_RX_CAPACITY 64U
static volatile uint8_t g3_rx[G3_RX_CAPACITY];
static volatile uint8_t g3_rx_head = 0U;
static volatile uint8_t g3_rx_tail = 0U;
static volatile uint8_t g3_rx_fault = 0U;
volatile uint32_t G3_rx_dropped = 0U;
volatile uint32_t G3_bad_lines = 0U;
static uint8_t g3_discard_line = 0U;
static char g3_line[16];
static uint8_t g3_line_length = 0U;

void G3_SerialInit(void)
{
    /* USART1 IRQ is already enabled by MX_USART1_UART_Init(). */
    (void)USART1->SR;
    (void)USART1->DR;
    __HAL_UART_ENABLE_IT(&huart1, UART_IT_RXNE);
}

void G3_SerialRxIRQ(void)
{
    uint32_t sr = USART1->SR;
    if ((sr & (USART_SR_RXNE | USART_SR_ORE | USART_SR_FE | USART_SR_NE)) != 0U)
    {
        uint8_t byte = (uint8_t)USART1->DR; /* SR, then DR also clears ORE. */
        uint8_t next = (uint8_t)((g3_rx_head + 1U) % G3_RX_CAPACITY);
        if ((sr & (USART_SR_ORE | USART_SR_FE | USART_SR_NE)) != 0U ||
            next == g3_rx_tail)
        {
            g3_rx_fault = 1U;
            ++G3_rx_dropped;
        }
        else
        {
            g3_rx[g3_rx_head] = byte;
            g3_rx_head = next;
        }
    }
}

static void G3_ConsumeLine(void)
{
    uint32_t command;
    g3_line[g3_line_length] = '\0';
    if (g3_line_length == 0U)
        return;
    if (strcmp(g3_line, "STATUS") == 0)
    {
        G1_Log("STATUS");
        return;
    }
    if (strcmp(g3_line, "STOP") == 0)
        command = 3U;
    else if (strcmp(g3_line, "ARM") == 0)
        command = 1U;
    else if (strcmp(g3_line, "START") == 0)
        command = 2U;
    else if (strcmp(g3_line, "PULSE") == 0)
        command = 4U;
    else
    {
        ++G3_bad_lines;
        G1_Log("SERIAL BAD");
        return;
    }
    if (command != 3U && (G2_command_request != 0U || g2_pulse_armed))
    {
        G1_Log("SERIAL BUSY");
        return;
    }
    /* STOP always supersedes a pending command. The existing G2 path validates
       M18 and implements the ARM -> confirmed ARMED -> START sequence. */
    G2_command_request = command;
    G1_Log("SERIAL QUEUED");
}

void G3_SerialPoll(void)
{
    while (g3_rx_tail != g3_rx_head)
    {
        uint8_t byte;
        if (g3_rx_fault)
        {
            g3_rx_fault = 0U;
            g3_discard_line = 1U;
            g3_line_length = 0U;
        }
        byte = g3_rx[g3_rx_tail];
        g3_rx_tail = (uint8_t)((g3_rx_tail + 1U) % G3_RX_CAPACITY);
        if (byte == '\r' || byte == '\n')
        {
            if (!g3_discard_line)
                G3_ConsumeLine();
            g3_discard_line = 0U;
            g3_line_length = 0U;
        }
        else if (!g3_discard_line)
        {
            if (byte < (uint8_t)'A' || byte > (uint8_t)'Z' ||
                g3_line_length + 1U >= sizeof(g3_line))
            {
                g3_discard_line = 1U;
                g3_line_length = 0U;
                ++G3_bad_lines;
            }
            else
                g3_line[g3_line_length++] = (char)byte;
        }
    }
}

static uint16_t G1_CRC16(const uint8_t *data, uint16_t size)
{
    uint16_t crc = 0xFFFFU;
    uint16_t i;
    uint8_t bit;
    for (i = 0U; i < size; ++i)
    {
        crc ^= data[i];
        for (bit = 0U; bit < 8U; ++bit)
            crc = (crc & 1U) ? (uint16_t)((crc >> 1U) ^ 0xA001U)
                             : (uint16_t)(crc >> 1U);
    }
    return crc;
}

void G1_Log(const char *status)
{
    char line[96];
    int n = snprintf(line, sizeof(line),
                     "G1 %s ok=%lu bad=%lu timeout=%lu ver=%u state=%u sec=%u\r\n",
                     status, (unsigned long)G1_frames_ok,
                     (unsigned long)G1_frames_bad, (unsigned long)G1_timeouts,
                     (unsigned)G1_reg[0], (unsigned)G1_reg[1],
                     (unsigned)G1_reg[9]);
    if (n > 0)
    {
        uint16_t len = (n < (int)sizeof(line)) ? (uint16_t)n
                                                 : (uint16_t)(sizeof(line) - 1U);
        (void)HAL_UART_Transmit(&huart1, (uint8_t *)line, len, 100U);
    }
}

/* FC06 writes only holding offset 0; a full echoed frame plus CRC is required
   for an acknowledged command. The following FC04 poll confirms motor state. */
static uint8_t G2_SendControl(uint16_t command)
{
    uint8_t request[8] = {1U, 6U, 0U, 0U, 0U, 0U, 0U, 0U};
    uint8_t response[8];
    uint16_t crc;
    uint8_t i;
    request[4] = (uint8_t)(command >> 8U);
    request[5] = (uint8_t)command;
    crc = G1_CRC16(request, 6U);
    request[6] = (uint8_t)crc;
    request[7] = (uint8_t)(crc >> 8U);
    ++G2_cmd_sent;
    if (HAL_UART_Transmit(&huart2, request, sizeof(request), 100U) != HAL_OK ||
        HAL_UART_Receive(&huart2, response, sizeof(response), 1500U) != HAL_OK)
    {
        ++G2_cmd_failed;
        G1_Log("CMD TIMEOUT");
        return 0U;
    }
    for (i = 0U; i < sizeof(request); ++i)
    {
        if (response[i] != request[i])
        {
            ++G2_cmd_failed;
            G1_Log("CMD BAD");
            return 0U;
        }
    }
    ++G2_cmd_acked;
    G1_Log("CMD ACK");
    return 1U;
}

void G2_ProcessCommand(void)
{
    uint32_t command = G2_command_request;
    if (command == 0U)
    {
        if (g2_pulse_armed)
        {
            if ((uint32_t)(HAL_GetTick() - g2_arm_tick) > 3500U)
            {
                g2_pulse_armed = 0U;
                ++G2_cmd_failed;
                G1_Log("CMD ARM EXPIRED");
            }
            else if (G1_frames_ok > g2_arm_frames && G1_reg[1] == 1U)
            {
                g2_pulse_armed = 0U;
                (void)G2_SendControl(G2_REMOTE_START);
            }
        }
        return;
    }
    G2_command_request = 0U; /* Consume each debug request exactly once. */
    g2_pulse_armed = 0U; /* A newer operator command replaces an armed pulse. */
    if (command == 3U)
    {
        (void)G2_SendControl(G2_REMOTE_STOP);
        return;
    }
    /* A legacy M13 board must never be sent a control command. */
    if (G1_frames_ok == 0U || G1_reg[0] != 18U)
    {
        ++G2_cmd_failed;
        G1_Log("CMD NO M18");
        return;
    }
    if (command == 1U)
        (void)G2_SendControl(G2_REMOTE_ARM);
    else if (command == 2U)
        (void)G2_SendControl(G2_REMOTE_START);
    else if (command == 4U)
    {
        if (G2_SendControl(G2_REMOTE_ARM))
        {
            g2_pulse_armed = 1U;
            g2_arm_tick = HAL_GetTick();
            g2_arm_frames = G1_frames_ok;
        }
    }
    else
    {
        ++G2_cmd_failed;
        G1_Log("CMD UNKNOWN");
    }
}

void G1_PollMotor(void)
{
    uint8_t request[8] = {1U, 4U, 0U, 0U, 0U, 10U, 0U, 0U};
    uint8_t response[25];
    uint16_t crc = G1_CRC16(request, 6U);
    uint8_t i;
    request[6] = (uint8_t)crc;
    request[7] = (uint8_t)(crc >> 8U);

    G1_rx_bytes_last = 0U;
    if (HAL_UART_Transmit(&huart2, request, sizeof(request), 100U) != HAL_OK)
    {
        ++G1_tx_errors;
        ++G1_timeouts;
        G1_Log("TX ERROR");
        return;
    }
    ++G1_tx_ok;
    /* M13 refreshes its OLED a byte at a time over 100 kHz I2C. A request
       arriving during that refresh can wait substantially longer than 300 ms
       before eMBPoll() runs. Track partial receives to separate link faults
       from a late or malformed Modbus reply. */
    if (HAL_UART_Receive(&huart2, response, sizeof(response), 1500U) != HAL_OK)
    {
        G1_rx_bytes_last = sizeof(response) - huart2.RxXferCount;
        ++G1_timeouts;
        G1_Log("RX TIMEOUT");
        return;
    }
    G1_rx_bytes_last = sizeof(response);
    crc = G1_CRC16(response, 23U);
    if (response[0] != 1U || response[1] != 4U || response[2] != 20U ||
        response[23] != (uint8_t)crc ||
        response[24] != (uint8_t)(crc >> 8U) ||
        response[3] != 0U || response[4] != 18U)
    {
        ++G1_frames_bad;
        G1_Log("BAD");
        return;
    }
    for (i = 0U; i < 10U; ++i)
        G1_reg[i] = (uint16_t)(((uint16_t)response[3U + 2U * i] << 8U) |
                               response[4U + 2U * i]);
    ++G1_frames_ok;
    G1_Log("OK");
}
