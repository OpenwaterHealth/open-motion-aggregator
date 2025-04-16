/*
 * crosslink.c
 *
 *  Created on: Aug 6, 2024
 *      Author: gvigelet
 */

#include "crosslink.h"
#include "common.h"
#include "main.h"
#include "utils.h"

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdbool.h>
#include <stdint.h>

#define I2C_ADDRESS 0x40  // Replace with your FPGA's I2C address
#define BITSTREAM_CHUNK_SIZE 8192
#define CMD_INITIATE 0xC0
#define CMD_ISSUE 0x74
#define CMD_ENABLE_SRAM 0xC6
#define CMD_ERASE_SRAM 0x26
#define CMD_READ_STATUS 0x3C
#define CMD_PROGRAM_SRAM 0x46
#define CMD_VERIFY_USERCODE 0xC8
#define CMD_DISABLE 0x26

volatile uint8_t txComplete = 0;
volatile uint8_t rxComplete = 0;
volatile uint8_t i2cError = 0;

unsigned char activation_key[5] = {0xFF, 0xA4, 0xC6, 0xF4, 0x8A};
unsigned char write_buf[4];
unsigned char read_buf[4];

extern uint8_t bitstream_buffer[];
extern uint32_t bitstream_len;


static int xi2c_write_bytes(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint8_t *data, uint16_t length) {
    return HAL_I2C_Master_Transmit(hi2c, DevAddress << 1, data, length, HAL_MAX_DELAY);
}

static int xi2c_write_and_read(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint8_t *wbuf, uint16_t wlen, uint8_t *rbuf, uint16_t rlen) {
    txComplete = 0;
    rxComplete = 0;
    i2cError = 0;

    if (HAL_I2C_Master_Seq_Transmit_IT(hi2c, DevAddress << 1, wbuf, wlen, I2C_FIRST_FRAME) != HAL_OK)
        return -1;

    // Wait for the transmission to complete
    while (!txComplete && !i2cError) {}

    if (i2cError)
    {
        return HAL_ERROR;
    }


    if (HAL_I2C_Master_Seq_Receive_IT(hi2c, DevAddress << 1, rbuf, rlen, I2C_LAST_FRAME) != HAL_OK)
        return -1;

    // Wait for the reception to complete
    while (!rxComplete && !i2cError) {}

    if (i2cError)
    {
        return HAL_ERROR;
    }

    return HAL_OK;
}

static HAL_StatusTypeDef xi2c_write_long(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint8_t *cmd, int cmd_len, uint8_t *data, size_t data_len) {
	HAL_StatusTypeDef ret;
    size_t offset = 0;
    uint32_t frame_flag;
    size_t total_len = data_len+cmd_len;
    int num_chunks = (total_len + BITSTREAM_CHUNK_SIZE - 1) / BITSTREAM_CHUNK_SIZE;  // Calculate number of chunks
    uint8_t *pData;
	uint16_t datalen;

	memset(bitstream_buffer, 0, MAX_BITSTREAM_SIZE);
    memcpy(bitstream_buffer, cmd, cmd_len);  // copy the long write command in
    memcpy(bitstream_buffer + cmd_len, data, data_len);

    for (int i = 0; i < num_chunks; i++) {
        size_t current_chunk_size = (total_len - offset > BITSTREAM_CHUNK_SIZE)
                                     ? BITSTREAM_CHUNK_SIZE
                                     : (total_len - offset);

        // Determine frame flags
        if (i == 0 && num_chunks == 1) {
            frame_flag = I2C_FIRST_AND_LAST_FRAME;
        } else if (i == 0) {
            frame_flag = I2C_FIRST_AND_NEXT_FRAME;
        } else if (i == num_chunks - 1) {
            frame_flag = I2C_LAST_FRAME;
        } else {
            frame_flag = I2C_NEXT_FRAME;
        }

        // Check if the I2C peripheral is ready
        while (HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_READY) {
            //if(verbose_on) printf("I2C busy, waiting...\r\n");
            HAL_Delay(1);  // Add a small delay to avoid busy looping
        }

        pData = (uint8_t*)&bitstream_buffer[offset];
        datalen = (uint16_t)current_chunk_size;

        // Reset completion flags
        txComplete = 0;
        i2cError = 0;

        // Transmit the current chunk
        ret = HAL_I2C_Master_Seq_Transmit_IT(hi2c, DevAddress << 1, pData, datalen, frame_flag);
        if (ret != HAL_OK) {
        	printf("++++> i2c_write_long HAL TX HAL_StatusTypeDef: 0%04X I2C_ERROR: 0%04lX\r\n", ret, hi2c1.ErrorCode);
            return ret; // Return if any transmission fails
        }

        // Wait for the transmission to complete
        while (!txComplete && !i2cError) {}

        if (i2cError)
        {
            return HAL_ERROR;
        }

        offset += current_chunk_size;
    }

    return HAL_OK;
}

int fpga_send_activation(I2C_HandleTypeDef *hi2c, uint16_t DevAddress)
{
	// Step 1: Initialize
	if(verbose_on) printf("Step 1: Send Activation Key\r\n");
	if (xi2c_write_bytes(hi2c, DevAddress, activation_key, 5) != HAL_OK) {
		if(verbose_on) printf("failed to send activation key\r\n");
	    return 1;  // Exit if writing activation key fails
	}

	return 0;
}

int fpga_checkid(I2C_HandleTypeDef *hi2c, uint16_t DevAddress)
{
    // Step 2: Check IDCODE (Optional)
    if(verbose_on) printf("Step 2: Check IDCODE (Optional)\r\n");
    write_buf[0] = 0xE0; write_buf[1] = 0x00; write_buf[2] = 0x00; write_buf[3] = 0x00;
    if (xi2c_write_and_read(hi2c, DevAddress, write_buf, 4, read_buf, 4) != HAL_OK) {
        if(verbose_on) printf("failed to send IDCODE Command\r\n");
        return 1;  // Exit if write/read fails
    }

    if(verbose_on) print_hex_buf("IDCODE", read_buf, 4);
	return 0;
}

int fpga_enter_sram_prog_mode(I2C_HandleTypeDef *hi2c, uint16_t DevAddress)
{
    // Step 3: Enable SRAM Programming Mode
    if(verbose_on) printf("Step 3: Enable SRAM Programming Mode\r\n");
    write_buf[0] = 0xC6; write_buf[1] = 0x00; write_buf[2] = 0x00; write_buf[3] = 0x00;
    if (xi2c_write_bytes(hi2c, DevAddress, write_buf, 4) != HAL_OK) {
        if(verbose_on) printf("failed to send SRAM Command\r\n");
        return 1;  // Exit if writing fails
    }
    HAL_Delay(1);
    return 0;
}

int fpga_exit_prog_mode(I2C_HandleTypeDef *hi2c, uint16_t DevAddress)
{

    // Step 11: Exit Programming Mode
    if(verbose_on) printf("Step 10: Exit Programming Mode\r\n");
    write_buf[0] = 0x26; write_buf[1] = 0x00; write_buf[2] = 0x00; write_buf[3] = 0x00;
    if (xi2c_write_bytes(hi2c, DevAddress, write_buf, 4) != HAL_OK) {
        if(verbose_on) printf("failed to send Exit Command\r\n");
        return 1;  // Exit if writing fails
    }

    return 0;
}

uint32_t fpga_read_status(I2C_HandleTypeDef *hi2c, uint16_t DevAddress)
{
    // Step 5: Read Status Register
    if(verbose_on) printf("Read Status Register\r\n");
    write_buf[0] = 0x3C; write_buf[1] = 0x00; write_buf[2] = 0x00; write_buf[3] = 0x00;
    if (xi2c_write_and_read(hi2c, DevAddress, write_buf, 4, read_buf, 4) != HAL_OK) {
        if(verbose_on) printf("failed to send READ Status Command\r\n");
        return 1;  // Exit if write/read fails
    }

    if(verbose_on) print_hex_buf("Erase Status", read_buf, 4);
    return 0;

}

uint32_t fpga_read_usercode(I2C_HandleTypeDef *hi2c, uint16_t DevAddress)
{
    // Step 9: Read USERCODE (Optional)
    if(verbose_on) printf("Step 9: Verify USERCODE (Optional)\r\n");
    write_buf[0] = 0xC0; write_buf[1] = 0x00; write_buf[2] = 0x00; write_buf[3] = 0x00;
    if (xi2c_write_and_read(hi2c, DevAddress, write_buf, 4, read_buf, 4) != HAL_OK) {
        if(verbose_on) printf("failed to send USERCODE Command\r\n");
        return 1;  // Exit if write/read fails
    }

    if(verbose_on) print_hex_buf("User Register", read_buf, 4);
    return 0;
}

int fpga_erase_sram(I2C_HandleTypeDef *hi2c, uint16_t DevAddress)
{
    // Step 4: Erase SRAM
    if(verbose_on) printf("Step 4: Erase SRAM...");
    write_buf[0] = 0x0E; write_buf[1] = 0x00; write_buf[2] = 0x00; write_buf[3] = 0x00;
    if (xi2c_write_bytes(hi2c, DevAddress, write_buf, 4) != HAL_OK) {
        if(verbose_on) printf("\r\nFAILED to send SRAM Erase Command\r\n");
        return 1;  // Exit if writing fails
    }
    if(verbose_on) printf("COMPLETED\r\n");
    return 0;
}

int fpga_program_sram(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, bool rom_bitstream, uint8_t* pData, uint32_t Data_Len)
{
    if(verbose_on) printf("Program SRAM\r\n");
    write_buf[0] = 0x46; write_buf[1] = 0x00; write_buf[2] = 0x00; write_buf[3] = 0x00;
    if (xi2c_write_bytes(hi2c, DevAddress, write_buf, 4)!= HAL_OK) {
        if(verbose_on) printf("failed to send Exit Command\r\n");
        return 1;  // Exit if writing fails
    }

    if(rom_bitstream)
    {
    	write_buf[0] = 0x7A; write_buf[1] = 0x00; write_buf[2] = 0x00; write_buf[3] = 0x00;
    	xi2c_write_long(hi2c, DevAddress, write_buf, 4, (uint8_t *)bitstream_buffer, bitstream_len);
    }
    else
    {
    	return 1;
    }

    return 0;
}


int fpga_configure(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin) {
	int ret_status = 0;
	if(verbose_on) printf("Starting FPGA configuration...\r\n");

    // Set GPIO HIGH
    HAL_GPIO_WritePin(GPIOx, GPIO_Pin, GPIO_PIN_SET);
    HAL_Delay(250);

    // Set GPIO LOW
    HAL_GPIO_WritePin(GPIOx, GPIO_Pin, GPIO_PIN_RESET);
    HAL_Delay(250);

    HAL_GPIO_WritePin(GPIOx, GPIO_Pin, GPIO_PIN_SET);
    HAL_Delay(250);

    // Set GPIO LOW
    HAL_GPIO_WritePin(GPIOx, GPIO_Pin, GPIO_PIN_RESET);
    HAL_Delay(1000);

    // Activation Key
    uint8_t activation_key[] = {0xFF, 0xA4, 0xC6, 0xF4, 0x8A};
    xi2c_write_bytes(hi2c, DevAddress, activation_key, 5);
    HAL_GPIO_WritePin(GPIOx, GPIO_Pin, GPIO_PIN_SET);
    HAL_Delay(10);

    // IDCODE
    memset(read_buf, 0, 4);
    memcpy(write_buf, (uint8_t[]){0xE0,0x00,0x00,0x00}, 4);
    xi2c_write_and_read(hi2c, DevAddress, write_buf, 4, read_buf, 4);
    if(verbose_on) print_hex_buf("IDCODE", read_buf, 4);

    // Enable SRAM
    memcpy(write_buf, (uint8_t[]){0xC6,0x00,0x00,0x00}, 4);
    xi2c_write_bytes(hi2c, DevAddress, write_buf, 4);
    HAL_Delay(1);

    // Erase SRAM
    memcpy(write_buf, (uint8_t[]){0x0E,0x00,0x00,0x00}, 4);
    xi2c_write_bytes(hi2c, DevAddress, write_buf, 4);
    HAL_Delay(5000);

    // Read Status
    memset(read_buf, 0, 4);
    memcpy(write_buf, (uint8_t[]){0x3C,0x00,0x00,0x00}, 4);
    xi2c_write_and_read(hi2c, DevAddress, write_buf, 4, read_buf, 4);
    if(verbose_on) print_hex_buf("Erase Status", read_buf, 4);

    // Program Command
    memcpy(write_buf, (uint8_t[]){0x46,0x00,0x00,0x00}, 4);
    xi2c_write_bytes(hi2c, DevAddress, write_buf, 4);
    HAL_Delay(1);

    // Send Bitstream
    memcpy(write_buf, (uint8_t[]){0x7A,0x00,0x00,0x00}, 4);
    xi2c_write_long(hi2c, DevAddress, write_buf, 4, (uint8_t*)0x081A0000, (size_t)163489);
    HAL_Delay(1);

    // USERCODE (optional)
    memset(read_buf, 0, 4);
    memcpy(write_buf, (uint8_t[]){0xC0,0x00,0x00,0x00}, 4);
    xi2c_write_and_read(hi2c, DevAddress, write_buf, 4, read_buf, 4);
    if(verbose_on) print_hex_buf("User Register", read_buf, 4);

    // Final Status
    memset(read_buf, 0, 4);
    memcpy(write_buf, (uint8_t[]){0x3C,0x00,0x00,0x00}, 4);
    xi2c_write_and_read(hi2c, DevAddress, write_buf, 4, read_buf, 4);
    if(verbose_on) print_hex_buf("Program Status", read_buf, 4);

    if(read_buf[2] != 0x0F) ret_status = 1;

    // Exit Program Mode
    memcpy(write_buf, (uint8_t[]){0x26,0x00,0x00,0x00}, 4);
    xi2c_write_bytes(hi2c, DevAddress, write_buf, 4);

    if(verbose_on) printf("FPGA configuration complete.\r\n");
    return ret_status;
}

// Callback implementations
void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef *hi2c)
{
    txComplete = 1;
}

void HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
    rxComplete = 1;
}

void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c)
{
    i2cError = 1;
}
