/*
 * crosslink.c
 *
 *  Created on: Aug 6, 2024
 *      Author: gvigelet
 */

#include "crosslink.h"
#include "main.h"
#include "common.h"
#include "utils.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdbool.h>
#include <stdint.h>

#define I2C_ADDRESS 0x40  // Replace with your FPGA's I2C address
#define CHUNK_SIZE 1024  // Define a manageable chunk size (can be adjusted)
#define BITSTREAM_SIZE 0x27EA1  // Replace with the size of your bitstream
#define BITSTREAM_ADDR 0x70C80000

#define CMD_INITIATE 0xC0
#define CMD_ISSUE 0x74
#define CMD_ENABLE_SRAM 0xC6
#define CMD_ERASE_SRAM 0x26
#define CMD_READ_STATUS 0x3C
#define CMD_PROGRAM_SRAM 0x46
#define CMD_VERIFY_USERCODE 0xC8
#define CMD_DISABLE 0x26

#define I2C_SCL_PIN GPIO_PIN_8
#define I2C_SDA_PIN GPIO_PIN_9
#define I2C_GPIO_PORT GPIOB

volatile uint8_t txComplete = 0;
volatile uint8_t rxComplete = 0;
volatile uint8_t i2cError = 0;


unsigned char write_buf[4];
unsigned char read_buf[4];
unsigned char activation_key[5] = {0xFF, 0xA4, 0xC6, 0xF4, 0x8A};

bool crosslink_verbose = true;

HAL_StatusTypeDef i2c_write_and_read(CameraDevice *cam, uint8_t *pData, uint16_t WriteSize, uint8_t *pReadData, uint16_t ReadSize)
{
    txComplete = 0;
    rxComplete = 0;
    i2cError = 0;

    // Check if the I2C peripheral is ready
    while (HAL_I2C_GetState(cam->pI2c) != HAL_I2C_STATE_READY) {
        if(crosslink_verbose) printf("I2C busy, waiting...\r\n");
        HAL_Delay(1);  // Add a small delay to avoid busy looping
    }

    // Write operation with NoStop (to generate a repeated start)
    HAL_StatusTypeDef ret = HAL_I2C_Master_Seq_Transmit_IT(cam->pI2c, I2C_ADDRESS << 1, pData, WriteSize, I2C_FIRST_FRAME);
    if (ret != HAL_OK)
    {
    	uint32_t hal_error = HAL_I2C_GetError(cam->pI2c);
    	if(crosslink_verbose) printf("++++> i2c_write_and_read HAL TX ERROR 0x%04lX\r\n", hal_error);
        return ret; // Return if the write operation fails
    }

    // Wait for the transmission to complete
    while (!txComplete && !i2cError) {}

    if (i2cError)
    {
        return HAL_ERROR;
    }

    // Check if the I2C peripheral is ready
    while (HAL_I2C_GetState(cam->pI2c) != HAL_I2C_STATE_READY) {
        if(crosslink_verbose) printf("I2C busy, waiting...\r\n");
        HAL_Delay(1);  // Add a small delay to avoid busy looping
    }

    // Read operation with a repeated start condition
    ret = HAL_I2C_Master_Seq_Receive_IT(cam->pI2c, I2C_ADDRESS << 1, pReadData, ReadSize, I2C_LAST_FRAME);
    if (ret != HAL_OK)
    {
    	uint32_t hal_error = HAL_I2C_GetError(cam->pI2c);
    	if(crosslink_verbose) printf("++++> i2c_write_and_read HAL REC ERROR 0x%04lX\r\n", hal_error);
        return ret; // Return if the read operation fails
    }

    // Wait for the reception to complete
    while (!rxComplete && !i2cError) {}

    if (i2cError)
    {
        return HAL_ERROR;
    }

    return HAL_OK; // Return HAL_OK if both operations succeed
}

extern uint8_t bitstream_buffer[];
HAL_StatusTypeDef i2c_write_long(CameraDevice *cam, uint8_t *write_command, uint16_t writecomm_len, uint8_t *write_data, uint32_t writedata_len) {
	int chunk_size = 1024;  // Define the chunk size
    int num_bytes_command = writecomm_len;  // Length in bytes for command data
    int num_bytes_data = writedata_len;  // Length in bytes for data
    int total_len_bytes = num_bytes_command + num_bytes_data;  // Total length in bytes
    int num_chunks = (total_len_bytes + chunk_size - 1) / chunk_size;  // Calculate number of chunks
    uint8_t *pData;
	uint16_t datalen;

    memset(bitstream_buffer, 0, MAX_BITSTREAM_SIZE);

    // Combine write_command and write_data into one buffer
    memcpy(bitstream_buffer, write_command, num_bytes_command);  // Copy command data
    memcpy(bitstream_buffer + num_bytes_command, write_data, num_bytes_data);  // Copy data

    if(crosslink_verbose) printf("Bitstream CRC: 0x%04X Length: %ld\r\n", util_crc16(write_data, writedata_len), writedata_len);
    if(crosslink_verbose) printf("FULL CRC: 0x%04X Length: %d\r\n", util_crc16(bitstream_buffer, total_len_bytes), total_len_bytes);

    HAL_StatusTypeDef ret;
    uint16_t offset = 0;
    uint32_t frame_flag;

    // Split into chunks and send them sequentially
    for (int i = 0; i < num_chunks; i++) {
        // Calculate the size of the current chunk
        int current_chunk_size = (i == num_chunks - 1) ? (total_len_bytes % chunk_size) : chunk_size;
        current_chunk_size = (total_len_bytes - offset) > chunk_size ? chunk_size : (total_len_bytes - offset);
        if (current_chunk_size == 0) current_chunk_size = chunk_size;  // Handle case where the last chunk is exactly chunk_size bytes


        // Decide frame flag for the chunk
        if (i == 0) {
            // First chunk
            frame_flag = I2C_FIRST_AND_NEXT_FRAME;
        } else if (i == num_chunks-1) {
            // Last chunk
            frame_flag = I2C_LAST_FRAME;
        } else {
            // Intermediate chunks
            frame_flag = I2C_NEXT_FRAME;
        }

        // if(crosslink_verbose) printf("Count: %d Chunk Size: %d\r\n", i, current_chunk_size);

        // Check if the I2C peripheral is ready
        while (HAL_I2C_GetState(cam->pI2c) != HAL_I2C_STATE_READY) {
            //if(crosslink_verbose) printf("I2C busy, waiting...\r\n");
            HAL_Delay(1);  // Add a small delay to avoid busy looping
        }
        pData = (uint8_t*)&bitstream_buffer[i * chunk_size];
        datalen = (uint16_t)current_chunk_size;

        // Transmit the current chunk
        ret = HAL_I2C_Master_Seq_Transmit_IT(cam->pI2c, I2C_ADDRESS << 1, pData, datalen, frame_flag);
        if (ret != HAL_OK) {
        	if(crosslink_verbose) printf("++++> i2c_write_long HAL TX HAL_StatusTypeDef: 0%04X I2C_ERROR: 0%04lX\r\n", ret, cam->pI2c->ErrorCode);
            return ret; // Return if any transmission fails
        }

        // Wait for the transmission to complete
        while (!txComplete && !i2cError) {}

        if (i2cError)
        {
            return HAL_ERROR;
        }

    }

    if(crosslink_verbose) printf("Programmed Successfully\r\n");
    HAL_Delay(100);
    return HAL_OK;
}

HAL_StatusTypeDef i2c_write_bitstream(CameraDevice *cam, uint8_t *write_bitstream, uint32_t bitstream_len) {
	int chunk_size = 1024;  // Define the chunk size
    int total_len_bytes = bitstream_len;  // Total length in bytes
    int num_chunks = (total_len_bytes + chunk_size - 1) / chunk_size;  // Calculate number of chunks
    uint8_t *pData;
	uint16_t datalen;

    HAL_StatusTypeDef ret;
    uint16_t offset = 0;
    uint32_t frame_flag;
    if(bitstream_len ==0) Error_Handler();
    if(crosslink_verbose) printf("Bitstream CRC: 0x%04X Length: %ld\r\n", util_crc16((uint8_t*)(write_bitstream+4), bitstream_len-4), bitstream_len-4);
    if(crosslink_verbose) printf("FULL CRC: 0x%04X Length: %ld\r\n", util_crc16(write_bitstream, bitstream_len), bitstream_len);

    // Split into chunks and send them sequentially
    for (int i = 0; i < num_chunks; i++) {
        // Calculate the size of the current chunk
        int current_chunk_size = (i == num_chunks - 1) ? (total_len_bytes % chunk_size) : chunk_size;
        current_chunk_size = (total_len_bytes - offset) > chunk_size ? chunk_size : (total_len_bytes - offset);
        if (current_chunk_size == 0) current_chunk_size = chunk_size;  // Handle case where the last chunk is exactly chunk_size bytes


        // Decide frame flag for the chunk
        if (i == 0) {
            // First chunk
            frame_flag = I2C_FIRST_AND_NEXT_FRAME;
        } else if (i == num_chunks-1) {
            // Last chunk
            frame_flag = I2C_LAST_FRAME;
        } else {
            // Intermediate chunks
            frame_flag = I2C_NEXT_FRAME;
        }

        // if(crosslink_verbose) printf("Count: %d Chunk Size: %d\r\n", i, current_chunk_size);

        // Check if the I2C peripheral is ready
        while (HAL_I2C_GetState(cam->pI2c) != HAL_I2C_STATE_READY) {
            //if(crosslink_verbose) printf("I2C busy, waiting...\r\n");
            HAL_Delay(1);  // Add a small delay to avoid busy looping
        }
        pData = (uint8_t*)&write_bitstream[i * chunk_size];
        datalen = (uint16_t)current_chunk_size;

        // Transmit the current chunk
        ret = HAL_I2C_Master_Seq_Transmit_IT(cam->pI2c, I2C_ADDRESS << 1, pData, datalen, frame_flag);
        if (ret != HAL_OK) {
        	if(crosslink_verbose) printf("++++> i2c_write_long HAL TX HAL_StatusTypeDef: 0%04X I2C_ERROR: 0%04lX\r\n", ret, cam->pI2c->ErrorCode);
            return ret; // Return if any transmission fails
        }

        // Wait for the transmission to complete
        while (!txComplete && !i2cError) {}

        if (i2cError)
        {
            return HAL_ERROR;
        }

    }

    if(crosslink_verbose) printf("Programmed Successfully\r\n");
    HAL_Delay(100);

    return HAL_OK;
}


HAL_StatusTypeDef i2c_write_byte( CameraDevice *cam, uint16_t size, uint8_t *pData) {
    return HAL_I2C_Master_Transmit(cam->pI2c, I2C_ADDRESS << 1, pData, size, HAL_MAX_DELAY);
}

void fpga_reset(CameraDevice *cam)
{

    if(crosslink_verbose) printf("Resetting FPGA....");
    HAL_GPIO_WritePin(cam->cresetb_port, cam->cresetb_pin, GPIO_PIN_SET);
    HAL_Delay(250);

    HAL_GPIO_WritePin(cam->cresetb_port, cam->cresetb_pin, GPIO_PIN_RESET);
    HAL_Delay(1000);
    if(crosslink_verbose) printf("COMPLETED\r\n");
}

void fpga_soft_reset(CameraDevice *cam)
{
	  if(crosslink_verbose) printf("Soft resetting FPGA....");

	  HAL_GPIO_WritePin(cam->gpio0_port, cam->gpio0_pin, GPIO_PIN_RESET);
	  HAL_Delay(200);
	  HAL_GPIO_WritePin(cam->gpio0_port, cam->gpio0_pin, GPIO_PIN_SET);
   	  if(crosslink_verbose) printf("COMPLETED\r\n");
}

int fpga_send_activation(CameraDevice *cam)
{
	// Step 1: Initialize
	if(crosslink_verbose) printf("Step 1: Send Activation Key\r\n");
	if (i2c_write_byte(cam, 5, activation_key) < 0) {
		if(crosslink_verbose) printf("failed to send activation key\r\n");
	    return 1;  // Exit if writing activation key fails
	}

	return 0;
}

void fpga_on(CameraDevice *cam)
{
    if(crosslink_verbose) printf("Turning on FPGA\r\n");
    HAL_GPIO_WritePin(cam->cresetb_port, cam->cresetb_pin, GPIO_PIN_SET);
    HAL_Delay(10);
}

void fpga_off(CameraDevice *cam)
{
    if(crosslink_verbose) printf("Turning off FPGA\r\n");
    HAL_GPIO_WritePin(cam->cresetb_port, cam->cresetb_pin, GPIO_PIN_RESET);
    HAL_Delay(10);
}

int fpga_checkid(CameraDevice *cam)
{
    // Step 2: Check IDCODE (Optional)
    if(crosslink_verbose) printf("Step 2: Check IDCODE (Optional)\r\n");
    write_buf[0] = 0xE0; write_buf[1] = 0x00; write_buf[2] = 0x00; write_buf[3] = 0x00;
    if (i2c_write_and_read(cam, write_buf, 4, read_buf, 4) < 0) {
        if(crosslink_verbose) printf("failed to send IDCODE Command\r\n");
        return 1;  // Exit if write/read fails
    }

    if(crosslink_verbose) printf("Device ID read: ");
    for (int i = 0; i < 4; i++) {
        if(crosslink_verbose) printf("%02X ", read_buf[i]);
    }
    if(crosslink_verbose) printf("\r\n");
	return 0;
}

int fpga_bitstream_burst(CameraDevice *cam)
{
    // Bitstream Burst
    if(crosslink_verbose) printf("Bitstream Burst\r\n");
    write_buf[0] = 0x7A; write_buf[1] = 0x00; write_buf[2] = 0x00; write_buf[3] = 0x00;
    if (i2c_write_and_read(cam, write_buf, 4, read_buf, 4) < 0) {
        if(crosslink_verbose) printf("failed to send IDCODE Command\r\n");
        return 1;  // Exit if write/read fails
    }

	return 0;
}

int fpga_enter_sram_prog_mode(CameraDevice *cam)
{
    // Step 3: Enable SRAM Programming Mode
    if(crosslink_verbose) printf("Step 3: Enable SRAM Programming Mode\r\n");
    write_buf[0] = 0xC6; write_buf[1] = 0x00; write_buf[2] = 0x00; write_buf[3] = 0x00;
    if (i2c_write_byte(cam, 4, write_buf) < 0) {
        if(crosslink_verbose) printf("failed to send SRAM Command\r\n");
        return 1;  // Exit if writing fails
    }
    HAL_Delay(1);
    return 0;
}

int fpga_erase_sram(CameraDevice *cam)
{
    // Step 4: Erase SRAM
    if(crosslink_verbose) printf("Step 4: Erase SRAM...");
    write_buf[0] = 0x0E; write_buf[1] = 0x00; write_buf[2] = 0x00; write_buf[3] = 0x00;
    if (i2c_write_byte(cam, 4, write_buf) < 0) {
        if(crosslink_verbose) printf("\r\nFAILED to send SRAM Erase Command\r\n");
        return 1;  // Exit if writing fails
    }
    HAL_Delay(5000);  // Wait for erase to complete
    if(crosslink_verbose) printf("COMPLETED\r\n");
    return 0;
}

int fpga_read_status(CameraDevice *cam)
{
    // Step 5: Read Status Register
    if(crosslink_verbose) printf("Read Status Register\r\n");
    write_buf[0] = 0x3C; write_buf[1] = 0x00; write_buf[2] = 0x00; write_buf[3] = 0x00;
    if (i2c_write_and_read(cam, write_buf, 4, read_buf, 4) < 0) {
        if(crosslink_verbose) printf("failed to send READ Status Command\r\n");
        return 1;  // Exit if write/read fails
    }

    // Check status register bits according to the guide (e.g., Bit-12 Busy = 0, Bit-13 Fail = 0)
    if(crosslink_verbose) printf("Status Register: ");
    for (int i = 0; i < 4; i++) {
        if(crosslink_verbose) printf("%02X ", read_buf[i]);
    }
    if(crosslink_verbose) printf("\r\n");
    return 0;

}

int fpga_read_usercode(CameraDevice *cam)
{
    // Step 9: Read USERCODE (Optional)
    if(crosslink_verbose) printf("Step 9: Verify USERCODE (Optional)\r\n");
    write_buf[0] = 0xC0; write_buf[1] = 0x00; write_buf[2] = 0x00; write_buf[3] = 0x00;
    if (i2c_write_and_read(cam, write_buf, 4, read_buf, 4) < 0) {
        if(crosslink_verbose) printf("failed to send USERCODE Command\r\n");
        return 1;  // Exit if write/read fails
    }
    // Compare read_buf with expected USERCODE if necessary

    if(crosslink_verbose) printf("USERCODE: ");
    for (int i = 0; i < 4; i++) {
        if(crosslink_verbose) printf("%02X ", read_buf[i]);
    }
    if(crosslink_verbose) printf("\r\n");
    return 0;
}

int fpga_program_sram(CameraDevice *cam, bool rom_bitstream, uint8_t* pData, uint32_t Data_Len)
{
    if(crosslink_verbose) printf("Program SRAM\r\n");
    write_buf[0] = 0x46; write_buf[1] = 0x00; write_buf[2] = 0x00; write_buf[3] = 0x00;
    if (i2c_write_byte(cam, 4, write_buf) < 0) {
        if(crosslink_verbose) printf("failed to send Exit Command\r\n");
        return 1;  // Exit if writing fails
    }

    if(rom_bitstream)
    {
    	write_buf[0] = 0x7A; write_buf[1] = 0x00; write_buf[2] = 0x00; write_buf[3] = 0x00;
    	i2c_write_long(cam, write_buf, 4, (uint8_t *)0x080C0000, 0x27EA2);
    }
    else
    {
    	i2c_write_bitstream(cam, pData, Data_Len);
    }

    return 0;
}

int fpga_exit_prog_mode(CameraDevice *cam)
{

    // Step 11: Exit Programming Mode
    if(crosslink_verbose) printf("Step 10: Exit Programming Mode\r\n");
    write_buf[0] = 0x26; write_buf[1] = 0x00; write_buf[2] = 0x00; write_buf[3] = 0x00;
    if (i2c_write_byte(cam, 4, write_buf) < 0) {
        if(crosslink_verbose) printf("failed to send Exit Command\r\n");
        return 1;  // Exit if writing fails
    }

    return 0;
}

int program_bitstream(CameraDevice *cam) {

    if(crosslink_verbose) printf("Starting FPGA Configuration\r\n");

    fpga_reset(cam);

    if(fpga_send_activation(cam)==1) return 1;

    fpga_on(cam);

    if(fpga_checkid(cam)==1) return 1;

    if(fpga_enter_sram_prog_mode(cam)==1) return 1;

    if(fpga_erase_sram(cam)==1) return 1;

    if(fpga_read_status(cam)==1) return 1;

    if(fpga_exit_prog_mode(cam)==1) return 1;

    return 0;
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

	if(crosslink_verbose) printf("++++> HAL_I2C_ErrorCallback 0x%04lX\r\n", hi2c->ErrorCode);
    // Check for the specific error code
    if (hi2c->ErrorCode == HAL_I2C_ERROR_BERR)
    {
        if(crosslink_verbose) printf("Bus Error occurred\r\n");
    }
    else if (hi2c->ErrorCode == HAL_I2C_ERROR_ARLO)
    {
        if(crosslink_verbose) printf("Arbitration Lost occurred\r\n");
    }
    else if (hi2c->ErrorCode == HAL_I2C_ERROR_AF)
    {
        if(crosslink_verbose) printf("Acknowledge Failure occurred\r\n");
    }
    else if (hi2c->ErrorCode == HAL_I2C_ERROR_OVR)
    {
        if(crosslink_verbose) printf("Overrun/Underrun Error occurred\r\n");
    }
    else if (hi2c->ErrorCode == HAL_I2C_ERROR_DMA)
    {
        if(crosslink_verbose) printf("DMA Transfer Error occurred\r\n");
    }
    else if (hi2c->ErrorCode == HAL_I2C_ERROR_TIMEOUT)
    {
        if(crosslink_verbose) printf("Timeout Error occurred\r\n");
    }
    else
    {
        if(crosslink_verbose) printf("Unknown I2C error: 0x%lX\r\n", hi2c->ErrorCode);
    }
}
