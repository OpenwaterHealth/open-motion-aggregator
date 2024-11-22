/*
 * crosslink.h
 *
 *  Created on: Aug 6, 2024
 *      Author: gvigelet
 */

#ifndef INC_CROSSLINK_H_
#define INC_CROSSLINK_H_
#include "main.h"
#include <stdbool.h>

void fpga_reset(CameraDevice *cam);
void fpga_soft_reset(CameraDevice *cam);
int fpga_send_activation(CameraDevice *cam);
void fpga_on(CameraDevice *cam);
void fpga_off(CameraDevice *cam);
int fpga_checkid(CameraDevice *cam);
int fpga_enter_sram_prog_mode(CameraDevice *cam);
int fpga_erase_sram(CameraDevice *cam);
int fpga_read_usercode(CameraDevice *cam);
int fpga_read_status(CameraDevice *cam);
int fpga_exit_prog_mode(CameraDevice *cam);

int fpga_program_sram(CameraDevice *cam, bool rom_bitstream, uint8_t* pData, uint32_t Data_Len);

int program_bitstream(CameraDevice *cam);

#endif /* INC_CROSSLINK_H_ */
