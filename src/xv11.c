/*
 * XV11 LIDAR communication library
 *
 * Copyright (C) 2016 Bartosz Meglicki <meglickib@gmail.com>
 * Copyright (C) 2016 Martin Schröder <mkschreder.uk@gmail.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 3 as
 * published by the Free Software Foundation.
 * This program is distributed "as is" WITHOUT ANY WARRANTY of any
 * kind, whether express or implied; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include "xv11.h"

#include <stdlib.h> //exit
#include <stdio.h> //fprintf
#include <errno.h> //errno
#include <string.h> //memcpy
#include <limits.h> //UCHAR_MAX
#include <stdbool.h>

typedef enum {
	XV11_WAIT_FRAME, 
	XV11_READ_FRAME
} xv11_state_t; 

#define FRAME_START 0xFA

/*	For complete information on LIDAR data format see: 
 *	http://xv11hacking.wikispaces.com/LIDAR+Sensor
 *  LIDAR returns data in frames of 4 consecutive readings (angles)
*/
//single angle reading
struct xv11_raw_reading {
	unsigned int distance : 14; //distance or error code when invalid_data flag is set
	unsigned int strength_warning : 1; //flag indicating that reported signal strength is lower then expected
	unsigned int invalid_data : 1; //flag indicating that distance could not be calculated	
	uint16_t signal_strength; //received signal strength 
} __attribute__((packed));

struct xv11_raw_frame {
	uint8_t start; //fixed 0xFA can be used for synchronization
	uint8_t index; //(index-0xA0)*4 is the angle for readings[0] (+1,2,3 for consecutive readings)
	uint16_t speed; //divide by 64 to get speed in rpm
	struct xv11_raw_reading readings[4]; //readings for 4 consecutive angles
	uint16_t checksum; //if you need it in xv11lidar.c there is a function for calculating the checksum, compare with this value
} __attribute__((packed));

struct xv11_pdata {
	xv11_state_t state; 
	size_t n_received; 
	struct xv11_raw_frame frame; 
}; 

/*
 *  This function calculates the checksum from the first 20 bytes of frame returned by the LIDAR
 *  You can expose this function (in xv11lidar.h) and compare its value with laser_frame.checksum
 *  Currently the ReadLaser implementation only outputs CRCFAIL
 *  to stderr when the checksum doesn't match the expected value
 *  A lot of checksum failures can indicate problem with reading the data (imperfect soldering, etc)
 *  or that the synchronization was lost and we are not reading frames correctly.
 *  Synchronization could be lost for example when not reading the data long and buffer overflow happens
 */
static uint16_t _xv11_checksum(const struct xv11_raw_frame *frame){
	uint8_t *data = (uint8_t*)frame; 
	uint32_t chk32=0;
	uint16_t word;
	int i;
	
	for(i=0;i<10;++i)
	{
		word=data[2*i] + (data[2*i+1] << 8);
		chk32 = (chk32 << 1) + word;
	}
	
	uint32_t checksum=(chk32 & 0x7FFF) + (chk32 >> 15);
	return checksum & 0x7FFF;
}

/*
 * Flushes the TTY buffer so that we synchronize on new data
 * Waits for 0xFA byte (starting laser_frame) followed by 0xA0 which is angle 0-3 frame
 * Discards the rest 20 bytes of frame and the next 89 frames so that the next read will 
 * start reading at frame with 0-3 angles. 
 * Finally sets the termios to return the data in largest possible chunks (termios.c_cc[VMIN])
 */
#if 0
int SynchronizeLaser(int fd, int laser_frames_per_read){		
	uint8_t c=0;
	int i;
		
	while(1)
	{
		if (read(fd,&c,1)>0)
		{
			//wait for frame start
			if(c==0xFA)
			{
				//wait for angle 0
				if (read(fd,&c,1)>0 && c!=0xA0)
					continue;
				
				//discard 360 degree scan (next 20 bytes of frame 0 and 89 frames with 22 bytes)
				for(i=0;i<20 + 22*89;++i) 
					if (read(fd,&c,1)<0)
						return TTY_ERROR;						
					
				//get the termios and set it to return data in largest possible chunks
				struct termios io;
				if(tcgetattr(fd, &io) < 0)				
					return TTY_ERROR;
							
				if(laser_frames_per_read*sizeof(struct laser_frame) <= UCHAR_MAX)					
					io.c_cc[VMIN]=laser_frames_per_read*sizeof(struct laser_frame); 
				else
					io.c_cc[VMIN]=11*sizeof(struct laser_frame); //11*22=242 which is the largest possible value <= UCHAR_MAX 	
					
				if(tcsetattr(fd, TCSANOW, &io) < 0)
					return TTY_ERROR;
		
				break;
			}	
		}
		else
			return TTY_ERROR;
	}
	return SUCCESS;
}
#endif

static void _xv11_enter_state(struct xv11 *self, xv11_state_t state){
	// leave state 
	switch(self->pdata->state){
		default: break; 
	}
	// enter state
	switch(state){
		default: break; 
	}
	self->pdata->state = state; 
}

/*
 * Open the terminal
 * Save its original settings in lidar_data->old_io
 * Set terminal for raw byte input single byte at a time at 115200 speed
 * Synchronize with the laser
 * Alloc internal buffer for laser readings
 */
int xv11_init(struct xv11 *self, struct xv11_callbacks *cb){
	memset(self, 0, sizeof(struct xv11)); 

	self->pdata = calloc(1, sizeof(struct xv11_pdata)); 	
	if(!self->pdata) return -ENOMEM; 

	self->cb = cb; 

	self->pdata->n_received = 0; 

	_xv11_enter_state(self, XV11_WAIT_FRAME);  

	return 0; 
}

void xv11_deinit(struct xv11 *self){
	free(self->pdata);
}

#include <stdio.h>

int xv11_putc(struct xv11 *self, int ch){
	if(ch < 0) return -1; 
	uint8_t *data = ((uint8_t*)&self->pdata->frame) + self->pdata->n_received; 
	switch(self->pdata->state){	
		case XV11_WAIT_FRAME: {
			if(ch == FRAME_START) {
				_xv11_enter_state(self, XV11_READ_FRAME); 
			} else {
				// discard
				self->pdata->n_received = 0; 
				break; 
			}
			// fall through
		}  
		case XV11_READ_FRAME: {
			*data = ch; 
			self->pdata->n_received++; 
			// check if we have received one frame and validate it
			if(self->pdata->n_received >= sizeof(struct xv11_raw_frame)){
				uint16_t chk = _xv11_checksum(&self->pdata->frame); 
				if(chk != self->pdata->frame.checksum){
					//printf("crc error: %04x != %04x\n", chk, self->pdata->frame.checksum); 
				} else {
					if(self->cb && self->cb->on_process_reading) {
						for(int c = 0; c < 4; c++){	
							struct xv11_reading r; 
							r.angle = (self->pdata->frame.index - 0xA0) * 4 + c;  
							r.distance = self->pdata->frame.readings[c].distance;  
							r.rpm = self->pdata->frame.speed / 64; 
							if(self->pdata->frame.readings[c].invalid_data) r.distance = 0; 
							self->cb->on_process_reading(self, &r);
						}
					}
				}
				self->pdata->n_received = 0; 
				_xv11_enter_state(self, XV11_WAIT_FRAME); 
			}
		} break; 
	}
	return 0; 
}

/*
 * Read from LIDAR until requested number of frames is read or error occurs
 * 
 */
#if 0
int xv11_read(struct xv11lidar_data *lidar_data, struct laser_frame *frame_data){
	const size_t total_read_size=sizeof(struct laser_frame)*lidar_data->laser_frames_per_read;
	uint8_t *data=lidar_data->data; 
	size_t bytes_read=0;
	int status=0;
	int i;
	
	while( bytes_read < total_read_size )
	{
		if( (status=read(lidar_data->fd,data+bytes_read,total_read_size-bytes_read))<0 )
			return -1;
		else if(status==0)
			return -1;
			
		bytes_read+=status;
	}

	memcpy(frame_data, data, total_read_size);
	
	for(i=0;i<lidar_data->laser_frames_per_read;++i)
	{
		if(Checksum(data+i*sizeof(struct laser_frame))!=frame_data[i].checksum)
			fprintf(stderr, " CRCFAIL ");			
		
		if(frame_data[i].start!=0xFA)
			return -1;
	}
	return 0; 
}
#endif
