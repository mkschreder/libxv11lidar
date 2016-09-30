/*
 * XV11 LIDAR communication library
 *
 * Copyright (C) 2016 Bartosz Meglicki <meglickib@gmail.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 3 as
 * published by the Free Software Foundation.
 * This program is distributed "as is" WITHOUT ANY WARRANTY of any
 * kind, whether express or implied; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#pragma once

#include <stdint.h>

//return value for all the functions
enum xv11_status {SUCCESS=0, SYNCHRONIZATION_ERROR, TTY_ERROR, MEMORY_ERROR};

struct xv11_pdata; 
struct xv11; 

struct xv11_reading {
	uint16_t angle; 
	uint16_t distance; 
	uint8_t rpm;  
}; 

struct xv11_callbacks {
	void (*on_process_reading)(struct xv11 *self, struct xv11_reading *r); 
}; 

struct xv11 {
	struct xv11_pdata *pdata; 
	struct xv11_callbacks *cb; 
};

int xv11_init(struct xv11 *self, struct xv11_callbacks *cb);
void xv11_deinit(struct xv11 *self);

// process one protocol character 
int xv11_putc(struct xv11 *self, int ch); 

#include "xv11_tty.h"

