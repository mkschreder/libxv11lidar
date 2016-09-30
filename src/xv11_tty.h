/*
	libxv11

	Copyright (C) 2016 Martin K. Schröder <mkschreder.uk@gmail.com>

	This program is free software: you can redistribute it and/or modify
	it under the terms of the GNU General Public License as published by
	the Free Software Foundation, either version 3 of the License, or
	(at your option) any later version. (Please read LICENSE file on special
	permission to include this software in signed images). 

	This program is distributed in the hope that it will be useful,
	but WITHOUT ANY WARRANTY; without even the implied warranty of
	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
	GNU General Public License for more details.
*/

#pragma once

#include <termios.h> 

struct xv11_tty {
	int fd; 
	struct termios old_io;
}; 

int xv11_tty_init(struct xv11_tty *self, const char *path); 
int xv11_tty_read(struct xv11_tty *self); 
int xv11_tty_flush(struct xv11_tty *self); 
int xv11_tty_process_data(struct xv11_tty *self, struct xv11 *lidar); 
void xv11_tty_deinit(struct xv11_tty *self); 

