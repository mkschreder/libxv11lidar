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

#ifdef HAVE_TERMIOS_H

#include <termios.h> //struct termios, tcgetattr, tcsetattr, cfsetispeed, tcflush
#include <fcntl.h> //file open flags
#include <unistd.h> //read, close
#include <errno.h>

#include "xv11.h"
#include "xv11_tty.h"

int xv11_tty_init(struct xv11_tty *self, const char *path){
	struct termios io;

	if ((self->fd=open(path, O_RDONLY))==-1)
		return errno;
	
	if(tcgetattr(self->fd, &self->old_io) < 0)
	{
		close(self->fd);
		return errno;		
	}
			
	io.c_iflag=io.c_oflag=io.c_lflag=0;
	io.c_cflag=CS8|CREAD|CLOCAL; //8 bit characters
	
	io.c_cc[VMIN]=1; //one input byte enough
	io.c_cc[VTIME]=0; //no timer
	
	if(cfsetispeed(&io, B115200) < 0 || cfsetospeed(&io, B115200) < 0)
	{
		close(self->fd);
		return errno;		
	}

	if(tcsetattr(self->fd, TCSAFLUSH, &io) < 0)
	{
		close(self->fd);
		return errno;
	}

	return 0; 
}

int xv11_tty_read(struct xv11_tty *self){
	uint8_t c; 
	int ret = read(self->fd, &c, 1); 
	if(ret < 0) return ret; 
	return (int)c; 
}

void xv11_tty_close(struct xv11_tty *self){
	tcsetattr(self->fd, TCSAFLUSH, &self->old_io); 
	
	close(self->fd);

	self->fd = -1; 
}

int xv11_tty_flush(struct xv11_tty *self){
	return tcflush(self->fd, TCIOFLUSH); 
}

#endif
