#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>

#include "../src/xv11.h"
#include "../src/xv11_tty.h"

void _process_reading(struct xv11 *self, struct xv11_reading *r){
	printf("reading: %d %d\n", r->angle, r->distance); 
}

static struct xv11_callbacks _cbs = {
	.on_process_reading = _process_reading
}; 

int main(int argc, char **argv){
	struct xv11 xv11; 

	xv11_init(&xv11, &_cbs); 
	
	// TODO 

	xv11_deinit(&xv11); 

	return 0; 
}

