/*
To compile:
gcc -o build/runBMC2K runBMC2K.c -I/opt/Boston\ Micromachines/include -I$HOME/cacao/lib -L/opt/Boston\ Micromachines/lib -Wl,-rpath-link,/opt/Boston\ Micromachines/lib -lBMC -lBMC_PCIeAPI -lncurses -lImageStreamIO
*/

/* BMC */
#include <BMCApi.h>

/* cacao */
#include "ImageStruct.h"   // cacao data structure definition
#include "ImageStreamIO.h" // function ImageStreamIO_read_sharedmem_image_toIMAGE()

#include <stdlib.h>
#include <ctype.h>
#include <string.h>
#include <time.h>
#include <curses.h>
#include <unistd.h>

typedef int bool_t;

BMCRC sendCommand()
{
	// Initialize variables
	DM hdm = {};
	BMCRC rv;
	int k=0;
	uint32_t *map_lut;
	double *command;

	// Open driver
	char serial_number[12] = "27BW027#081";
	rv = BMCOpen(&hdm, serial_number);
	// Check for errors
	if(rv) {
		printf("Error %d opening the driver type %u.\n", rv, (unsigned int)hdm.Driver_Type);
		printf("%s\n\n", BMCErrorString(rv));
		return rv;
	}

	printf("Opened Device %d with %d actuators.\n", hdm.DevId, hdm.ActCount);

	// Load actuator map
    map_lut = (uint32_t *)malloc(sizeof(uint32_t)*MAX_DM_SIZE);
    for(k=0; k<(int)hdm.ActCount; k++) {
        map_lut[k] = 0;
    }
    rv = BMCLoadMap(&hdm, NULL, map_lut);

	// Make up an arbitrary command for now
    command = (double *)calloc(hdm.ActCount, sizeof(double));
	for(k=0; k<hdm.ActCount; k++) {
		command[k] = 0.5;
	}

	// Send command
	rv = BMCSetArray(&hdm, command, map_lut);
	// Check for errors
	if(rv) {
		printf("Error %d sending voltages.\n", rv);
		return rv;
	}

	// Zero the DM and close.
	rv = BMCClearArray(&hdm);
	if (rv) {
		printf("Error %d clearing voltages.\n", rv);
		return rv;
	}
	rv = BMCClose(&hdm);
	if (rv) {
		printf("Error %d closing the driver.\n", rv);
		return rv;
	}

	// Clean up
	free(command);

	return 0;
}


int main(int argc, char* argv[]) {

	// initialize variables
	BMCRC rv;

	// send command
	rv = sendCommand();
	if (rv) {
		printf("Error %d sending voltages.\n", rv);
		return rv;
	}

	return 0;
}