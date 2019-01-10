/*
If, for whatever reason, runBMC2K crashes without releasing the mirror properly, 
run this script to open the driver connection, zero the DM, and release the connection.

To compile:
gcc -o build/releaseBMC2K releaseBMC2K.c -I/opt/Boston\ Micromachines/include -L/opt/Boston\ Micromachines/lib -Wl,-rpath-link,/opt/Boston\ Micromachines/lib -lBMC -lBMC_PCIeAPI
*/


/* System Headers */
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <stddef.h>

/* BMC */
#include  <BMCApi.h>


BMCRC releaseMirror(char *serial_number) {

    BMCRC rv;
    int idx;
    DM hdm;
    uint32_t *map_lut;

    // Open driver
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
    for(idx=0; idx<(int)hdm.ActCount; idx++) {
        map_lut[idx] = 0;
    }
    rv = BMCLoadMap(&hdm, NULL, map_lut);

    // Zero all actuators
    rv = BMCClearArray(&hdm);
    if (rv) {
        printf("Error %d clearing voltages.\n", rv);
        return rv;
    }
    printf("BMC %s: all voltages set to 0.\n", serial_number);

    // Close the connection
    rv = BMCClose(&hdm);
    if (rv) {
        printf("Error %d closing the driver.\n", rv);
        return rv;
    }
    printf("BMC %s: connection closed.\n", serial_number);

    return 0;
}

/* Main program */
int main( int argc, char ** argv )
{
    char *serial_number;
    BMCRC rv;

    if (argc < 2)
    {
        printf("Serial number must be supplied.\n");
        return -1;
    }
    serial_number = argv[1];

    rv = releaseMirror(serial_number);
    if (rv)
        printf("Error %d releasing the connection.\n", rv);

    return rv;
}