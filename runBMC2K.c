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
#include <signal.h>
#include <curses.h>
#include <unistd.h>

typedef int bool_t;

// interrupt signal handling for safe DM shutdown
volatile sig_atomic_t stop;

void handle_signal(int signal)
{
    if (signal == SIGINT)
    {
        printf("\nExiting the BMC 2K control loop.\n");
        stop = 1;
    }
}

// Initialize the shared memory image
void initializeSharedMemory(char * serial, uint32_t nbAct)
{
    long naxis; // number of axis
    uint8_t atype;     // data type
    uint32_t *imsize;  // image size 
    int shared;        // 1 if image in shared memory
    int NBkw;          // number of keywords supported
    IMAGE* SMimage;

    SMimage = (IMAGE*) malloc(sizeof(IMAGE));

    naxis = 2;
    imsize = (uint32_t *) malloc(sizeof(uint32_t)*naxis);
    imsize[0] = nbAct;
    imsize[1] = 1;
    
    // image will be float type
    // see file ImageStruct.h for list of supported types
    atype = _DATATYPE_DOUBLE;
    // image will be in shared memory
    shared = 1;
    // allocate space for 10 keywords
    NBkw = 10;
    // create an image in shared memory
    ImageStreamIO_createIm(&SMimage[0], serial, naxis, imsize, atype, shared, NBkw);

    /* flush all semaphores to avoid commanding the DM from a 
    backlog in shared memory */
    ImageStreamIO_semflush(&SMimage[0], -1);
    
    // write 0s to the image
    SMimage[0].md[0].write = 1; // set this flag to 1 when writing data
    int i;
    for (i = 0; i < nbAct; i++)
    {
      SMimage[0].array.D[i] = 0.;
    }

    // post all semaphores
    ImageStreamIO_sempost(&SMimage[0], -1);
        
    SMimage[0].md[0].write = 0; // Done writing data
    SMimage[0].md[0].cnt0++;
    SMimage[0].md[0].cnt1++;
}

BMCRC sendCommand(DM hdm, uint32_t *map_lut, IMAGE * SMimage) {
    // Initialize variables
    double *command;
    int k=0;
    BMCRC rv;

    // Cast to array type ALPAO expects
    command = (double*)calloc(hdm.ActCount, sizeof(double));
    for (k = 0; k < hdm.ActCount; k++) {
        command[k] = SMimage[0].array.D[k];
    }

    // Send command
    rv = BMCSetArray(&hdm, command, map_lut);
    // Check for errors
    if(rv) {
        printf("Error %d sending voltages.\n", rv);
        return rv;
    }

    // Clean up
    free(command);

    return 0;
}

// intialize DM and shared memory and enter DM command loop
int controlLoop() {

    // Initialize variables
    DM hdm = {};
    BMCRC rv;
    int k=0;
    uint32_t *map_lut;
    IMAGE * SMimage;

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

    // initialize shared memory image to 0s
    initializeSharedMemory(serial_number, hdm.ActCount);

    // connect to shared memory image (SMimage)
    SMimage = (IMAGE*) malloc(sizeof(IMAGE));
    ImageStreamIO_read_sharedmem_image_toIMAGE(serial_number, &SMimage[0]);

    // Validate SMimage dimensionality and size against DM
    if (SMimage[0].md[0].naxis != 2) {
        printf("SM image naxis = %d\n", SMimage[0].md[0].naxis);
        return -1;
    }
    if (SMimage[0].md[0].size[0] != hdm.ActCount) {
        printf("SM image size (axis 1) = %d", SMimage[0].md[0].size[0]);
        return -1;
    }

    // set DM to all-0 state to begin
    printf("BMC %s: initializing all actuators to 0.\n", serial_number);
    ImageStreamIO_semwait(&SMimage[0], 0);
    rv  = sendCommand(hdm, map_lut, SMimage);
    if (rv) {
        printf("Error %d sending command.\n", rv);
        return rv;
    }

    // SIGINT handling
    struct sigaction action;
    action.sa_flags = SA_SIGINFO;
    action.sa_handler = handle_signal;
    sigaction(SIGINT, &action, NULL);
    stop = 0;

    // control loop
    while (!stop) {
        printf("BMC %s: waiting on commands.\n", serial_number);
        // Wait on semaphore update
        ImageStreamIO_semwait(&SMimage[0], 0);
        
        // Send Command to DM
        if (!stop) { // Skip DM on interrupt signal
            rv = sendCommand(hdm, map_lut, SMimage);
            if (rv) {
                printf("Error %d sending command.\n", rv);
                return rv;
            }
        }
    }

    // Safe DM shutdown on loop interrupt
    // Zero all actuators
    rv = BMCClearArray(&hdm);
    if (rv) {
        printf("Error %d clearing voltages.\n", rv);
        return rv;
    }
    // Close the connection
    rv = BMCClose(&hdm);
    if (rv) {
        printf("Error %d closing the driver.\n", rv);
        return rv;
    }
    return 0;
}


int main(int argc, char* argv[]) {

    // add bias, etc. options here

    BMCRC rv = controlLoop();
    if (rv) {
        printf("Encountered error %d.\n", rv);
        return rv;
    }

    return 0;
}