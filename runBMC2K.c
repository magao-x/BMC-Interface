/*
To compile:
gcc -o build/runBMC2K runBMC2K.c -I/opt/Boston\ Micromachines/include -L/opt/Boston\ Micromachines/lib -Wl,-rpath-link,/opt/Boston\ Micromachines/lib -lBMC -lBMC_PCIeAPI -lncurses -lImageStreamIO -lpthread -lrt -lm
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
#include <math.h>
#include <argp.h>

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
    atype = _DATATYPE_FLOAT;
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
      SMimage[0].array.F[i] = 0.;
    }

    // post all semaphores
    ImageStreamIO_sempost(&SMimage[0], -1);
        
    SMimage[0].md[0].write = 0; // Done writing data
    SMimage[0].md[0].cnt0++;
    SMimage[0].md[0].cnt1++;
}

/* Remove DC bias in inputs to maximize actuator range */
void bias_inputs(float * command, uint32_t ActCount)
{
    int idx;
    float mean;
    float cenval;

    // calculate mean value
    mean = 0;
    for ( idx = 0 ; idx < ActCount ; idx++)
    {
        mean += command[idx];
    }
    mean /= ActCount;
    printf("I found a mean of: %f\n", mean);

    /* Remove mean from each actuator input
    and add voltage bias to center of range.
    */
    cenval = sqrt(0.5);
    for ( idx = 0 ; idx < ActCount ; idx++)
    {
        command[idx] += cenval - mean;
    }
    printf("Adding bias of: %f\n", cenval);
}

/* Convert any DM inputs to [0, 1] to avoid 
exceeding safe DM operation. */
void clip_to_limits(float * command, uint32_t ActCount)
{
    int idx;
    // check each actuator and clip if needed
    for ( idx = 0 ; idx < ActCount ; idx++)
    {
        if (command[idx] > 1.0)
        {
            printf("Actuator %d saturated!\n", idx + 1);
            command[idx] = 1.0;
        } else if (command[idx] < 0.0)
        {
            printf("Actuator %d saturated!\n", idx + 1);
            command[idx] = 0.0;
        }
    }
}

BMCRC sendCommand(DM hdm, uint32_t *map_lut, IMAGE * SMimage, int nobias) {
    // Initialize variables
    float *command;
    double *command_double;
    int idx;
    uint32_t ActCount;
    BMCRC rv;

    // Cast to array type ALPAO expects
    ActCount = (uint32_t)hdm.ActCount;
    command = (float*)calloc(ActCount, sizeof(float));
    for (idx = 0; idx < ActCount; idx++) {
        command[idx] = SMimage[0].array.F[idx];
    }

    // Apply the bias
    if (nobias != 1) {
        bias_inputs(command, ActCount);
    }

    // Clip to limits
    clip_to_limits(command, ActCount);

    for (idx = 0; idx < ActCount; idx++) {
        printf("Act %d: %f\n", idx, command[idx]);
    }

    // convert to double (there's probably a better way than this loop)
    command_double = (double*)calloc(ActCount, sizeof(double));
    for (idx = 0; idx < ActCount; idx++) {
        command_double[idx] = (double)command[idx];
    }

    // Send command (expected as double)
    rv = BMCSetArray(&hdm, command_double, map_lut);
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
int controlLoop(char * serial_number, int nobias) {

    // Initialize variables
    DM hdm = {};
    BMCRC rv;
    int idx;
    uint32_t ActCount;
    uint32_t *map_lut;
    IMAGE * SMimage;

    // Open driver
    //char serial_number[12] = "27BW027#081";
    rv = BMCOpen(&hdm, serial_number);
    // Check for errors
    if(rv) {
        printf("Error %d opening the driver type %u.\n", rv, (unsigned int)hdm.Driver_Type);
        printf("%s\n\n", BMCErrorString(rv));
        return rv;
    }
    ActCount = (uint32_t)hdm.ActCount;

    printf("Opened Device %d with %d actuators.\n", hdm.DevId, ActCount);

    // Load actuator map
    map_lut = (uint32_t *)malloc(sizeof(uint32_t)*MAX_DM_SIZE);
    for(idx=0; idx<ActCount; idx++) {
        map_lut[idx] = 0;
    }
    rv = BMCLoadMap(&hdm, NULL, map_lut);

    // initialize shared memory image to 0s
    initializeSharedMemory(serial_number, ActCount);
    // connect to shared memory image (SMimage)
    SMimage = (IMAGE*) malloc(sizeof(IMAGE));
    ImageStreamIO_read_sharedmem_image_toIMAGE(serial_number, &SMimage[0]);

    // Validate SMimage dimensionality and size against DM
    if (SMimage[0].md[0].naxis != 2) {
        printf("SM image naxis = %d\n", SMimage[0].md[0].naxis);
        return -1;
    }
    if (SMimage[0].md[0].size[0] != ActCount) {
        printf("SM image size (axis 1) = %d", SMimage[0].md[0].size[0]);
        return -1;
    }

    // set DM to all-0 state to begin
    printf("BMC %s: initializing all actuators to 0.\n", serial_number);
    ImageStreamIO_semwait(&SMimage[0], 0);
    rv  = sendCommand(hdm, map_lut, SMimage, nobias);
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
            rv = sendCommand(hdm, map_lut, SMimage, nobias);
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


/*
Argument parsing
*/

/* Program documentation. */
static char doc[] =
  "runBMC2K-- enter the BMC2K DM command loop and wait for milk shared memory images to be posted at <serial>";

/* A description of the arguments we accept. */
static char args_doc[] = "serial";

/* The options we understand. */
static struct argp_option options[] = {
  {"nobias",     'b', 0, 0,  "Disable automatically biasing the DM (enabled by default)" },
  { 0 }
};

/* Used by main to communicate with parse_opt. */
struct arguments
{
  char *args[1];                /* serial */
  int nobias;
};

/* Parse a single option. */
static error_t parse_opt (int key, char *arg, struct argp_state *state)
{
  /* Get the input argument from argp_parse, which we
     know is a pointer to our arguments structure. */
  struct arguments *arguments = state->input;

  switch (key)
    {
    case 'b':
      arguments->nobias = 1;
      break;
    case ARGP_KEY_ARG:
      if (state->arg_num >= 1)
        /* Too many arguments. */
        argp_usage (state);

      arguments->args[state->arg_num] = arg;

      break;

    case ARGP_KEY_END:
      if (state->arg_num < 1)
        /* Not enough arguments. */
        argp_usage (state);
      break;

    default:
      return ARGP_ERR_UNKNOWN;
    }
  return 0;
}

/* Our argp parser. */
static struct argp argp = { options, parse_opt, args_doc, doc };


int main(int argc, char* argv[]) {

    struct arguments arguments;

    /* Default values. */
    arguments.nobias = 0;

    /* Parse our arguments; every option seen by parse_opt will
     be reflected in arguments. */
    argp_parse (&argp, argc, argv, 0, 0, &arguments);

    BMCRC rv = controlLoop(arguments.args[0], arguments.nobias);
    if (rv) {
        printf("Encountered error %d.\n", rv);
        return rv;
    }

    return 0;
}