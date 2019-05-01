/*
To compile:
gcc -o build/runBMC2K runBMC2K.c -I/opt/Boston\ Micromachines/include -L/opt/Boston\ Micromachines/lib -Wl,-rpath-link,/opt/Boston\ Micromachines/lib -lBMC -lBMC_PCIeAPI -lncurses -lImageStreamIO -lpthread -lrt -lm -lcfitsio

To run:
./runBMC2K <serial> <shared_memory_name> --bias <bias_value> --linear --fractional
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

/* FITS */
#include "fitsio.h"

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
void initializeSharedMemory(const char * shm_name, uint32_t ax1, uint32_t ax2)
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
    imsize[0] = ax1;
    imsize[1] = ax2;
    
    // image will be float type
    // see file ImageStruct.h for list of supported types
    atype = _DATATYPE_FLOAT;
    // image will be in shared memory
    shared = 1;
    // allocate space for 10 keywords
    NBkw = 10;
    // create an image in shared memory
    ImageStreamIO_createIm(&SMimage[0], shm_name, naxis, imsize, atype, shared, NBkw);

    /* flush all semaphores to avoid commanding the DM from a 
    backlog in shared memory */
    ImageStreamIO_semflush(&SMimage[0], -1);
    
    // write 0s to the image
    SMimage[0].md[0].write = 1; // set this flag to 1 when writing data
    int i;
    for (i = 0; i < ax1*ax2; i++)
    {
      SMimage[0].array.F[i] = 0.;
    }

    // post all semaphores
    ImageStreamIO_sempost(&SMimage[0], -1);
        
    SMimage[0].md[0].write = 0; // Done writing data
    SMimage[0].md[0].cnt0++;
    SMimage[0].md[0].cnt1++;
}

/* BMC expects inputs between 0 and +1, but we'd like to provide
stroke values in physical units. This function makes two conversions:
1. It converts from microns of stroke to fractional voltage. 
2. It normalizes inputs such that volume displaced by the requested command roughly
matches the equivalent volume that would be displaced by a cuboid of dimensions
actuator-pitch x actuator-pitch x normalized-stroke. This is a constant factor 
that's found by calculating the volume under the DM influence function.

This requires DM calibration.
 */
void scale_inputs(float * command, uint32_t ActCount, float scale)
{
    int idx;
    // normalize each actuator stroke
    for ( idx = 0 ; idx < ActCount ; idx++)
    {
        command[idx] *= scale;
    }
}

/* Remove DC bias in inputs to maximize actuator range.
There's something not quite right about my approach.
Ex: requesting full stroke on one actuation only
slightly changes the avg, so it gets clipped
at some value very short of full stroke. */
void bias_inputs(float * command, float bias, uint32_t ActCount)
{
    int idx;
    float mean;

    // calculate mean value
    mean = 0;
    for ( idx = 0 ; idx < ActCount ; idx++)
    {
        mean += command[idx];
    }
    mean /= ActCount;

    /* Remove mean from each actuator input
    and add voltage bias to center of range.
    */
    for ( idx = 0 ; idx < ActCount ; idx++)
    {
        command[idx] += bias - mean;
    }
}

/* Convert any DM inputs to [0, 1] to avoid 
exceeding safe DM operation. */
double clip_to_limits(double command)
{
    if (command > 1.0) {
        command = 1.0;
    } else if (command < 0.0) {
        command = 0.0;
    }
    return command;
}

/* Read in a configuration file with user-calibrated
values to determine the conversion from physical to
fractional stroke as well as the volume displaced by
the influence function. */
int parse_calibration_file(const char * serial, float *act_gain, float *volume_factor)
{
    char * bmc_calib;
    char calibpath[1000];
    char serial_lc[1000];
    FILE * fp;
    char * line = NULL;
    size_t len = 0;
    ssize_t read;
    char * token;
    float * calibvals;

    // find calibration file location from bmc_calib env variable
    bmc_calib = getenv("bmc_calib");
    if (bmc_calib == NULL)
    {
        printf("'bmc_calib' environment variable not set!\n");
        return -1;
    }
    strcpy(calibpath, bmc_calib);
    strcat(calibpath,  "/bmc_2k_userconfig.txt");

    // open file
    fp = fopen(calibpath, "r");
    if (fp == NULL)
    {
        printf("Could not read configuration file at %s!\n", calibpath);
        return -1;
    }

    calibvals = (float*) malloc(2*sizeof(float));
    int idx = 0;
    while ((read = getline(&line, &len, fp)) != -1)
    {
        // grab first value from each line
        calibvals[idx] = strtod(line, NULL);
        idx++;
    }

    fclose(fp);

    // assign stroke and volume factors
    (*act_gain) = calibvals[0];
    (*volume_factor) = calibvals[1];

    printf("BMC %s: Using stroke and volume calibration from %s\n", serial, calibpath);
    return 0;
}

int get_actuator_mapping(const char * serial_number, int nbAct, int * actuator_mapping)
{
    /* This function closely follows the CFITSIO imstat
    example */

    fitsfile *fptr;  /* FITS file pointer */
    int status = 0;  /* CFITSIO status value MUST be initialized to zero! */
    int hdutype, naxis, ii;
    long naxes[2], totpix, fpixel[2];
    int *pix;
    int ij = 0; /* actuator mapping index */

    char * bmc_calib;
    char calibname[1000];
    char calibpath[1000];

    // get file path to actuator map
    bmc_calib = getenv("bmc_calib");
    strcpy(calibpath, bmc_calib);
    sprintf(calibname, "/bmc_2k_actuator_mapping.fits");
    strcat(calibpath, calibname);

    if ( !fits_open_image(&fptr, calibpath, READONLY, &status) )
    {
      if (fits_get_hdu_type(fptr, &hdutype, &status) || hdutype != IMAGE_HDU) { 
        printf("Error: this program only works on images, not tables\n");
        return(1);
      }

      fits_get_img_dim(fptr, &naxis, &status);
      fits_get_img_size(fptr, 2, naxes, &status);

      if (status || naxis != 2) { 
        printf("Error: NAXIS = %d.  Only 2-D images are supported.\n", naxis);
        return(1);
      }

      pix = (int *) malloc(naxes[0] * sizeof(int)); /* memory for 1 row */

      if (pix == NULL) {
        printf("Memory allocation error\n");
        return(1);
      }

      totpix = naxes[0] * naxes[1];
      fpixel[0] = 1;  /* read starting with first pixel in each row */

      /* process image one row at a time; increment row # in each loop */
      for (fpixel[1] = 1; fpixel[1] <= naxes[1]; fpixel[1]++)
      {  
         /* give starting pixel coordinate and number of pixels to read */
         if (fits_read_pix(fptr, TINT, fpixel, naxes[0],0, pix,0, &status))
            break;   /* jump out of loop on error */

         
         for (ii = 0; ii < naxes[0]; ii++) {
           if (pix[ii] > 0) {
                // get indices of active actuators in order
                // ij-th pixels maps to actuator pix[ii]
                actuator_mapping[pix[ii] - 1] = ij;//(fpixel[1]-1) * naxes[0] + ii;
                //printf("Actuator %d at address %d\n", pix[ii] - 1, ij);
           }
           ij++;
         }
      }
      fits_close_file(fptr, &status);
    }

    if (status)  {
        fits_report_error(stderr, status); /* print any error message */
    }

    free(pix);

    printf("BMC %s: Using actuator mapping from %s\n", serial_number, calibpath);
    return 0;
}

BMCRC sendCommand(DM hdm, double *command, uint32_t *map_lut, IMAGE * SMimage, double bias, int linear, int fractional, float act_gain, float volume_factor, int * actuator_mapping, uint32_t ActCount) {
    // Initialize variables
    int idx, address;
    BMCRC rv;
    double mean;

    // Loop 1: pull command from shared memory and scale/convert as requested
    for (idx = 0; idx < ActCount; idx++) {

        // use actuator mapping to pull correct element of shared memory image
        address = actuator_mapping[idx];
        if (address == -1) {
            /* addressable but ignored actuators should
            always be set to 0. */
            command[idx] = 0.; 
        }
        else {
            /* addressable and active actuators have an integer
            address to their location in the command vector */
            command[idx] = (double)SMimage[0].array.F[address]; // test if you can get rid of the double
        }

        /* If inputs are given in microns, convert from micronts
        to fractional volts */ 
        if (fractional == 0) {
            command[idx] *= volume_factor / act_gain;
        }

        /* Keep track of the mean. Only used if we're explicitly
        biasing the inputs */
        mean += command[idx];
    }

    mean /= ActCount;

    /* Loop 2: apply a bias (if requested), clip commands to safe limits, and
    take the sqrt (if requested), */
    for (idx = 0; idx < ActCount; idx++) {

        /* Note that the bias is applied in fractional volts before sqrt,
        so it can mean different things:
        Bias = 0.5 with linear==1 -> 0.5 fractional volts applied to DM
        Bias = 0.5 with linear==0 (default) -> 0.7 fractional volts applied to DM
        */
        if (bias > 0.) {
            // Remove the mean from the commands and apply the requested bias
            command[idx] += bias - mean;
        }

        /* Clip to limits (0, 1)
        Must happen before square root to avoid invalid entries from sqrt(-x)
        but after the bias to avoid clipping commands that would be shifted
        to valid values by the bias
        */
        clip_to_limits(command[idx]);

        /* If requested, take the sqrt of inputs. If inputs
        are given in microns, you should always take the sqrt
        (otherwise the conversion is nonsense), but I'm not
        enforcing this since the option to send fractional volts 
        with and without the sqrt option is useful */
        if (linear == 0) {
            command[idx] =  sqrt(command[idx]);
        }
    }


    //for (idx = 0; idx < ActCount; idx++) {
    //    printf("Act %d: %f\n", idx, command[idx]);
    //}

    // Send command
    rv = BMCSetArray(&hdm, command, map_lut);
    // Check for errors
    if(rv) {
        printf("Error %d sending voltages.\n", rv);
        return rv;
    }

    return 0;
}

// intialize DM and shared memory and enter DM command loop
int controlLoop(const char * serial_number, const char * shm_name, double bias, int linear, int fractional) {

    // Initialize variables
    DM hdm = {};
    BMCRC rv;
    int idx;
    uint32_t ActCount;
    uint32_t *map_lut;
    IMAGE * SMimage;
    int *actuator_mapping; // 50x50 image to 1D vector of commands
    uint32_t shm_dim = 50; // Hard-coded for now

    // command vector
    double *command;

    float act_gain, volume_factor; // calibration

    /* get actuator gain and volume normalization factor from
    the user-defined config file */
    rv = parse_calibration_file(serial_number, &act_gain, &volume_factor);
    if (rv == -1)
    {
        return -1;
    }

    // Open driver
    rv = BMCOpen(&hdm, serial_number);
    // Check for errors
    if(rv) {
        printf("Error %d opening the driver type %u.\n", rv, (unsigned int)hdm.Driver_Type);
        printf("%s\n\n", BMCErrorString(rv));
        return rv;
    }
    ActCount = (uint32_t)hdm.ActCount;

    printf("Opened Device %d with %d actuators.\n", hdm.DevId, ActCount);

    // Load actuator map (BMC SDK specific)
    map_lut = (uint32_t *)malloc(sizeof(uint32_t)*MAX_DM_SIZE);
    for(idx=0; idx<ActCount; idx++) {
        map_lut[idx] = 0;
    }
    rv = BMCLoadMap(&hdm, NULL, map_lut);

    /* get actuator mapping from 2D cacao image to 1D vector for
    ALPAO input */
    actuator_mapping = (int *) malloc(ActCount * sizeof(int)); /* memory for actuator mapping */
    /* initialize to -1 to allow for handling addressable but ignored actuators */
    for (idx=0; idx<ActCount; idx++) {
        actuator_mapping[idx] = -1;
    }
    get_actuator_mapping(serial_number, ActCount, actuator_mapping);

    // initialize shared memory image to 0s
    initializeSharedMemory(shm_name, shm_dim, shm_dim);
    // connect to shared memory image (SMimage)
    SMimage = (IMAGE*) malloc(sizeof(IMAGE));
    ImageStreamIO_read_sharedmem_image_toIMAGE(shm_name, &SMimage[0]);

    // Validate SMimage dimensionality and size against DM
    if (SMimage[0].md[0].naxis != 2) {
        printf("SM image naxis = %d\n", SMimage[0].md[0].naxis);
        return -1;
    }
    if (SMimage[0].md[0].size[0] != shm_dim) {
        printf("SM image size (axis 1) = %d", SMimage[0].md[0].size[0]);
        return -1;
    }
    if (SMimage[0].md[0].size[1] != shm_dim) {
        printf("SM image size (axis 2) = %d", SMimage[0].md[0].size[1]);
        return -1;
    }

    // initialize command vectors outside of the control loop
    command = (double*)calloc(ActCount, sizeof(double));

    // set DM to all-0 state to begin
    printf("BMC %s: initializing all actuators to 0.\n", serial_number);
    ImageStreamIO_semwait(&SMimage[0], 0);
    rv  = sendCommand(hdm, command, map_lut, SMimage, bias, linear, fractional, act_gain, volume_factor, actuator_mapping, ActCount);
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
        //printf("BMC %s: waiting on commands.\n", serial_number);
        // Wait on semaphore update
        ImageStreamIO_semwait(&SMimage[0], 0);
        
        // Send Command to DM
        if (!stop) { // Skip DM on interrupt signal
            rv = sendCommand(hdm, command, map_lut, SMimage, bias, linear, fractional, act_gain, volume_factor, actuator_mapping, ActCount);
            if (rv) {
                printf("Error %d sending command.\n", rv);
                return rv;
            }
        }
    }

    free(command);

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
  "runBMC2K-- enter the BMC2K DM command loop and wait for cacao shared memory images to be posted at <shm_name>";

/* A description of the arguments we accept. */
static char args_doc[] = "[serial] [shared memory name]";

/* The options we understand. */
static struct argp_option options[] = {
  {"bias",       'b', "bias", 0,  "Remove mean from all commands and add a fixed bias level in fractional volts. By default, this is disabled and assumes the user will build the bias into the flat command. The bias is applied\
  before the square root of inputs is taken (if enabled), so bias=0.5 -> 0.7 fractional volts." },
  {"linear",     'l', 0,      0,  "By default, the square root of inputs is sent to the DM. Toggling 'linear' disables this." },
  {"fractional", 'f', 0,      0,  "Disable multiplication by gain and volume factors. Toggling 'fractional' means commands are expected in the range [0,1]." },
  { 0 }
};

/* Used by main to communicate with parse_opt. */
struct arguments
{
  char *args[2];                /* serial shm_name*/
  double bias;
  int linear, fractional;
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
      arguments->bias = atof(arg);
      break;
    case 'l':
      arguments->linear = 1;
      break;
    case 'f':
      arguments->fractional = 1;
      break;
    case ARGP_KEY_ARG:
      if (state->arg_num >= 2)
        /* Too many arguments. */
        argp_usage (state);

      arguments->args[state->arg_num] = arg;

      break;

    case ARGP_KEY_END:
      if (state->arg_num < 2)
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
    arguments.bias = 0.0;
    arguments.linear = 0;
    arguments.fractional = 0;

    /* Parse our arguments; every option seen by parse_opt will
     be reflected in arguments. */
    argp_parse (&argp, argc, argv, 0, 0, &arguments);

    BMCRC rv = controlLoop(arguments.args[0], arguments.args[1], arguments.bias, arguments.linear, arguments.fractional);
    if (rv) {
        printf("Encountered error %d.\n", rv);
        return rv;
    }

    return 0;
}