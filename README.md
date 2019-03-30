# BMC-Interface
Interface the modern BMC API with cacao

To compile with cacao and the BMC SDK:

    gcc -o build/runBMC2K runBMC2K.c -I/opt/Boston\ Micromachines/include -L/opt/Boston\ Micromachines/lib -Wl,-rpath-link,/opt/Boston\ Micromachines/lib -lBMC -lBMC_PCIeAPI -lncurses -lImageStreamIO -lpthread -lrt -lm -lcfitsio
    
To enter the control loop with the default settings:

    ./runBMC2K "<DM serial number>" <shared memory image>
    
This connects to the DM and creates a shared memory image which can be updated via cacao. Inputs are expected as single-precision floats in a 50x50 array in units of microns. `ctrl+c` will interrupt the control loop and safely reset and release the DM.
  
To run with mean-subtraction and a fixed bias (given in fractional volts between 0 and 1):

    ./runBMC2K "<DM serial number>" <shared memory image> --bias=value
    
To disable taking the square root of the inputs (which is enabled by default):

     ./runBMC2K "<DM serial number>" <shared memory image> --linear
     
 To send commands in fractional volts rather than microns:
 
    ./runBMC2K "<DM serial number>" <shared memory image> --fractional
    
For help:

    ./runBMC2K --help
    
Before running, set the path to a user configuration file

    export bmc_calib=$/some/path/

where the bmc_calib directory contains:

    bmc_2k_actuator_mapping.fits #2D image of actuator positions
    bmc_2k_actuator_mask.fits #2D binary image of active/inactive actuators
    bmc_2k_userconfig.txt #calibrated gain and volume conversion factors

and bmc_2k_userconfig.txt is a plaintext file with content:

    -1.1572 # actuator gain (microns/fractional voltage^2)
    0.5275 # volume conversion factor

    


