# BMC-Interface
Interface the modern BMC API with cacao

To compile with cacao and the BMC SDK:

    gcc -o build/runBMC2K runBMC2K.c -I/opt/Boston\ Micromachines/include -L/opt/Boston\ Micromachines/lib -Wl,-rpath-link,/opt/Boston\ Micromachines/lib -lBMC -lBMC_PCIeAPI -lncurses -lImageStreamIO -lpthread -lrt -lm
    
To enter the control loop with the default settings:

    ./runBMC2K "<DM serial number>"
    
This creates a shared memory image of the name <DM serial number> which can be updated via cacao. Inputs are expected as single-precision floats between 0 and 1 (normalized voltage). `ctrl+c` will exit the loop and safely reset and release the DM.
  
To run with biasing disabled:

    ./runBMC2K "<DM serial number>" --nobias
    
For help:

    ./runBMC2K --help
