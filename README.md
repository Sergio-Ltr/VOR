First, start the yarpserver typing:  

```bash

yarserver

```

then, on another terminal, start the iCub simulator, typing: 

```bash

iCub_SIM

```

On a separate window the iCub simulator will prompt. 
In order to carry on the simulation, you can enamble the left camera view with two commands: 

```bash

yarpview --name /viewer & 
yarp rpc /icubSim/cam/left /viewer

```

A new window, showing the simulator from the robot persepctive will prompt.
To lounch the VOR application cd to the project directory: 

```bash

cd <yourpath>/VOR

```

Here create a "build" directory, move inside it and launch cmake: 

```bash

mkdir build 
cd build
cmkae ../

```

Then (on Linux) use the make command

```bash

make

```

and an executable will be inside the bin subdirectory. 
To run it just use: 

```bash

./bin/head-controller

```

To run the "static gain" version, use -sg as argument.

```bash

./bin/head-controller -sg

```

I think it anyway works better with the default dynamic gain. 
Enjoy...