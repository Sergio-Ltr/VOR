Implementation of a simplified vestibular ocular reflex. 



Two periodic threads are used: 

1) Movement Thread (Called EyeMover in the code)

2) Vision Thread



The vision thread only handles image related tasks: it connects to the camera port, computes the estimation of the sphere center(assuming it is red), and send the result to an internal buffered port. This result is required by the movement thread at each cycle, after having imposed the sinusoidal movement of the head. Once the coordinates have been received, difference with respect to the desired position (160,120) is computed and used to correct eyes position. Only a proportional correction term has been implemented, with empirically found values for the K constant, which differs for the X and the Y axis stabiliziation. Horizontal one requires a negative term. 


An additional experimentation was proposed. Since different values of KPx and KPy seemed to be required when the head movement speed changed, intuition was the one to use the frequence of the sinusoidal movement as a multiplication factor for both the KPs, douboing them at each iteration too. This solution seems to work better in terms of stability of the sphere at the center of the view. It is still possible to use the constant KPs just adding the -sg option when executing this program (more instructions at the bottom of the README file).


To run the simulation ensure YARP and iCub simulator are correctly working on your machine. 


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
yarp connect /icubSim/cam/left /viewer

```

A new window, showing the simulator from the robot persepctive will prompt.
To lounch the VOR application cd to the project directory: 

```bash

cd <yourpath>/VOR-YARP

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
