[![View MPC_Node.js_Framework on File Exchange](https://www.mathworks.com/matlabcentral/images/matlab-file-exchange.svg)](https://www.mathworks.com/matlabcentral/fileexchange/73930-mpc_node-js_framework)

# MPC Node.js
This framework generates a Model Predictive Control (MPC) using JavaScript to run the code on Node.js Platform. 
This project can be run on any development board that support Node.js and programed with JavaScript such as Intel Galileo/Edison boards.

## Prerequisites
You need to install these software programs to insure that the project will run correctly:
* MATLAB R2011a or latest. 
* [Node.js Platform](https://nodejs.org/en/download/current/) 'latest version'. 
* If you will run the project on Intel Galileo/Edison development board, download [Intel XDK]() with image software of Galileo board and the [Bonjour Software](https://bonjour.en.softonic.com/) for Windows users.

## Installing and Deployment
* Download the project folder.
* Open the folder with MATLAB.
* Run the script that generates the JacaScript code of the MPC by writing this command and follow the instructions:
```
MPC_init
```
* Enter the continuous-time state-space model of the system and its sampling time (Ts).
* Enter the rest parameters of the MPC designing such as prediction and control horizons, lower and upper limitations of the Manipulated Variables (MV), interval-to-interval change for the MV and Output Variables (OV).
* Set the set-point that you want to test the system on.
* Define the analog/digital pins of you will run the code on the development board.
* Determine if you need to use *Kalman Filter* with your process to reduce the noises from the sensor and set the required parameters. 
* After finishing the generated code will be found in **main.js** file.
* If you will test the code on the Galileo board using Intel XDK, copy the **main.js** file to the **MPC_RealTime** foler and open the project with the intel XDK.
* Manage your pins and modified your code if you use more than one analog/digital pin. 

### Testing
You can use the state-space system given in the folder by loading the .mat file by typing this command:
```
load sys_node_mpc.mat
```
Use the valriable *sys* as an example for a state-space system.

## Author
 **Al-Shaimaa A. Younis** - *CSE Department, Faculty of Engineering, Minia University, Egypt*
