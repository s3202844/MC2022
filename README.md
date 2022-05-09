# Mobile Challenge 2022
**Group name**
## Preparartion
First, install evironment requirements:
```
sudo apt install python3-opencv
pip3 install smbus numpy
```
Besides, you need to install a gyroscope called *MPU6050* to the picar-4wd. The IIC pins of MPU6050 are connected to IIC communication port (SCL/SDA/3V3/GND) **1** of the extension board of picar-4wd, and INT pin is connected to **SIG 0**. Here is an example:

<center class="half">
    <img src="doc/connection.jpg" width="200"/>
    <img src="doc/gyroscope.jpg" width="200"/>
</center>

## Guide
run the command below to let the picar-4wd move along trajectory 1 and finish task1.
```
python src.py 1 1
```
The first 1 represents trajectory id, and the second 1 represents task id.
So, if you want the picar-4wd to move along trajectory 2 and finish task1, you should run the command below:
```
python src.py 2 1
```

## Calibration
You can print *doc\calibration_board.pdf* into an A4 paper and calibrate picamera. This is essential for *solvepnp*. Here is the calibration result:
### Intrinsic Matrix:
$$\left [ \begin{matrix}
249.2451&0&160.5182\\
0&249.4006&122.0914\\
0&0&1\\
\end{matrix} \right ]$$
### Radial Distortion:
$$\left [ \begin{matrix}
0.1759 & -0.2849
\end{matrix} \right ]$$
### Tangential Distortion:
$$\left [ \begin{matrix}
0. & 0.
\end{matrix} \right ]$$

