# Embedded Systems Lab-Kit

The readme is structured as follows:
* [Getting started on lab-computers](#getting-started-with-a-new-board-on-lab-computers)
* [Getting started on your own computer](#getting-started-with-a-new-board-and-environment-on-your-own--computer)
*  [ES-Lab-Kit Software](#provided-software)
*  [ES-Lab-Kit Hardware](#hardware)
*  [Tipps](#tipps)
### Repository Structure
```
├── README.md                           # Readme file
├── Software                            # Folder for ES-Lab-Kit software
│   ├── FreeRTOS-Kernel                 # FreeRTOS kernel
│   │   └── ...
│   ├── bsp                             # Board Support Package for the ES-Lab-Kit hardware
│   │   └── ...
│   └── example                         # Example project to test hardware
│   │   └── ...
├── Tools                               # Helper tools
│   ├── ProjectCreator                  # Program to create new ES-Lab-Kit projects
│   │   └── ...
│   └── scripts                         # Helper scripts
│   │   └── ...
├── ES-Lab-Kit_Schematics_V1_0_A.pdf    # Schematics of the ES-Lab-Kit
├── install_student.sh                  # Install ES-Lab-Kit software on lab-computer
└── uninstall.sh                        # Uninstall ES-Lab-Kit software on lab-computer
```

## Getting started with a new board on Lab-Computers

${{\color{red}\Huge{\textsf{  Note: The installation requires significant space on your AFS volume.\}}}}\$
${{\color{red}\Huge{\textsf{  You will get additional AFS volume space during the course. \}}}}\$

### Lab-Computer Setup and Software Installation
First create an SSH-key on the lab computer and add it to gits.
See the official GitHub instructions [here](https://docs.github.com/en/authentication/connecting-to-github-with-ssh/adding-a-new-ssh-key-to-your-github-account?platform=linux) on how to to create a SSH-key and add it to your gits account. Note that the SSH-key will be stored on your AFS home drive and work on any of the computers in Linux computer rooms.

${{\color{red}\Huge{\textsf{  Note: Do not clone this repository, the install script will clone it. \}}}}\$

Download the file [install_student.sh](https://gits-15.sys.kth.se/mabecker/ES-Lab-Kit/blob/main/install_student.sh).
Make the file executable and execute it.
```
chmod +x install_student.sh
./install_student.sh
```

The script installs all necessary software, VSCode extension, and clones this repository.
After the script is executed, you can find the repository here: `~/Documents/ES-Lab-Kit`

A summary and location of all installed resources is printed at the end of the script.

### Flashing Debugger Firmware (Only done once!)
For a new board, the picoprobe firmware for the Pico2 must be flashed to the hardware once.
This can conveniently be done by mounting the debugger as a storage device.

* Hold the `BOOTSEL` button of the debugger (`SW3`). While the button is pressed, connect the USB-C cable to the right USB-C connector `J5`.
* The board will mount as a device called `RP2350`.
* Open the folder `RP2350` in the finder and copy the file `~/.pico-sdk/debugprobe/v2.2.3/debugprobe_on_pico2.uf2` to it.
* After the file is transferred, the board unmounts automatically.
* Now the board is ready to be used.

${{\color{red}\Huge{\textsf{  Note: Make sure that SW9 is in position 'DBG Power' to power the target MCU. \}}}}\$

### Import the example project into VSCode

1) Open a new VSCode window `File->New Window`.
2) On the welcome screen select `Open...` to open a folder.
3) Navigate to the location you cloned the repository and then `/ES-Lab-Kit/Software/example`, click `Open`.
4) VSCode automatically detects that this is a Raspberry Pi Pico project and asks if we like to import it, click `Yes`.
5) The `Import Pico Project` dialog opens. We can leave the default selection (make sure the SDK version is `2.2.0`) and only click `Import`.
6) Now the project setup is complete and we can run it on the board. To do this, select `Run and Debug` symbol on the left menu bar (4th from the top).
7) Make sure `Pico Debug (Cortex-Debug)` is selected and press the green play button. This will trigger the compilation of the project, transfer the binary to the board, and start a debug session. An automatic breakpoint is set at the beginning of `main()`.
8) Resume execution by pressing the `Continue` button in the debug menu, or press `F5`. Now the program is running on your board.

The expected functionality of the example project is described [here](#example-project). The project uses all peripherals of the board and is therefore a good start to verify that your hardware is functioning correctly.

### Creating a new ES-Lab-Kit project
A helper tool is provided to create new projects with and without FreeRTOS.
The project is created with Python.
For ease of use, the [installation on th elab computers](#lab-computer-setup-and-software-installation) places a wrapper script into your `.local/bin` folder and we can use the command `newembproj` to create a new project for the ES-Lab-Kit.

You can create a new project by executing the following command in your terminal, where `PROJECT_NAME` is the name of your project:
```
newembproj PROJECT_NAME
```
The new project will be located inside the `ES-Lab-Kit` folder, specifically under `ES-Lab-Kit/Software/Projects/PROJECT_NAME`.

By default, the new project includes FreeRTOS.
A bare-metal project can be created by passing passing the option `-noRTOS`.

For FreeRTOS optional tracing of operating system events is supported.
Two versions exist.
By passing the option `-trace` the OS events of each core are stored in respectively different SRAM banks.
By passing the option `-tracePSRAM` the OS events are stored in the larger, but slower, PSRAM memory. This allows for larger event traces, but has also a higher runtime impact.

#### Calling the Python program directly

If you want to use the functionality on your own computer the provided script might not work (it assumed specific installation locations). Here we describe how you can run the project creation app directly.

The implementation files are located in `ES-Lab-Kit/Tools/ProjectCreater`. Install all dependencies (found in the `requirements.txt` file. It is recommended to use a virtial environment.)

A new project with name `PROJECT_NAME` and FreeRTOS is created by executing the following command in a terminal:
```
python ProjectCreator.py PROJECT_NAME
```

A new project with name `PROJECT_NAME` and without FreeRTOS is created by executing the following command in a terminal:
```
python ProjectCreator.py PROJECT_NAME -noRTOS
```

All new projects are created in the folder `ES-Lab-Kit/Software/Projects`.

## Getting started with a new board and environment on your own  computer

This section describes how you can install all needed software on your own Linux computer.
The description closely follows the [official documentation](https://datasheets.raspberrypi.com/pico/getting-started-with-pico.pdf) from Raspberry Pi.
It is recommended to also read the official documentation.
There you can also find details on the installation with different operating systems (Linux, OSX, Windows are all supported!).

**Note: As there are many OS versions and types, we can only provide limited installation support and refer to the [official Raspberry Pi instructions](https://datasheets.raspberrypi.com/pico/getting-started-with-pico.pdf) for installation support.**

If you have a new board, or you want to update the firmware of the debugger, follow the steps described [here](#flashing-debugger-firmware-only-done-once).

### Setup VSCode on Linux
- **Python 3.9 or later** (ensure it’s in your PATH or set in settings)
- **Git 2.28 or later** (ensure it’s in your PATH)
- **Tar** (ensure it’s in your PATH)
- **\[Optional\]** gdb-multiarch for debugging (x86_64 only)
- **\[Optional\]** udev rules installed to use [OpenOCD](https://github.com/raspberrypi/openocd/blob/sdk-2.0.0/contrib/60-openocd.rules) and [picotool](https://github.com/raspberrypi/picotool/blob/master/udev/99-picotool.rules) without `sudo`, for debugging and loading
- For **\[Ubuntu 22.04\]**, install `libftdi1-2` and `libhidapi-hidraw0` packages to use OpenOCD

Install VSCode - Ubuntu from Software Centre

```
sudo apt install gdb-multiarch
sudo apt install binutils-multiarch
cd /usr/bin
sudo ln -s /usr/bin/objdump objdump-multiarch
sudo ln -s /usr/bin/nm nm-multiarch
```

### Installing the Raspberry Pi Pico extension
Open VSCode and select the `Extensions` icon on the left (four squares stacked on each other). Search for "`Raspberry Pi Pico`" and click `Install` button next to the extension.

Alternatively, you can also install the extension via the command line:
```
code --install-extension raspberry-pi.raspberry-pi-pico
```

### Cloning the repository and download the FreeRTOS kernel

Clone this repository
```
git@gits-15.sys.kth.se:mabecker/ES-Lab-Kit.git
cd ES-Lab-Kit
```

After cloning the repository update and initialize submodules.
```
git submodule update --init
```

Alternatively clone the repository with the option `--recursive`.

This will download the correct FreeRTOS kernel to the `Software` folder.
The project CMake-file already configures the variable `FREERTOS_KERNEL_PATH` to use the kernel from here.

### Flashing Debugger Firmware (Only done once!)

Download the pre-compiled firware `debugprobe_on_pico2.uf2` from [here](https://github.com/raspberrypi/debugprobe/releases). You can also download it via this direct link: [debugprobe_on_pico2.uf2](https://github.com/raspberrypi/debugprobe/releases/download/debugprobe-v2.2.3/debugprobe_on_pico2.uf2)

Afterwards, you can follow the [same steps as for lab-computers](#flashing-debugger-firmware-only-done-once).
Note that, depending on where you stored the file `debugprobe_on_pico2.uf2`, you need to adjust the instructions accordingly.

### Import the example project into VSCode
To import the example project, follow the [same steps as in the lab environment](#import-the-example-project-into-vscode ).



## Provided Software

### Board Support Package (BSP)
Includes the BSP for the lab-kit to access all peripherals.

The BSP is based on the Pico [Pico C/C++ SDK](https://www.raspberrypi.com/documentation/microcontrollers/c_sdk.html#sdk-setup).

### Example Project
The example project demonstrates how to access all peripherals and also uses FreeRTOS.

The project has the following functions:
* On the LED square, one LED is active and moving counter clockwise.
* Buttons `SW_5`, `SW_6`, `SW_7` and `SW_8` are used to control the brightness between 0 and 100% with steps of +5%, -5%, +10% and -10% respectively, starting with a brightness of 50%.  
* Red, yellow and green LED are lit in sequence before all are turned off again.
* Switches are used to select what is displayed on the 7-segment display. The active switch with lowest index has priority.
    * `SW_10`: Displays a moving dot animation.
    * `SW_11`: Displays the acceleration in X-axis.
    * `SW_12`: Displays the acceleration in Y-axis.
    * `SW_13`: Displays the acceleration in Z-axis.
    * `SW_14`: Displays the brightnes of the LED-square in %.
    * `SW_15`: Shows the incrementing integer value that is sent over UART on CN1.
    * `SW_16`: Shows the integer value that was received last over UART on CN1.
    * `SW_17`: Will be used for PSRAM tests.

## Hardware

The ES-Lab-Kit hardware combines a target MCU with several peripherals and a debugger on the same PCB.
The target MCU and debugger MCU are identical RP2350A microcontrollers.

The schematics of the ES-Lab-Kit can be found [here](ES-Lab-Kit_Schematics_V1_0_A.pdf).

### Component Datasheets

Below you can find links to the datasheets of the main hardware components on the ES-Lab-Kit.
* [Microcontroller - RP2350](https://datasheets.raspberrypi.com/rp2350/rp2350-datasheet.pdf)
* [LED Controller Driver- HT16K33A](https://www.holtek.com/webapi/116711/HT16K33Av102.pdf)
* [7-Segment - Single Digit SMD Display](https://jlcpcb.com/api/file/downloadByFileSystemAccessId/8603393381133398016)
* [Shift Register - 74HC595D](https://jlcpcb.com/api/file/downloadByFileSystemAccessId/8579708624672903168)
* [3-Axis Accelerometer - MMA8452Q](https://www.nxp.com.cn/docs/en/data-sheet/MMA8452Q.pdf)

## Tipps

### Plotting Serial Data
To display a timeseries of data, the data can be sent to the computer via UART, keeping a space between each value.
Then programs like [Better Serial Plotter](https://hackaday.io/project/181686-better-serial-plotter) can be used to plot the data online.

`Better Serial Plotter` is available for Linux, OSX and Windows.

### Command Line Interface

The command line interface can be used to build and flash the programms to the ES-Lab-Kit without the need of an IDE (such as VSCode in our case).
Here we briefly descirbe how to perform the most important tasks.

#### Building the executable
First navigate to your the directory of your project.
From there, follow the instructions below.
```
mkdir build
cd build
cmake ..
make
```
#### Downloading the binaries to the board
First navigate to your the directory of your project.
From there, follow the instructions below.
Replace `PROGRAM_NAME` with your project name.
```
~/.pico-sdk/openocd/0.12.0+dev/openocd -s ~/.pico-sdk/openocd/0.12.0+dev/scripts -f interface/cmsis-dap.cfg -f target/rp2350.cfg -c "adapter speed 5000" -c "program build/PROGRAM_NAME.elf verify reset exit"
```
Make sure to select the correct path, depending on the location of your `.pico-sdk` folder.
The above is correct for the installation on the lab-computers.

#### Receiving serial output

Use a terminal program (here `screen`). Start terminal with the right name replace `ttyXYZ` with your serial connection id.
```
screen /dev/ttyXYZ 115200
```

Terminate the screen session by pressing `Control+a` and then `d`.
