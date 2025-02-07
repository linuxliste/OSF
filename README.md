
This repository is a fork of this "mbrusa" project https://github.com/emmebrusa/TSDZ2-Smart-EBike-1 that was developed for TSDZ2 Tongsheng motor

The purpose here is to have a version that can be used on a TSDZ8 Tongsheng motor.

Changes were required mainly because TSDZ8 uses a different microprocessor (XMC1302).

This version is supposed to provide the same functionalities and to support the same displays as the mbrusa project above.

Compare to the TSDZ8 original firmware there are some expected benefits:
* let the user adapt many parameters to his preferences.
* show more data on the display (without having to change the display firmware)

For more information, you can: 
* look at Endless Sphere forum reference thread: [endless-sphere.com.](https://endless-sphere.com/forums/viewtopic.php?f=30&t=110682).
* see the [wiki](https://github.com/emmebrusa/TSDZ2-Smart-EBike-1/wiki) from mbrusa

IMPORTANT : at this stage, this is just a beta version. It has even NOT been tested on a bike and there are probably some bugs.
Try it at your own risk!!!!

Like the TSDZ2 version, this version is supposed to let you change some setup parameters using the display keyboard. Still currently the changes done with the display are lost after a power off. This should be fixed in a future version.

To use this firmware, you will have to:

* Install some softwares to compile this firmware to take care of your preferences
* Donwload this firmware
* Set up parameters to match your preferences
* Compile This firmware
* Use a Segger Jlink device and a cable to connect it to the controller ( to flash the firmware in the TSDZ8 controller); note : this device replace the Stlink used for TSDZ2
* Flash the compiled firmware to the TSDZ8 controller

# 1.Installing the softwares to compile this firmware

This software has been developped with 
 - Modus toolbox (from infineon)
 - Visual Studio Code (and some extensions).
Note: Segger Jlink is also used to flash the controller if you want to do it inside VS Code.

To install those firmwares, you have to follow the instructions provided in this link in the steps 1.1 and 1.2 (and 1.3 if you plan to use Jlink inside VS Code)
https://www.infineon.com/dgdl/Infineon-Visual-Studio-Code-user-guide-UserManual-v04_00-EN.pdf?fileId=8ac78c8c92416ca50192787be52923b2&redirId=248223

This can be quite long but is not very difficult.
There is no need to follow the instructions in chapters 2 and after.

On my side I still had an issue to use Jlink to directly flash/debug the firmware and I had to
- rename the Jlink folder to remove the version nr (so it becomes just "SEEGER")
- edit the .vscode/settings.json file in this project to adapt the paths to Segger tools

# 2.Installing this firmware

Download or clone this repository. 
If you downloaded, unzip the archive where you want.
Still take care to put all the folders inside an empty folder. 

Then there are some more steps to perform:
- open "modus-shell"  (this tool is part of the programs that have been dowloaded with modus toolbox)
- in modus-shell, make the folder named "OSF" the current folder (use "cd" commands)
- then enter in modus-shell the command "make getlibs" ; this will copy several libs
- then enter in modus shell the command "make vscode" ; this will generate/update some files in .vscode folder

Normally you are now ready to use VS Code and to compile.

# 3.Setting up the parameters
The parameters used by this firmware are stored in the file config.h.

They can be manually edited in this file using VS Code.

They are the same as those of mbrusa firmware: see manual https://github.com/emmebrusa/TSDZ2-Smart-EBike-1/blob/master/manuals/EN-Parameter_configurator_guide-TSDZ2-v20.1C.2-2.pdf

Still as this firmware uses the same parameters as the mbrusa project, it is also possible (not tested yet) to use a "mbrusa" graphical configurator to generate the config.h file in a user friendly way. See java program JavaConfigurator.jar that is available at https://github.com/emmebrusa/TSDZ2-Smart-EBike-1.
This requires to follow the instructions of mbrusa manual (except what is related to compiling/flashing/Stlink).

Important note: the JavaConfigurator has been developped by mbrusa to fill the config.h AND automaticaly also compile and flash the firmare on a TSDZ2 controller.
As the TSDZ8 uses another microprocessor, compilation and flashing process are different and will fail when pressing on the button Compile & Flash of the configurator.

I did not tested it, but I expect that before giving a flashing error, the configurator will first generate a config.h file.
This "mbrusa" file can then be copied into this TSDZ8 project (and so replace the default file I provided)


# 4.Compiling this firmware
Open VS Code.



In menu "File", select the option "Open Workspace from File..."

Select a file named "TSDZ8.code-workspace" in folder "OSF"

This should open all files.

Edit the config.h file (if not yet done).

You can then try to compile: select the menu "Terminal" and the option "Run Build Task"

Check if errors are generated. It is not abnormal to get quite many "Warnings" (but not "errors").
This step created a HEX file that can be flashed in the controller.

# 5.Preparing Jlink

You need a Segger Jlink device and a cable.
You can get it from here : https://ebikestuff.eu/en/content/18-tsdz8-firmware-update-using-j-link-jlink-v9-programming-kit

Note: I tested with a chinese clone V9 and it worked too.

For the cable, you can also make your own cable with a speed sensor extension for TSDZ2 like this:https://fr.aliexpress.com/item/1005007479961045.html?spm=a2g0o.order_list.order_list_main.120.21ef5e5bFWfkqS&gatewayAdapt=glo2fra

I cut the cable and connect it based on the diagram given in "DOC" folder in "Diagram-TS 32-bit Mid Drive Motor Controller Programming Cable (EN).pdf". It is safe to check that the extension cable you get uses the same colors.

Note: After cutting the extension cable in 2 parts, I used the one with the female connector and connected it directly to the motor.
I expect (not tested yet) that it is also possible to use the part with the male connector. The advantage is that you can keep the speed sensor cable connected to the motor when you flash. You connect then the Jlink device cable to the second yellow connector present on the speed sensor cable (the one foreseen for lights).  

# 6.Flashing

To flash, you can follow the instructions from here: 
https://ebikestuff.eu/en/content/18-tsdz8-firmware-update-using-j-link-jlink-v9-programming-kit

The HEX file to upload is "mtb-example-xmc-gpio-toggle.hex". It has been generated during the compilation in
folder "test_gpio_in/build/tsdz8_for_GPIO_TEST/Debug".

Note: while flashing, the motor should not be powered by the battery. Disconnect it or at least power it OFF.
It seems possible to keep the motor connected the display but do not press any button on the display.
 
Note : it is also possible to flash the code directly from VS Code at the same time as you compile it but this requires probably to modify some set up in .vscode/settings.json file in order to let VS Code knows the location of some Segger files (depend on the folder where you installed Segger Jlink).

# IMPORTANT NOTES
* Installing this firmware will void your warranty of the TSDZ8 mid drive.
* We are not responsible for any personal injuries or accidents caused by use of this firmware.
* There is no guarantee of safety using this firmware, please use it at your own risk.
* We advise you to consult the laws of your country and tailor the motor configuration accordingly.
* Please be aware of your surroundings and maintain a safe riding style.
