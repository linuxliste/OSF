
This repository is a fork of this "mbrusa" project https://github.com/emmebrusa/TSDZ2-Smart-EBike-1 that was developed for TSDZ2 Tongsheng motor

The purpose here is to have a version that can be used on a TSDZ8 Tongsheng motor.

Changes were required because TSDZ8 uses a different microprocessor (XMC1302).

This version is supposed to provide the same functionalities and to support the same 4 displays as the mbrusa project above.

Compare to the TSDZ8 original firmware there are some expected benefits:
* The user can adapt many parameters to his preferences.
* The display can show more data (without having to change the display firmware) and allow you to make a few changes to the setup.

As the TSDZ8 version is similar to TSDZ2 version, it is good to look at the TSDZ2 sites: 
* look at Endless Sphere forum reference thread: [endless-sphere.com.](https://endless-sphere.com/forums/viewtopic.php?f=30&t=110682).
* see the [wiki](https://github.com/emmebrusa/TSDZ2-Smart-EBike-1/wiki) from mbrusa

This version differs with :
* it requires a Jlink device instead of a stlink device to flash the firmware and the configuration in the controller
* it does not require to install additionnal software in order to compile the firmware yourself. The firmware is already precompiled on this site. Still if you want to change some code, see below the tools to be installed. 
* it is possible to fine tune one parameter (global offset angle in range -5/+5) in order to take care of litle differences (tolerances) that could exist between different motors. This is optionnal.


# IMPORTANT NOTES
* At this stage, this is just a beta version. It has NOT been tested on a bike and there are probably some bugs. Try it at your own risk!!!!
* Installing this firmware will void your warranty of the TSDZ8 mid drive.
* We are not responsible for any personal injuries or accidents caused by use of this firmware.
* There is no guarantee of safety using this firmware, please use it at your own risk.
* We advise you to consult the laws of your country and tailor the motor configuration accordingly.
* Please be aware of your surroundings and maintain a safe riding style.


To use this firmware, you will have to:
* Donwload this firmware
* Generate a file with your preferences (with a GUI configurator)
* Use a Segger Jlink device and a cable (this device replace the Stlink used for TSDZ2)
* Flash the compiled firmware on the TSDZ8 controller
* Flash also the file containing your preferences
* Fine tune one parameter (optional)

If you have questions on this Tsdz8 project, you can ask on this forum:
https://endless-sphere.com/sphere/threads/new-tsdz8-pswpower.120798/page-12

If you find bugs, best is to open an issue in github : https://github.com/mstrens/OSF


# 1.Download this firmware

Download or clone this repository. 
If you downloaded, unzip the archive where you want.

# 2.Generate the configuration file

There configuration file is generated with the same tool as TSDZ2: javaconfigurator.jar.
To know how to use this tool and the purpose of all parameters, see the TSDZ2 manuals at https://github.com/emmebrusa/TSDZ2-Smart-EBike-1/tree/master/manuals
Note: it could be that the TSDZ2 manual is not 100% up to date (the configurator contains perhaps a few more fields).


At this stage, I made a specific java configurator for TSDZ8 but later on, it will probably be included in the TSDZ2 configuration.

So currently, you have to use the file named OSF_TSDZ8_configurator.jar from this github site.

When you run it, after editing your preferences, click the button "Compile & flash". The configurator will open a popup (tested with windows) and you have to press twice a key. It creates a file "files_to_flash/TSDZ8_config.hex" that has to be flashed(=uploaded)  in the controller. 

Note: This version is just a raw copy of TSDZ2 version and it will also try to compile and flash. Those steps will fail but it is not a problem for TSDZ8 as there is no need for compilation and flashing is done manually with a Jlink device.


# 3.Preparing Jlink

You need a Segger Jlink device and a cable.
You can get it from here : https://ebikestuff.eu/en/content/18-tsdz8-firmware-update-using-j-link-jlink-v9-programming-kit

This link provides also a link to an archive with all the files for flashing. You must download and unzip it.

Note about the harware: I tested with a chinese jlink clone V9 from aliexpress and it worked too.

For the cable, like me, you can also make your own cable with a speed sensor extension cord for TSDZ2 like this:https://fr.aliexpress.com/item/1005007479961045.html?spm=a2g0o.order_list.order_list_main.120.21ef5e5bFWfkqS&gatewayAdapt=glo2fra

I cut the extension cable and connect it directly to the Jlink flat cable based on:
- the diagram of Jlink connector : https://www.segger.com/products/debug-probes/j-link/technology/interface-description/
- the picture of speed cable connector : https://empoweredpeople.co.uk/2020/05/28/tongsheng-tsdz2-what-firmware-options-are-there/
In my case (be careful the colors may be different)
- red = SWDIO = TMS = pin 7 of Jlink
- black = SWCLK = TCK = pin 9 of Jlink
- brown = Vcc = VTRef = pin 1 of Jlink
- orange = Grnd = e.g. pin 4 of Jlink

Note: it is usefull to be able to connect/disconnect the VCC/VtRef wire.
When flashing, best is to avoid to power the motor from the main battery. VTref can be connected in order to let the PC provide the power supply (by default only when using a Jlink clone and not an official one).
When monitoring/fine tuning some parameters, the motor must be powered from the main battery. It is then better not to connect VtRef to avoid voltage conflict on Jlink (TSDZ8 deliver 5V, Jlink 3.3V). I expect there is a protection but I am not 100% sure.

Note: if you make your own cable with an extension cord, it can be usefull to be able to use it as well to flash the controller as well to monitor motor parameters and also as extension cord.
To do so, cut the extension cord and resolder the 6 wires and solder also the 4 wires to the Jlink device.

Note : when you want to use the cable for flashing/monitoring the controller, you always have to connect the cable directly to the controller. It means that you have to disconnect the speed sensor from the controller. If you build your own cable and keep it as extension cord, you can then reconnect the speed sensor at the other end (e.g. to monitor some parameters while the wheel is turning).   

# 4.Flash the firmware

To flash, you can follow the instructions from here to know how to use Jlink: 
https://ebikestuff.eu/en/content/18-tsdz8-firmware-update-using-j-link-jlink-v9-programming-kit

The HEX file to upload is the one you downloaded from this github site at step 1.
It is in folder "files_to_flash" and named OSF_TSDZ8_Vxx_xx.hex where xx.xx is a version number.

While flashing, the motor should best not be powered by the battery. Disconnect it or at least power it OFF. In principe, the Jkink will provide power to the controller (at least if it is a Jlink clone device). It seems possible to keep the motor connected to the display but do not press any button on the display.

Note: if you are using an original Segger Jlink device, you probably have to configure it to let it generate the power supply on the pin Vref in order to provide power supply to the controller. Otherwise, you can try to power on the controller with the battery (and pressing the power button on the display). Still I did not tested this method.

# 5.Flash the file with your configuraton

Flash the HEX file your created at step2 in the same way, you flashed the firmware.

Note:
There is normally no need to re-flash the firmware each time you change the HEX file with other preferences.

Still if there are deep changes (in the firmware and/or the sheet), the Version nr in the sheet will be changed.
When the controller starts running, it checks that the firmware is compatible with the parameter version.
If not, it provides an error code  E09 on the display (and blocks the motor).  

# 6. Fine tune one parameter.

The motor use hall sensors to know the position of the rotor and syncronize the magnetic flux.
There can be minor differences in the positions, the sensitivity and the hysteresis of those sensors.
Optionnally, you can fine tune one parameter (the "global offset" applied to hall sensors).

If you want to do this step, you have to:

* REMOVE THE CHAIN FROM THE MOTOR!! (so the motor can freely run without driving the wheel)
* install on you PC (windows) a tool uc_probe provided by Infineon at this link https://infineoncommunity.com/uC-Probe-XMC-software-download_ID712 . It requires that you fill in the register form to dowload it.
- once unzip and installed, run this software
- in to upper left corner, select settings and then select the options Jlink, 4000 kHz , SWD and little endian.
- in the menu File, select Open and in the file explorer, go to the folder where you put this TSDZ8 and select the file "test_uc_probe.wspx"
- in the lowest panel named "Symbol browser", there should be a file name displayed ("mtb-example-xmc-gpio-toggle.elf"). It could be that it do not work the first time you try because I expect the software uses an absolute path to the file (and you are using your own path). In this case, click on the ELF button and it will open a popup to select the HEX file. In this popup, go to folder build/last_config and select the file named "mtb-example-xmc-gpio-toggle.elf". This step should not be required if you reopen the software later on.
- the central panel should now displays different fields and boxes.
- power on the motor (with the battery and the display) and connect the Jlink device between the motor and the PC.
- in uc_probe, in the upper left corner, click on the Run button (green arrow).
- you should see some values that this program collects from the controller. 
- the first row of fields are just for information.
- the ON/OFF button near "Testing" allows to switch between normal running mode and Testing
- when ON, the motor should immediately start running (do not forget to remove the chain before this step).
- you can change the 2 sliders just below. Set the duty cycle in order to let the motor run quite fast (depends on the battery voltage) without driving a to high current.
- the principle consists to change the "Offset" slider position in order to get the lowest "Average current" for a given "Duty cycle" 
- the offset value providing the lowest average current has to be noted and filled in the configurator. Note: the current configurator still does not allow to fill this parameter. It will be added in a future version.
- generate a new configuration file and flash it like in step 5.

Note: when Testing is ON, the firmware discard the wheel speed sensor (because the Jlink uses the same connector).
If Jlink is connected while Testing is OFF, the firmware will give an error on the display (after a short delay) because it does not get the expected signal from the speed sensor. To avoid this error, you can enable the option "Assist with error". 

# Developper

If you want, you can look at the software and modify it.

This software has been developped with 
 - Modus toolbox (from infineon)
 - Visual Studio Code (and some extensions).
Note: Segger Jlink is also used to flash the controller if you want to do it inside VS Code.

To install those firmwares, you have to follow the instructions provided in this link in the steps 1.1 and 1.2 (and 1.3 if you plan to use Jlink inside VS Code)
https://www.infineon.com/dgdl/Infineon-Visual-Studio-Code-user-guide-UserManual-v04_00-EN.pdf?fileId=8ac78c8c92416ca50192787be52923b2&redirId=248223

This can be quite long but is not very difficult.
There is no need to follow the instructions in chapters 2 and after.

Note: on my side I still had an issue to use Jlink to directly flash/debug the firmware and I had to
- rename the Jlink folder to remove the version nr (so it becomes just "SEEGER")
- edit the .vscode/settings.json file in this project to adapt the paths to Segger tools

Still take care to put all the folders inside an empty folder in the case you would like to develop/compile your self. The m 

Then there are some more steps to perform:
- open "modus-shell"  (this tool is part of the programs that have been dowloaded with modus toolbox)
- in modus-shell, make the folder named "OSF" the current folder (use "cd" commands)
- then enter in modus-shell the command "make getlibs" ; this will copy several libs
- then enter in modus shell the command "make vscode" ; this will generate/update some files in .vscode folder

Normally you are now ready to use VS Code and to compile.

To compiling the firmware:
* Open VS Code.
* In menu "File", select the option "Open Workspace from File..."
* Select a file named "TSDZ8.code-workspace" in folder "OSF".This should open all files.
* Select the menu "Terminal" and the option "Run Build Task"

Check if errors are generated. It is not abnormal to get quite many "Warnings" (but not "errors").
This step created a HEX file that can be flashed in the controller.
 
Note : it is also possible to flash the code directly from VS Code at the same time as you compile (e.g. to debug it) but this requires probably to modify some set up in .vscode/settings.json file in order to let VS Code knows the location of some Segger files (depend on the folder where you installed Segger Jlink).

Note: if you compile the firmware yourself, the generated hex file to upload is named "mtb-example-xmc-gpio-toggle.hex" and is present in
folder "test_gpio_in/build/tsdz8_for_GPIO_TEST/Debug" or in "test_gpio_in/build/last_config" (depending if you use the menu Run/start debugging or the menu Terminal/Run build task)