# midi2piousbhub
Use a Raspberry Pi Pico to interconnect a MIDI host, a serial MIDI device and
up to 4 MIDI devices via a USB hub. If you have a Pico W or compatible board,
this project supports connection from a Bluetooth MIDI client (iPad, phone,
PC...) or connection to a Bluetooth MIDI server (Bluetooth MIDI Keyboard,
control surface, etc.)

This project uses the RP2040 processor's built-in USB port as a USB device
port for connection to a USB MIDI host like a PC or Mac. It uses the RP2040's
PIOs plus 2 GPIO pins to create a USB host port, and it uses the RP2040
processor's built-in UART1 port for serial port MIDI. Bluetooth MIDI uses
the Pico W's built-in WiFi/Bluetooth module.

You configure how the MIDI streams connect by using serial terminal command
line interface (CLI) connected to either the RP2040's UART0 port or the RP2040's
built-in USB port's serial (CDC ACM) interface.

The software uses some of the Pico board's program flash for a file system
to store configurations in presets. If you save your settings to a preset, then
the midi2piousbhub software will automatically reload the last saved preset on startup
and when you plug a USB MIDI Device to the host port. You can back up any or all of
your presets to a USB Flash drive connected to the USB hub. Presets are stored in
JSON format.

# Hardware
My first test circuit used a Raspberry Pi Pico board, a USB A breakout board,
and a hand wired MIDI I/O port. The circuit is the same as the hardware described
in the [usb_midi_host README](https://github.com/rppicomidi/usb_midi_host/blob/main/README.md)
 "Software-based USB Host Port: Pico_PIO_USB Library" section
except I wired a MIDI IN and MIDI OUT port to pins GP4 and GP5 like the
[midi2usbhost](https://github.com/rppicomidi/midi2usbhost) project shows.

However, it should also run on the [Adafruit Feather RP2040 with USB Type A Host](https://learn.adafruit.com/adafruit-feather-rp2040-with-usb-type-a-host/overview)
with pins D4 and D5 wired to an
[Adafruit MIDI FeatherWing](https://learn.adafruit.com/adafruit-midi-featherwing)
board (take FeatherWing 3.3V power and ground from the RP2040 board), or any
similar hardware configuration.

# Setting Up Your Build and Debug Environment
I am running Ubuntu Linux 24.04LTS on an old PC. I have Visual Studio Code (VS Code)
installed and went
through the tutorial in Chapter 7 or [Getting started with Raspberry Pi Pico](https://datasheets.raspberrypi.com/pico/getting-started-with-pico.pdf) to make sure it was working
first. I use a picoprobe for debugging, so I have openocd running in a terminal window.
I use minicom for the serial port terminal (make sure your linux account is in the dialup
group). I found that when debugging using the picoprobe whilst also using the Pico board as
a USB device connected to the same PC, I had to connect the picoprobe first, then
connect the target Pico board via a USB hub. Your experience may be different.

You do not need to use the picoprobe or the UART 0 output at all. The USB device
connector on the RP2040 target board serves both as a MIDI port and serial port
console.

## Install the Pico C/C++ SDK
If you have not already done so, follow the instructions for installing the Raspberry Pi Pico SDK in Chapter 2 of the 
[Getting started with Raspberry Pi Pico](https://datasheets.raspberrypi.com/pico/getting-started-with-pico.pdf)
document. In particular, make sure `PICO_SDK_PATH` is set to the directory where you installed the pico-sdk.

## Pico W Users: Update the Pico SDK
At the time of this writing, the Pico C/C++ SDK version is 1.5.1. It has some
issues with Bluetooth Support that are fixed in the `develop` branch of the pico-sdk.
To address these issues, please check out the `develop` branch of the pico-sdk and
get the latest version.
```
cd ${PICO_SDK_PATH}
git checkout -b develop origin/develop
git pull
git submodule update -- lib/btstack
git submodule update -- lib/cyw43-driver
```

## Use a TinyUSB library version that supports application host drivers
The USB MIDI host driver is currently not part of the TinyUSB stack. It is an
application host driver found in this project's `lib/usb_midi_host` directory.
The Pico SDK uses the main repository for TinyUSB as a git submodule. The version
of TinyUSB that ships with the Pico SDK 1.5.1 does not support application host
drivers. That feature was added 15-Aug-2023. You will likely need the latest version
of the TinyUSB library for this code to work correctly. The following describes
how to make sure your Pico SDK version's TinyUSB library supports application host
drivers.

1. Set the working directory to the tinyusb library and make sure you are on the main branch.
```
cd ${PICO_SDK_PATH}/lib/tinyusb
git checkout master
```
2. Check the date on the last commit to the TinyUSB library master branch.
```
git log -1
```
3. If the `Date:` is >= 15-Aug-2023, your TinyUSB library should be fine. If not, get the latest
```
git pull
```

## Install PIO USB support for TinyUSB
The latest TinyUSB does not come with PIO USB driver source code.
To install that source, you need to have Python 3 installed on
your computer. From a command line, type
```
cd ${PICO_SDK_PATH}/lib/tinyusb
python tools/get_deps.py rp2040
```
For more information, see the [TinyUSB documentation](https://docs.tinyusb.org/en/latest/reference/getting_started.html#dependencies)

## Update the PIO USB Driver to fix memory corruption
Until version 0.6.0, the [Pico-PIO-USB](https://github.com/sekigon-gonnoc/Pico-PIO-USB) project consumed
most of the 32 PIO 0 instructions and all of the PIO 1 instructions. Version 0.6.0 and later
only uses PIO 0. The CYW43 Bluetooth radio module on the Pico W communicates
with the Pico W's RP2040 chip via a custom SPI driver implemented in PIO 1.
Before version Pico-PIO-USB version 0.6.0, I found that the Pico W CYW43 SPI PIO driver
would lock up from time to time if both PIO USB was active and there was a lot
of data transfer between the CYW43 module and the RP2040 (such as during Bluetooth
LE client mode active scanning). Switching to version 0.6.0 seems to have fixed
most of that issue. However, version 0.6.1 fixed a memory corruption issue, so
it is better to have that one or later.

To get version Pico-PIO-USB 0.6.1 into your TinyUSB library, make sure you first
installed PIO USB support for TinyUSB as described in the previous section.
Then type the following instructions at a command prompt:
```
cd ${PICO_SDK_PATH}/lib/tinyusb/hw/mcu/raspberry_pi/Pico-PIO-USB
git pull
git checkout 0.6.1
```

## Get the project code
Clone the midi2piousbhub project to a directory at the same level as the pico-sdk directory.

```
cd ${PICO_SDK_PATH}/..
git clone --recurse-submodules https://github.com/rppicomidi/midi2piousbhub.git
```

## Command Line Build (skip if you want to use Visual Studio Code)

Enter this series of commands (assumes you installed the pico-sdk
and the midid2usbhub project in the ${PICO_MIDI_PROJECTS} directory)

If your system is based on a Pico W board, enter this command first
```
export PICO_BOARD=pico_w
```
For all boards, enter this commands.

```
export PICO_SDK_PATH=${PICO_MIDI_PROJECTS}/pico-sdk/
cd ${PICO_MIDI_PROJECTS}/midi2piousbhub
mkdir build
cd build
cmake ..
make
```
The build should complete with no errors. The build output is in the build directory you created in the steps above.

# Troubleshooting
If your project works for some USB MIDI devices and not others, one
thing to check is the size of buffer to hold USB descriptors and other
data used for USB enumeration. Look in the file `tusb_config.h` for
```
#define CFG_TUH_ENUMERATION_BUFSIZE 512
```
Very complex MIDI devices or USB Audio+MIDI devices like DSP guitar pedals
or MIDI workstation keyboards may have large USB configuration descriptors.
This project assumes 512 bytes is enough, but it may not be for your device.

To check if the descriptor size is the issue, use your development computer to
dump the USB descriptor for your device and then add up the wTotalLength field
values for each configuration in the descriptor.


For Linux and MacOS Homebrew, the command is lsusb -d [vid]:[pid] -v
For Windows, it is simplest to install a program like
[Thesycon USB Descriptor Dumper](https://www.thesycon.de/eng/usb_descriptordumper.shtml).

For example, this is the important information from `lsusb -d 0944:0117 -v`
from a Korg nanoKONTROL2:
```
  bNumConfigurations      1
  Configuration Descriptor:
    bLength                 9
    bDescriptorType         2
    wTotalLength       0x0053
    bNumInterfaces          1
    bConfigurationValue     1
    iConfiguration          0 
    bmAttributes         0x80
      (Bus Powered)
    MaxPower              100mA
```
This is the important information from the Thesycon USB Descriptor Dumper for
a Valeton NUX MG-400
```
0x01	bNumConfigurations

Device Qualifier Descriptor is not available. Error code: 0x0000001F


-------------------------
Configuration Descriptor:
-------------------------
0x09	bLength
0x02	bDescriptorType
0x0158	wTotalLength   (344 bytes)
0x04	bNumInterfaces
0x01	bConfigurationValue
0x00	iConfiguration
0xC0	bmAttributes   (Self-powered Device)
0x00	bMaxPower      (0 mA)
```
You can see that if `CFG_TUH_ENUMERATION_BUFSIZE` were 256 instead of 512,
the Korg nanoKONTROL2 would have no trouble enumerating but the Valeton
NUX MG-400 would fail because TinyUSB couldn't load the whole configuration
descriptor to memory.

# Terms this document uses
- **Connected MIDI Device**: a MIDI device connected to a USB hub port or to a serial
port MIDI DIN connector, or the USB C Device interface or the Bluetooth LE MIDI.
- **USB ID**: A pair of numbers the Connected MIDI Device reports to the
hub when it connects. They are supposed to be unique to a particular
product. The MIDI DIN connectors, the USB C Device interface, and the Bluetooth LE
MIDI interface have fake USB ID numbers to be compatible with this system.
- **Routing Matrix**: The software that sends MIDI data to and from Connected MIDI Devices
- **Terminal**: a MIDI data input to or output from the Routing Matrix.
- **FROM terminal**: an input to the Routing Matrix. It will be a MIDI OUT signal from
a Connected MIDI Device.
- **TO terminal**: an output from the Routing Matrix. It will be a MIDI IN signal to
a Connected MIDI Device.
- **Port**: usually a group of 1 MIDI IN data stream and one MIDI OUT data stream associated with a Connected MIDI Device. A Port of a Connected MIDI Device may omit MIDI IN or MIDI OUT, but not both. Ports are numbered 1-16
- **Direction** of a terminal: either FROM the Connected MIDI Device's MIDI OUT
or TO the Connected MIDI Device's MIDI IN.
- **Nickname**: a more human name than specifying a device port's FROM and TO
data streams using a USB ID, a Port number and a Direction. Nicknames have
a maximum of 12 characters. The default nickname for a port in a given
direction is the USB ID followed by either a "F" for a FROM data stream or
"T" for a TO data stream, followed by the port number (1-16). For example,
"Drumpads" above was renamed from "0000-0000-F1"
- **Product Name**: a name that identifies the the attached MIDI
device. The Connected MIDI Device sends it to the hub on connection; it is a more friendly
name than USB ID, and is the easiest way to assocate the Connected MIDI Device
with all the other info.
- **Dedicated Device**: The DIN MIDI ports, the Bluetooth LE MIDI interface, and USB C Device
interface always have place holders in the system, so you can always route them. These
are therefore dedicated devices.


# Command Line Commands
## help
Show a list of all available commands and brief help text.

## list
List all Connected MIDI Devices currently connected to the USB hub followed
by the built-in devices. For example:

```
USB ID      Port  Direction Nickname    Product Name
0499-1622    1      FROM    lead-out    reface CS
0499-1622    1       TO     lead        reface CS
1C75-02CA    1      FROM    keys        Arturia Keylab Essential 88
1C75-02CA    1       TO     keys-in     Arturia Keylab Essential 88
1C75-02CA    2      FROM    faders      Arturia Keylab Essential 88
1C75-02CA    2       TO     faders-in   Arturia Keylab Essential 88
0000-0000    1      FROM    Drumpads    MIDI IN A
0000-0000    1       TO     TR-707      MIDI OUT A
0000-0001    1      FROM    DAW-OUT     PC MIDI
0000-0001    1       TO     DAW-IN      PC MIDI
0000-0002    1      FROM    iPad-OUT    BT MIDI
0000-0002    1       TO     iPad-IN     BT MIDI
```

## rename \<Old Nickname\> \<New Nickname\>
Rename the nickname for a product's port. All nicknames must be unique. If you need to
hook up more than one device with the same USB ID, then you must do so one at a
time and change the nickname for each port before attaching the next one to the hub.

## connect \<From Nickname\> \<To Nickname\>
Send data from the MIDI Out port of the MIDI device with nickname \<From Nickname\> to the
MIDI IN port of the device with nickname \<To Nickname\>. If more than one device connects
to the TO terminal of a particular device, then the streams are merged.

## disconnect \<From Nickname\> \<To Nickname\>
Break a connection previously made using the `connect` command.

## reset
Disconnect all routings.

## show
Show a connection matrix of all MIDI devices connected to the hub. A blank box means "not
connected" and an `x` in the box means "connected." A '!' in the box means the dedicated
device is routed but is not connected. For example, the following shows
MIDI OUT of the "keys" device connected to the MIDI IN of the "lead" device.
The "faders" device in this example is Bluetooth LE MIDI, and is currently not connected, so
the connection to the "lead" device is shown with an '!'.

```
       TO-> |   |   |   |
            |   |   |   |
            |   |   |   |
            |   |   | f |
            |   |   | a |
            | l | k | d |
            | e | e | e |
FROM |      | a | y | r |
     v      | d | s | s |
            | - | - | - |
            | i | i | i |
            | n | n | n |
------------+---+---+---+
lead        |   |   |   |
------------+---+---+---+
keys        | x |   |   |
------------+---+---+---+
faders      | ! |   |   |
------------+---+---+---+
```

## save \<preset name\>
Save the current setup to the given \<preset name\>. If there is already a preset with that
name, then it will be overwritten. If the preset name is omitted, then the last saved
preset is overwritten.

## load \<preset name\>
Load the current setup from the given \<preset name\>. If the preset was not previously
saved using the save command, then print an error message to the console. If the name
is omitted, then the last saved preset is loaded.

## backup [\<preset name\>]
Copy the specified preset to USB flash drive to a file on the drive named `/rppicomidi-midi2usbhub/<preset name>`. If no preset name is given, then all presets are copied to the
flash drive.

## restore \<preset name\>
Copy the specified preset from the USB flash drive directory `/rppicomidi-midi2usbhub/<preset name>` to the file system on Pico board's program flash.

## format
Reformat the LittleFs file system in the Pico's program memory. It delete all presets.

## fsstat
Print information about the LittleFs file system

## ls [path]
List all files in the LittleFs file system. If you specify a `path`, then list the contents
of the `path` directory. For now, the only directory path is `/`.

## rm \<filename\>
Deletes the file with name \<filename\> in the LitteFs file system

## f-cd [path]
Change current directory of the current USB flash drive to `path`. If `path` is not specified,
equivalent to `f-cd /` (i.e., set to the drive root directory).

## f-chdrive \<drive number 0-3\>
Change current drive number for the USB flash drive. Will only need to do this if you have
more than one flash drive plugged in. When you plug in a drive, the code automatically
sets the drive number to the latest drive.

## f-ls [path]
List contents of the current directory on the current USB flash drive if `path` is not
specified. Otherwise, list the contents of the specified path.

## f-pwd
Print the current directory path of the current USB flash drive.

## set-date \<year(2022-9999)\> \<month(1-12)\> \<day(1-31)\>
Change real-time clock date. The date and time is used for external flash drive file timestamps.

## set-time \<hour(0-23)\> \<minute(0-59)\> \<second(0-59)\>
Change the real-time clock time of day.
The date and time is used for external flash drive file timestamps.

## get-datetime
Print the current date and time as read from the on-chip real-time clock. The time
has a resolution of 2 seconds as because that is what is required for flash drive
file timestamps. The initial date and time will be the last time you built the
msc-rp2040rtc library.

## btmidi-disconnect
Disconnect an active Bluetooth MIDI connection or report "Already disconnected."
You must enter this command before switching between client mode and server mode
if Bluetooth MIDI was previously connected.

## btmidi-list
List all Bluetooth bonded devices stored in the Bluetooth database

## btmidi-rm
Remove the device specified by the index from the Bluetooth bonded database

## btmidi-client-scan-begin
Start a scan for MIDI devices. Switch to client mode if necessary.

## btmidi-client-scan-list
List all found Bluetooth MIDI devices discovered during scan.

## btmidi-client-scan-end
End scan for Bluetooth MIDI devices and list all found

## btmidi-client-connect <0 for last connected or index number from scan list>
Connect to the specfied device index number from the scan list. Index 0 is the
last connected device. TODO: right now this is the first device in the bonded
device list.

## btmidi-client-cancel-connect
Cancel the pending client mode connection request.

## btmidi-client-keep-connected `[0|1]`
In client mode, the "keep connected" flag is 1 if the client should immediately
attempt to reconnect if the currently connected server disconnects. If the
"keep connected" flag is 0, then if the server disconnects, then the client
will not attempt to disconnect. If you issue this command with no arguments,
it will print the current flag state, 0 or 1. This flag is stored as part
of the settings when you issue the save command.

## btmidi-server-start
Leave client mode and enter server mode.

## btmidi-get-state
Prints Client or Server, Connected or Disconnected

