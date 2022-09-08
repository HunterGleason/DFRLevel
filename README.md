# DESCRIPTION
This repository contains Arduino code for operating a DFRobot [DFRobot sensor](https://wiki.dfrobot.com/Throw-in_Type_Liquid_Level_Transmitter_SKU_KIT0139) 
and datalogger intended for use in hydrologic studies. The code was developed and tested on the 
[Adafruit Adalogger M0](https://www.adafruit.com/product/2796) based on the SAMD21 processor but likely will work on similar boards. This code contains functions for sending daily 
Iridium satellite communications using the [RockBlock 9603](https://www.iridium.com/products/rock-seven-rockblock-9603/) satellite modem. The code requires several 
parameters be specified using a parameter file, this is explained below. In addition, a wiring diagram of the required circuit is provided. 
Because this script uses the RockBlock Iridium modem it is important to make sure that a line rental with Ground Control is active and the modem is registered with credits available.  

# DEPENDENCIES 
There are several libraries required for running this code in addition to having the Arduino IDE installed and [configured](https://learn.adafruit.com/adafruit-feather-m0-adalogger/setup)
to work with the Adalogger M0, and are listed below.

- <OneWire.h> //Needed for oneWire communication 
- <RTClib.h> //Needed for communication with Real Time Clock
- <SPI.h>//Needed for working with SD card
- <SD.h>//Needed for working with SD card
- <ArduinoLowPower.h>//Needed for putting Feather M0 to sleep between samples
- <IridiumSBD.h>//Needed for communication with IRIDIUM modem 
- <CSV_Parser.h>//Needed for parsing CSV data

# EQUIPMENT LIST
The following list is the hardware required for creating the circuit shown in the diagram and does not include items for field deployment such as logger box, grommets, standoffs,
conduit etc.


| Item | Description | Link | Qnt. |
| -------- | -------- | -------- | -------- |
| Feather Adalogger M0 | MCU for data acquisition | [Adalogger](https://www.adafruit.com/product/2796)[^2] | 1 |
| FeatherWing Terminal Breakout | For interfacing with the Adalogger | [Terminal Breakout](https://www.adafruit.com/product/2926)[^2] | 1 |
| PCF8523 | Real Time Clock | [PCF8523](https://www.adafruit.com/product/3295) | 1 |
| DFRobot Throw In Level Logger | Liquid Level Pressure Transducer | [DFR Liquid Level Sensor](https://wiki.dfrobot.com/Throw-in_Type_Liquid_Level_Transmitter_SKU_KIT0139) | 1 |
| Latching Relay | For switching power to HYDROS21 | [Relay](https://www.adafruit.com/product/2923)[^2] | 1 |
| DS18B20 | Waterproof 1-Wire temp. sensor | [DS18B20](https://www.adafruit.com/product/381) | 1 |
| Cabel | Cabel for extending solar panel and / or  DS18B20 if needed | NA[^1] | NA |
| bq24074 | Solar LiPo charger | [bq24074](https://www.adafruit.com/product/4755) | 1 | 
| Solar Panel | 6V 3.5 Watt | [Solar Panel](https://www.adafruit.com/product/500) | 1 |
| LiPo Battery | 3.7V 6600 mAh | [Lipo Battery](https://www.adafruit.com/product/353)[^2] | 1 |
| RockBLOCK 9603 | IRIDIUM Modem | [RockBLOCK](https://www.iridium.com/products/rock-seven-rockblock-9603/)[^2] | 1 |
| Patch Antenna | For improved signal | [Antenna](https://www.sparkfun.com/products/14580)[^1] | 1 |
| 2N222 | NPN Transistor for switching power to RockBLOCK | [2N2222](https://www.adafruit.com/product/756)[^2] | 1 | 
| Connecting Wire | Wire for making connections, or PCB | NA | NA |
| Screw Terminals | 2.54 mm Pitch Terminal Block - 2-Pin | [Terminal Block](https://www.adafruit.com/product/2138)[^1] | ~5 |

[^1]: Optional 
[^2]: Or Functional Equivalent


# LOGGER OPERATION
This section outlines the steps for operating the logger.

## Set up a parameter file
The first step to using the logger is to obtain a micro-SD card and add to it a parameter file the logger will use to define several important variables. This file must be 
named "PARAM.txt" (case sensitive),comma delimited, and should look like the example parameter file shown below, !column order is important for parsing of file!:

```
filename,sample_intvl,irid_freq,start_time,sample_n
MYFILE.CSV,10,12,00:00:00,50
```

Where the "filename" header declares the name of the log file to be written (!!MUST BE LESS THEN 8 CHARACTERS IN LENGTH!!) and is user defined, the "sampl_intvl" header stands 
for sample interval and declares the sampling rate of the logger in seconds. Note that every sample will be written to the SD card but only hourly averages will be sent over 
the IRIDIUM modem daily. The sampling rate should be <60 minutes as the code sends hourly averages. The "irid_freq" parameter is the frequncy at which the hourly values get 
sent over IRIDIUM in hours, so a value of 12 would result in IRIDIUM transmission every 12 hours with 12 hours worth of data. The "start_time" parameter indicates the start 
time for the first Iridium transmission as HH:MM:SS, so 13:00:00 would result in a transmission at 01:00:00 PM, the next would be at "start_time + irid_freq". The final parameter 
"sample_n" is a integer that sets the number of samples to take per reading, of which the median value is returned as the value to be logged to the SD card. Typically a value of 
50 returns good results. 

## Set the time on the RTC
To keep track of time this code relies on a Real Time Clock, specifically the PCF8523. To set the time run the [pcf8523 script](https://learn.adafruit.com/adafruit-pcf8523-real-time-clock/rtc-with-arduino) 
from the "RTClib" library and upload it to the MCU BEFORE uploading the logging script. We recommend setting the RTC to UTC time to avoid complications with daylight savings time. 

## Launch the logger
Once the desired time has been set on the RTC, the logging script "HYDROS21.ino" can be uploaded. This script will sample the HYDROS21 at the specified sampling interval and record 
each sample to the Micro-SD card along with the sample datetime. In addition, the logger will transmit the most recent hourly averages for the over the IRIDIUM modem at the speciified 
interval. 

1. To launch the logger, first remove the Micros-SD card, plug the Adalogger into a computer over USB, make sure the power switch is set to "ON" on the Feather Terminal block, 
and upload the code. Upon completing the upload the MCU should flash the built in LED at 1-sec intervals indicating the Micro-SD card cannot be initialized, this is ok as we 
removed it on purpose, as it prevents the loggger from entering sleep mode, during which code can only be uploaded by double-clicking the reset button during upload.
2. Unplug the MCU from the computer and toggle the power switch to "OFF" on the Feather Terminal Block.
3. Plug in the Micro-SD card (!!with configured "PARAM.txt" parameter file present!!) and plug in the 5-Pin to USB adapter (or JST jumper) into the correct Adalogger port.
4. Toggle the power switch to "ON" on the Feather Terminal Block, a reading should be taken immediately, indicated by short flash of built in LED, and the latching relay should be audible,
if the 'start_time' parameter was set to value in the past, the logger will proceed to send a message right away, which will be without data as no data exist upon initial launch.  

After sampling the Adalogger is put into low power mode until the next sample. !!During this time, it will be impossible to upload code to the MCU without double clicking the reset 
button during upload, or removing the SD card and cycling the power to the MCU!! If for whatever reason after turning on the MCU the built in LED continues to flash at a given interval, 
consult the error codes section below to help identify the issue. Note that upon removing the Micro-SD card after logging there should be three files, the "PARAM.txt" file provided at 
launch, a log file with the name specified in "PARAM.txt", and one called "HOURLY.CSV"; this file is used for Iridium transmissions and can be ignored, if deleted hourly data for the 
most recent observations will be deleted, but subsequent transmissions will be complete. 


## Error Codes
Errors are conveyed via the built in LED (Pin 13) on the Adalogger as repeated blinking at specific time intervals, these intervals and their meaning are listed below:

- Repeated 2-sec: Problem initializing SD card, check SD is inserted, [formatted](https://www.arduino.cc/reference/en/libraries/sd/) correctly. 
- Repeated 1-sec: Problem parsing "PARAM.txt" parameter file, make sure it is present on SD card, named correctly, and formatted as specified above. 
- Repeated 0.5-sec: Problem initializing RTC, check that wiring is correct, and coin cell is inserted and at appropriate voltage. 
- Two consecutive 1-sec blinks during Iridium transmission: Modem failed to begin, check wiring, logging should continue.  


- Note that the built in LED will be held HIGH during Iridium transmission to indicate that it is trying to send the message. 
- During normal operation while the MCU is sleeping the built in LED will be OFF, this is OK, the logger is still operating, however code cannot be uploaded without double clicking 
the reset button during upload.
- A 0.25-sec LED flash at the end of each sample will occur for each sample to indicate a sample was just taken, this is normal, and the latching relay should be heard during each 
sample as well.  
