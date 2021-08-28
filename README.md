# Toyota_Spd_SAS_Tester
This project is for a Toyota Steering Angle Sensor (SAS) tester. The tester also includes the capability to calculate speed based on a VSS signal.

I created this tester in order to confirm the precision of several SAS units I had acquired in the hopes of adding [openpilot]( https://github.com/commaai/openpilot) to my [old car]( https://github.com/Lukilink/openpilot) . I was confused as to why some people stated that the resolution of the TSS1 SAS was only 1.5 degrees while others stated that the sensors were good to 0.1 degrees. 

## The hardware components are: 
* STM32F1 Bluepill (using STM32duino core)
* MCP2515 CAN Bus Module (SPI)
* 0.96 Inch OLED 128x64 Module (I2C)
* 2 SPST switches
* 82K and 2.2K resistors

The switches select between mode of operation (Angle or Speed) and SAS precision (High and Low).

Low and High precision refer to either 1.5 degree or 0.1 degree resolution from the SAS.

## Bluepill pinout
![bluepin pinout](https://github.com/ZuluSpl0it/Toyota_Spd_SAS_Tester/blob/2a8fb83629f6e887d10cb9ac5db400b4c211e8c6/images/bluepill_pinout.jpg)

## Here are the results of my testing:

Part# | Precision
------|----------
89245-02080 | High (0.1 deg resolution)
89245-0R020 | Low (1.5 deg resolution)
89245-07020 | Low (1.5 deg resolution)

## CAN message detail (for high precision sensor)

The High precision SAS sends a CAN message in the following format:
````
 BO_ 37 STEER_ANGLE_SENSOR: 8 XXX  
    SG_ STEER_ANGLE : 3|12@0- (1.5,0) [-500|500] "deg" XXX  
    SG_ STEER_FRACTION : 39|4@0- (0.1,0) [-0.7|0.7] "deg" XXX  
    SG_ STEER_RATE : 35|12@0- (1,0) [-2000|2000] "deg/s" XXX  
 ````

Visually, it looks like this:
````
    Name:       STEER_ANGLE_SENSOR
    Id:         HEX = 0x25        Decimal = 37
    Length:     8 bytes
    Cycle time: - ms
    Senders:    XXX
    Layout:
                          Bit

             7   6   5   4   3   2   1   0
           +---+---+---+---+---+---+---+---+
         0 |   |   |   |   |<--------------|
           +---+---+---+---+---+---+---+---+
         1 |------------------------------x|
           +---+---+---+---+---+---+---+---+
                                         └-- STEER_ANGLE   [Min = -500   Max = 500]   (Units = deg)
           +---+---+---+---+---+---+---+---+
         2 |   |   |   |   |   |   |   |   |
           +---+---+---+---+---+---+---+---+
     B   3 |   |   |   |   |   |   |   |   |
     y     +---+---+---+---+---+---+---+---+
     t   4 |<-------------x|<--------------|
     e     +---+---+---+---+---+---+---+---+
                         └-- STEER_FRACTION   [Min = -0.7   Max = 0.7]   (Units = deg)
           +---+---+---+---+---+---+---+---+
         5 |------------------------------x|
           +---+---+---+---+---+---+---+---+
                                         └-- STEER_RATE   [Min = -2000   Max = 2000]   (Units = deg/s)
           +---+---+---+---+---+---+---+---+
         6 |   |   |   |   |   |   |   |   |
           +---+---+---+---+---+---+---+---+
         7 |   |   |   |   |   |   |   |   |
           +---+---+---+---+---+---+---+---+

    Signal tree:

    -- {root}
       └-- STEER_ANGLE
       └-- STEER_FRACTION              Note: 1/15th of the signal STEER_ANGLE, which
                                       is 1.5 deg; note that 0x8 is never set
       └-- STEER_RATE                  Note: factor is tbd
````

Then following table is a parital listing of the final calculated Steer Angle:

![Steer Angle Calculation Table](https://github.com/ZuluSpl0it/Toyota_Spd_SAS_Tester/blob/2a8fb83629f6e887d10cb9ac5db400b4c211e8c6/images/Value_Table.jpg)

The 89245-02080 sensor passes both high and low tests:
![High Test](https://github.com/ZuluSpl0it/Toyota_Spd_SAS_Tester/blob/43abf7e65f18e23d9a98ca571faac3835bc93f86/images/angle_high_good.jpg)
![Low Test](https://github.com/ZuluSpl0it/Toyota_Spd_SAS_Tester/blob/43abf7e65f18e23d9a98ca571faac3835bc93f86/images/angle_low_good2.jpg)

However, the other sensors fail the high precision test, while passing the low precision test:
![High Test](https://github.com/ZuluSpl0it/Toyota_Spd_SAS_Tester/blob/43abf7e65f18e23d9a98ca571faac3835bc93f86/images/angle_high_bad2.jpg)
![Low Test](https://github.com/ZuluSpl0it/Toyota_Spd_SAS_Tester/blob/43abf7e65f18e23d9a98ca571faac3835bc93f86/images/angle_low_good.jpg)
