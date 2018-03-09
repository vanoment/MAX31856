# MAX31856
This is a class for reading temperature from a MAX31856 sensor via a Raspberry Pi 3 Model B. The purpose of this is simply to demonstrate how to read temperatures from a MAX31856 sensor.

I referenced the MAX31856 data sheet (rev 0) https://datasheets.maximintegrated.com/en/ds/MAX31856.pdf and johnrbnsn's MAX31856 python library https://github.com/johnrbnsn/Adafruit_Python_MAX31856, as well as various Adafruit forum posts.

This class takes in a chip select pin because the Raspberry Pi 3 Model B has a limit of two devices for the SPI bus (CE0 and CE1), and we would like to use more than two sensors. The GPIO portions can be taken out if a maximum of two MAX31856 sensors are being read from the Raspberry Pi.

This class does not make use of the FAULT pin or the DRDY pin.

This was written in C# because we were using the Windows 10 IoT Core to connect to our Raspberry Pi and C# was what was being used the most.
