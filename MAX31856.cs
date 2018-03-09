using System;
using System.Linq;
using Windows.Devices.Gpio;
using Windows.Devices.Spi;

namespace SensorDataLogger.Contracts
{
    /* I referenced the MAX31856 data sheet (rev 0) https://datasheets.maximintegrated.com/en/ds/MAX31856.pdf and johnrbnsn's MAX31856 python library https://github.com/johnrbnsn/Adafruit_Python_MAX31856
     * This class takes in a chip select pin because the Raspberry Pi 3 Model B has a limit of two devices for the SPI bus (CE0 and CE1), and we would like to use more than two sensors.
     * If only using one or two sensors, all GPIO portions can be taken out given that the SpiDevice is correctly configured
     */
    public class MAX31856 : Sensor
    {
        public enum ThermocoupleType
        {
            /// <summary>
            /// B Type Thermocouple
            /// </summary>
            B = 0,

            /// <summary>
            /// E Type Thermocouple
            /// </summary>
            E = 1,

            /// <summary>
            /// J Type Thermocouple
            /// </summary>
            J = 2,

            /// <summary>
            /// K Type Thermocouple
            /// </summary>
            K = 3,

            /// <summary>
            /// N Type Thermocouple
            /// </summary>
            N = 4,

            /// <summary>
            /// R Type Thermocouple
            /// </summary>
            R = 5,

            /// <summary>
            /// S Type Thermocouple
            /// </summary>
            S = 6,

            /// <summary>
            /// T Type Thermocouple
            /// </summary>
            T = 7
        }

        // Write addresses
        private byte MAX31856_REG_WRITE_CR0 = 0x80; // CR0 configuration
        private byte MAX31856_REG_WRITE_CR1 = 0x81; // CR1 configuration
        private byte MAX31856_REG_WRITE_MASK = 0x82; // Fault Mask Register
        private byte MAX31856_REG_WRITE_CJHF = 0x83; // Cold Junction High Fault Threshold
        private byte MAX31856_REG_WRITE_CJLF = 0x84; // Cold Junction Low Fault Threshold
        private byte MAX31856_REG_WRITE_LTHFTH = 0x85; // Linearized Temperature High Fault Threshold MSB
        private byte MAX31856_REG_WRITE_LTHFTL = 0x86; // Linearized Temperature High Fault Threshold LSB
        private byte MAX31856_REG_WRITE_LTLFTH = 0x87; // Linearized Temperature Low Fault Threshold MSB
        private byte MAX31856_REG_WRITE_LTLFTL = 0x88; // Linearized Temperature Low Fault Threshold LSB
        private byte MAX31856_REG_WRITE_CJTO = 0x89; // Cold-Junction Temperature Offset Register

        // Read addresses
        private byte MAX31856_REG_READ_CR0 = 0x00; // CR0 configuration
        private byte MAX31856_REG_READ_CR1 = 0x01; // CR1 configuration
        private byte MAX31856_REG_READ_MASK = 0x02; // Fault Mask Register
        private byte MAX31856_REG_READ_CJHF = 0x03; // Cold Junction High Fault Threshold
        private byte MAX31856_REG_READ_CJLF = 0x04; // Cold Junction Low Fault Threshold
        private byte MAX31856_REG_READ_LTHFTH = 0x05; // Linearized Temperature High Fault Threshold MSB
        private byte MAX31856_REG_READ_LTHFTL = 0x06; // Linearized Temperature High Fault Threshold LSB
        private byte MAX31856_REG_READ_LTLFTH = 0x07; // Linearized Temperature Low Fault Threshold MSB
        private byte MAX31856_REG_READ_LTLFTL = 0x08; // Linearized Temperature Low Fault Threshold LSB
        private byte MAX31856_REG_READ_CJTO = 0x09; // Cold-Junction Temperature Offset Register
        private byte MAX31856_REG_READ_CJTH = 0x0A; // Cold-Junction Temperature Register, MSB
        private byte MAX31856_REG_READ_CJTL = 0x0B; // Cold-Junction Temperature Register, LSB
        private byte MAX31856_REG_READ_LTCBH = 0x0C; // Linearized TC Temperature, Byte 2
        private byte MAX31856_REG_READ_LTCBM = 0x0D; // Linearized TC Temperature, Byte 1
        private byte MAX31856_REG_READ_LTCBL = 0x0E; // Linearized TC Temperature, Byte 0
        private byte MAX31856_REG_READ_FAULT = 0x0F; // Fault status register

        // Constants
        private double MAX31856_CONST_THERM_LSB = 0.0078125; //  0.0078125 degrees C resolution for linear temperature
        private int MAX31856_CONST_THERM_BITS = 19; // 19 bit resolution for linear temperature
        private double MAX31856_CONST_CJ_LSB = 0.015625; // 0.015625 degrees C resolution for cold junction temperature
        private int MAX31856_CONST_CJ_BITS = 14; // 14 bit resolution for cold junction temperature

        // The maximum clock frequency for the MAX31856
        public new static int clockFrequency = 5000000;

        /// <summary>
        /// Setup for the MAX31856, which sets the chip select pin and the configurations/defaults
        /// </summary>
        /// <param name="thermocoupleType">Type of thermocouple based on ThermocoupleType enum</param>
        /// <param name="averageSamples">Representation of the number of samples to get an average from, range from 0 to 4, which respectively corresponds to 1, 2, 4, 8, or 16 samples (1*(2^averageSamples))</param>
        /// <param name="gpioController">GpioController specified for this device</param>
        /// <param name="spiDisplay">SpiDevice specified for this device</param>
        /// <param name="chipSelectPin">Chip select pin number specified by the device</param>
        public MAX31856(ref GpioController _gpioController, ref SpiDevice _spiDisplay, int chipSelectPin, ThermocoupleType thermocoupleType = ThermocoupleType.K, int averageSamples = 0)
        {
            // Set up device's gpio controller and spi device and chip select pin
            this.Setup(ref _gpioController, ref _spiDisplay, chipSelectPin);

            /*** SET UP THE CONFIGURATION AND ALL DEFAULTS FOR THE MAX31856 ***/

            // Set the Cold-Junction Temperature Offset to 0 first (important to do before setting up Configuration 0)
            WriteRegister(MAX31856_REG_WRITE_CJTO, 0x0);

            // Set Configuration 0 (conversion mode, one-shot mode fault configuration)
            WriteRegister(MAX31856_REG_WRITE_CR0, 0x82); // 0x80 is for continuous reading, but 0x40 is for one shot reading

            // Set Configuration 1 (Type of thermocouple, how many samples to average from)
            // Do some checks on averageSamples: default to 0, make sure it's not more than 4 or less than 0
            averageSamples = (averageSamples < 0 || averageSamples > 4) ? 0 : averageSamples;
            WriteRegister(MAX31856_REG_WRITE_CR1, (byte)((averageSamples << 4) + (int)thermocoupleType));

            // Set Fault Mask Register
            WriteRegister(MAX31856_REG_WRITE_MASK, 0xFF);

            // Set Cold-Junction Temperature Thresholds (defaults for now)
            WriteRegister(MAX31856_REG_WRITE_CJHF, 0x7F);
            WriteRegister(MAX31856_REG_WRITE_CJLF, 0xC0);

            // Set Linear Temperature Thresholds (defaults for now)
            WriteRegister(MAX31856_REG_WRITE_LTHFTH, 0x7F);
            WriteRegister(MAX31856_REG_WRITE_LTHFTL, 0xFF);
            WriteRegister(MAX31856_REG_WRITE_LTLFTH, 0x80);
            WriteRegister(MAX31856_REG_WRITE_LTLFTL, 0x00);

            // Just for testing to see if it accurately sets values to the register
            var cr0Config = ReadRegister(MAX31856_REG_READ_CR0);
            var cr1Config = ReadRegister(MAX31856_REG_READ_CR1);
            var faultMask = ReadRegister(MAX31856_REG_READ_MASK);
            var coldJunctionThresholds = ReadRegister(MAX31856_REG_READ_CJHF, 2); // Gets MAX31856_REG_READ_CJHF and MAX31856_REG_READ_CJLF
            var linearTemperatureThresholds = ReadRegister(MAX31856_REG_READ_LTHFTH, 4); // Grabs MAX31856_REG_READ_LTHFTH, MAX31856_REG_READ_LTHFTL, MAX31856_REG_READ_LTLFTH, and MAX31856_REG_READ_LTLFTL
            var coldJunctionTemperatureOffset = ReadRegister(MAX31856_REG_READ_CJTO);
            //Verify the defaults
            var taco = cr0Config;
        }

        /// <summary>
        /// Reads the temperature from the MAX31856
        /// </summary>
        /// <returns>Temperature (in celsius)</returns>
        public override double GetData()
        {
            //The temperature is read from three different registers, and the fourth is the fault register
            var bytes = ReadRegister(MAX31856_REG_READ_LTCBH, 4);

            //Convert the bytes into double
            // The first byte (bytes[0]) contains the sign of the temperature (so we want to remove that portion, hence the & 0x7F)
            var val_bytes = ((bytes[0] & 0x7F) << 16) + (bytes[1] << 8) + bytes[2];
            // The third byte (bytes[2]) contains 5 dead bits at the end
            val_bytes = val_bytes >> 5;

            //Check if positive or negative
            if ((bytes[0] & 0x80) == 1)
            {
                val_bytes -= 2 ^ (MAX31856_CONST_THERM_BITS - 1);
            }
            var temperature = val_bytes * MAX31856_CONST_THERM_LSB;
            try
            {
                GetFaults(bytes[3]);
            }
            catch (Exception ex)
            {
                throw new Exception(ex.Message + ", " + temperature.ToString());
            }
            return temperature;
        }

        /// <summary>
        /// Gets the cold junction temperature
        /// </summary>
        /// <returns>Cold Junction Temperature (in celsius)</returns>
        public double GetColdJunctionTemperature()
        {
            // The internal temperature is read from two different registers (MAX31856_REG_READ_CJTH and MAX31856_REG_READ_CJTL)
            var bytes = ReadRegister(MAX31856_REG_READ_CJTH, 2);

            // Convert the bytes into double
            // MSB (bytes[0]) contains the sign of the temperature (so we want to remove that portion, hence the & 0x7F)
            var val_bytes = ((bytes[0] & 0x7F) << 8) + bytes[1];
            // LSB (bytes[1]) contains 2 dead bits at the end
            val_bytes = val_bytes >> 2;

            // Check if positive or negative
            if ((bytes[0] & 0x80) == 1)
            {
                val_bytes -= 2 ^ (MAX31856_CONST_CJ_BITS - 1);
            }
            var temperature = val_bytes * MAX31856_CONST_CJ_LSB;
            try
            {
                GetFaults();
            }
            catch (Exception ex)
            {
                throw new Exception(ex.Message + ", " + temperature.ToString());
            }
            return temperature;
        }

        /// <summary>
        /// Determines the fault given the fault byte (if any exists)
        /// </summary>
        /// <param name="fault">The fault byte to be interpreted</param>
        public void GetFaults(byte? fault = null)
        {
            // If no byte was specified, then get the fault byte
            if (fault == null)
            {
                fault = ReadRegister(MAX31856_REG_READ_FAULT)[0];
            }
            if ((fault & 0x80) != 0)
            {
                throw new Exception("Cold Junction Out-of-Range");
            }
            if ((fault & 0x40) != 0)
            {
                throw new Exception("Thermocouple Out-of-Range");
            }
            if ((fault & 0x20) != 0)
            {
                throw new Exception("Cold Junction High Fault");
            }
            if ((fault & 0x10) != 0)
            {
                throw new Exception("Cold Junction Low Fault");
            }
            if ((fault & 0x08) != 0)
            {
                throw new Exception("Thermocouple Temperature High Fault");
            }
            if ((fault & 0x04) != 0)
            {
                throw new Exception("Thermocouple Temperature Low Fault");
            }
            if ((fault & 0x02) != 0)
            {
                throw new Exception("Overvoltage or Undervoltage Input Fault");
            }
            if ((fault & 0x01) != 0)
            {
                throw new Exception("Thermocouple Open-Circuit Fault");
            }
        }

        /// <summary>
        /// Reads in a specified number of bytes starting at the specified address
        /// </summary>
        /// <param name="address">Address to read from</param>
        /// <param name="registersToRead">Number of registers to read</param>
        /// <returns></returns>
        private byte[] ReadRegister(byte address, int registersToRead = 1)
        {
            if (registersToRead < 1)
            {
                throw new Exception("At least one byte must be specified to read in");
            }
            var readBuffer = new byte[1 + registersToRead]; // registersToRead bytes for reading in, one byte of padding
            var regAddressBuffer = new byte[1 + registersToRead]; // One byte for address, registersToRead bytes of padding
            regAddressBuffer[0] = address; // Set the address to the first byte
            CHIP_SELECT.Write(GpioPinValue.Low);
            spiDisplay.TransferFullDuplex(regAddressBuffer, readBuffer);
            CHIP_SELECT.Write(GpioPinValue.High);
            return readBuffer.Skip(1).ToArray(); // The first has a dead bit that was transferred while reading in the address to read from, so chop that off
        }

        /// <summary>
        /// Writes one byte to the specified register's address
        /// </summary>
        /// <param name="address">Address to write to</param>
        /// <param name="value">Value to write to the register</param>
        private void WriteRegister(byte address, byte value)
        {
            CHIP_SELECT.Write(GpioPinValue.Low);
            spiDisplay.Write(new byte[] { address, value });
            CHIP_SELECT.Write(GpioPinValue.High);
        }
    }
}
