package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchDevice;
import com.qualcomm.robotcore.hardware.configuration.annotations.DeviceProperties;
import com.qualcomm.robotcore.hardware.configuration.annotations.I2cDeviceType;
import com.qualcomm.robotcore.util.TypeConversion;

@SuppressWarnings({"WeakerAccess", "unused"}) // Ignore access and unused warnings

@I2cDeviceType
@DeviceProperties(name = "Qwiic LED", description = "SparkFun Qwiic LED Stick for APA102C LEDs", xmlTag = "QwiicLED")

public class QwiicLED extends I2cDeviceSynchDevice<I2cDeviceSynch> {
    
    public short changeLEDLength(short length) {
        writeShort(Register.CHANGE_LED_LENGTH,length);
        return 0;
    }
    
    public short setSingleLEDColor(byte id, byte red, byte green, byte blue) {
        writeByteArray(Register.WRITE_SINGLE_LED_COLOR,new byte[]{id,red,green,blue});
        return 0;
    }
    
    public short setLEDColor(byte red, byte green, byte blue) {
        writeByteArray(Register.WRITE_ALL_LED_COLOR,new byte[]{red,green,blue});
        return 0;
    }
    
    public short setLEDBrightness(byte id, byte brightness) {
        writeByteArray(Register.WRITE_SINGLE_LED_BRIGHTNESS, new byte[]{id, brightness});
        return 0;
    }
    
    protected void writeShort(final Register reg, short value)
    {
        deviceClient.write(reg.bVal, TypeConversion.shortToByteArray(value));
    }
    
    protected void writeByteArray(final Register reg, byte[] value)
    {
        deviceClient.write(reg.bVal, value);
    }
    
    protected short readShort(Register reg)
    {
        return TypeConversion.byteArrayToShort(deviceClient.read(reg.bVal, 2));
    }
    
    ///////////////////////////// Config ///////////////////////////////
    
    public final static I2cAddr ADDRESS_I2C_DEFAULT = I2cAddr.create7bit(0x23);
    
    public QwiicLED(I2cDeviceSynch deviceClient) {
        
        super(deviceClient, true);
    
        this.setOptimalReadWindow();
        this.deviceClient.setI2cAddress(ADDRESS_I2C_DEFAULT);
        
        super.registerArmingStateCallback(false);
        this.deviceClient.engage();
    }
    
    protected void setOptimalReadWindow()
    {
        // Sensor registers are read repeatedly and stored in a register. This method specifies the
        // registers and repeat read mode
        I2cDeviceSynch.ReadWindow readWindow = new I2cDeviceSynch.ReadWindow(
                Register.FIRST.bVal,
                Register.LAST.bVal - Register.FIRST.bVal + 1,
                I2cDeviceSynch.ReadMode.REPEAT);
        this.deviceClient.setReadWindow(readWindow);
    }
    
    
    @Override
    protected synchronized boolean doInitialize() {
        return true;
    }
    
    @Override
    public Manufacturer getManufacturer() {
        return Manufacturer.Other;
    }
    
    @Override
    public String getDeviceName() {
        return "SparkFun Qwiic LED Stick - APA102C";
    }
    
    public enum Register {
        FIRST(0x6F),
        CHANGE_LED_LENGTH (0x70),
        WRITE_SINGLE_LED_COLOR (0x71),
        WRITE_ALL_LED_COLOR (0x72),
        WRITE_RED_ARRAY (0x73),
        WRITE_GREEN_ARRAY (0x74),
        WRITE_BLUE_ARRAY (0x75),
        WRITE_SINGLE_LED_BRIGHTNESS (0x76),
        WRITE_ALL_LED_BRIGHTNESS (0x77),
        WRITE_ALL_LED_OFF (0x78),
        LAST(WRITE_ALL_LED_OFF.bVal),
        CHANGE_ADDRESS (0xC7);
        
        public int bVal;
        
        Register(int bVal) {
            
            this.bVal = bVal;
        }
    }
}

