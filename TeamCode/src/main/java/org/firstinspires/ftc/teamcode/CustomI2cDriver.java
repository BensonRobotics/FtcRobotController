package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchDevice;
import com.qualcomm.robotcore.hardware.configuration.annotations.DeviceProperties;
import com.qualcomm.robotcore.hardware.configuration.annotations.I2cDeviceType;

@I2cDeviceType
@DeviceProperties(name = "Arduino UNO I2C", xmlTag = "ARDUINO")
public class CustomI2cDriver extends I2cDeviceSynchDevice<I2cDeviceSynch>
{
    public static final int PIN_STATE_INPUT_REGISTER = 0x00;
    public static final int PIN_STATE_OUTPUT_REGISTER = 0x01;

    public CustomI2cDriver(I2cDeviceSynch i2cDeviceSynch, boolean deviceClientIsOwned)
    {
        super(i2cDeviceSynch, deviceClientIsOwned);

        // Set the I2C address to 0x40 (as used by the Arduino)
        this.deviceClient.setI2cAddress(I2cAddr.create7bit(0x00));

        super.registerArmingStateCallback(false);
        this.deviceClient.engage();
    }

    @Override
    public Manufacturer getManufacturer()
    {
        return Manufacturer.Other;
    }

    @Override
    protected synchronized boolean doInitialize()
    {
        return true;
    }

    @Override
    public String getDeviceName()
    {
        return "Arduino UNO I2C";
    }
}
