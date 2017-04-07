package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.I2cAddr;

/**
 * Created by 4924_Users on 4/1/2017.
 */

@TeleOp(name = "TwoColorSensors")
public class TwoColorSensors extends OpMode {

    ColorSensor leftBeaconSensor;
    ColorSensor rightBeaconSensor;

    public void init() {
        leftBeaconSensor = hardwareMap.colorSensor.get("leftBeaconSensor");
        leftBeaconSensor.setI2cAddress(I2cAddr.create7bit(0x1e));
        rightBeaconSensor = hardwareMap.colorSensor.get("rightBeaconSensor");
        rightBeaconSensor.setI2cAddress(I2cAddr.create7bit(0x24));
    }

    public void loop() {
        telemetry.addData("Left Side Reads: ", leftBeaconSensor.alpha());
        telemetry.addData("Right Side Reads: ", rightBeaconSensor.alpha());
    }
}
