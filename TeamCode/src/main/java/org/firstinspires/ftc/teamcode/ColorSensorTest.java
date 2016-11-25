package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;

/**
 * Created by 4924_Users on 11/19/2016.
 */
@TeleOp(name = "ColorSensorTest")
public class ColorSensorTest extends OpMode {

    ColorSensor rightBeaconSensor;
    ColorSensor leftBeaconSensor;

    @Override
    public void init() {

        rightBeaconSensor = hardwareMap.colorSensor.get("rightBeaconSensor");
        leftBeaconSensor = hardwareMap.colorSensor.get("leftBeaconSensor");
        rightBeaconSensor.enableLed(false);
        leftBeaconSensor.enableLed(false);
    }

    @Override
    public void loop() {

        telemetry.addData("Right Red", rightBeaconSensor.red());
        telemetry.addData("Right Blue", rightBeaconSensor.blue());
        telemetry.addData("Left Red", leftBeaconSensor.red());
        telemetry.addData("Left Blue", leftBeaconSensor.blue());
    }
}
