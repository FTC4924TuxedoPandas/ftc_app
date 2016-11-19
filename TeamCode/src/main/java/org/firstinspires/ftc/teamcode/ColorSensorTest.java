package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;

/**
 * Created by 4924_Users on 11/19/2016.
 */
@TeleOp(name = "ColorSensorTest")
public class ColorSensorTest extends OpMode {

    ColorSensor colorSensor;

    @Override
    public void init() {

        colorSensor = hardwareMap.colorSensor.get("colorSensor");
    }

    @Override
    public void loop() {

        telemetry.addData("Red", colorSensor.red());
        telemetry.addData("Blue", colorSensor.blue());
        telemetry.addData("Green", colorSensor.green());
    }
}
