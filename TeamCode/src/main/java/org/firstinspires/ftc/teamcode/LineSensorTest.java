package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;

/**
 * Created by 4924_Users on 1/5/2017.
 */
@TeleOp(name = "LineSensorTest")
public class LineSensorTest extends OpMode {

    OpticalDistanceSensor LineSensor;

    @Override
    public void init() {

        LineSensor = hardwareMap.opticalDistanceSensor.get("lineSensor");
        LineSensor.enableLed(true);
    }

    @Override
    public void loop() {

        telemetry.addData("Reading", LineSensor.getRawLightDetected());
    }
}
