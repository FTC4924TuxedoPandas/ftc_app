package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * Created by 4924_Users on 1/14/2017.
 */

@TeleOp(name = "BeaconSensorReadingTest")
public class BeaconSensorReadingTest extends RevolutionVelocityBase {

    @Override
    public void init() {

        rightBeaconSensor = hardwareMap.colorSensor.get("rightBeaconSensor");
        leftBeaconSensor = hardwareMap.colorSensor.get("leftBeaconSensor");
        rightBeaconSensor.enableLed(true);
        leftBeaconSensor.enableLed(false);
    }

    @Override
    public void init_loop() {}

    @Override
    public void start() {}

    @Override
    public void loop() {

        telemetry.addData("Right", rightBeaconSensor.red());
        telemetry.addData("Left", leftBeaconSensor.red());
    }
}
