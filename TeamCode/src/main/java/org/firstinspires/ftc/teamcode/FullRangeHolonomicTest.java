package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.configuration.MatrixConstants;

/**
 * Created by 4924_Users on 12/17/2016.
 */

@TeleOp(name = "FullRangeHolonomicTest")
public class FullRangeHolonomicTest extends VelocityBase {

    public boolean headingSet = false;

    @Override
    public void loop() {

        int currentHeading = turningGyro.getHeading();

        float x = -gamepad1.left_stick_x;
        float y = -gamepad1.left_stick_y;

        if (Math.abs(x) > 0.01 || Math.abs(y) > 0.01) {

            headingSet = true;
        }

        if (Math.abs(x) < 0.01 && Math.abs(y) < 0.01) {

            headingSet = false;
            steadyHeading = currentHeading;
        }

        telemetry.addData("x", x);
        telemetry.addData("y", y);
        telemetry.addData("Steady Angle", steadyHeading);

        if (headingSet) {

            setPowerForFullRangeHolonomic(x, y, currentHeading);
            setMotorPowerLevels(powerLevels);

        } else {

            TurnOffAllDriveMotors();
        }
    }
}
