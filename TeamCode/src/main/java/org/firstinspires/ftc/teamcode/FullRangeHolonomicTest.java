package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.configuration.MatrixConstants;

/**
 * Created by 4924_Users on 12/17/2016.
 */

@TeleOp(name = "FullRangeHolonomicTest")
public class FullRangeHolonomicTest extends VelocityBase {

    @Override
    public void loop() {

        int currentHeading = turningGyro.getHeading();

        float x = -gamepad1.left_stick_x;
        float y = -gamepad1.left_stick_y;

        isTurningLeft = leftTriggerValue() > 0.01f;
        isTurningRight = rightTriggerValue() > 0.01f;


        if (Math.abs(x) > 0.01 || Math.abs(y) > 0.01) {

            headingSet = true;
        }

        if ((Math.abs(x) < 0.01 && Math.abs(y) < 0.01)) {

            headingSet = false;
            steadyHeading = currentHeading;
        }

        if (isTurningLeft || isTurningRight) {

            steadyHeading = currentHeading;
        }

        telemetry.addData("x", x);
        telemetry.addData("y", y);
        telemetry.addData("Steady Angle", steadyHeading);

        if (headingSet) {

            if (isTurningLeft) {

                setPowerForFullRangeHolonomic(x, y, currentHeading, leftTriggerValue());
                setMotorPowerLevels(powerLevels);

            } else {

                if (isTurningRight) {

                    setPowerForFullRangeHolonomic(x, y, currentHeading, rightTriggerValue());

                } else {

                    setPowerForFullRangeHolonomic(x, y, currentHeading, 0.0f);
                }

                setMotorPowerLevels(powerLevels);
            }

        } else {

            TurnOffAllDriveMotors();
        }
    }
}
