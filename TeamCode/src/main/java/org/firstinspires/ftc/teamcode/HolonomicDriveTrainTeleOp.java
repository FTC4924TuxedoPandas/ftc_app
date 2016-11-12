package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * Created by 4924_Users on 10/8/2016.
 */

@TeleOp(name = "HolonomicDriveTrainTeleOp")
public class HolonomicDriveTrainTeleOp extends VelocityBase {

    @Override
    public void loop() {

        driveStickX = gamepad1.left_stick_x;
        driveStickY = gamepad1.left_stick_y;

        if (leftTriggerValue() > 0.0f) {

            setPowerForTurning(leftTriggerValue());
        }

        if (rightTriggerValue() > 0.0f) {

            setPowerForTurning(rightTriggerValue());
        }

        if (dpadUpIsPressed()) {

            RaiseThrowingArm();

        } else if (dpadDownIsPressed()) {

            LowerThrowingArm();

        } else {

            StopMovingThrowingArm();
        }

        if (isDiagonal()) {

            setPowerForDiagonalMove((Math.abs(driveStickX) + Math.abs(driveStickY)) / 2);

        } else if (isStrafing()) {

            setPowerForMecanumStrafe(driveStickX);

        } else {

            setPowerForLinearMove(driveStickY);
        }

        if (d1AIsPressed()) {

            openGateLow();

        } else if (d1YIsPressed()) {

            openGateHigh();

        } else if (d1BIsPressed()) {

            closeGate();
        }

        if (collectionIn()) {

            collectionIntake();

        } else if (collectionOut()) {

            collectionRelease();

        } else {

            collectionOff();
        }

        if (d2XIsPressed()) {

            leftBeaconServoOut();

        } else if (d2YIsPressed()) {

            leftBeaconServoIn();
        }

        if (d2AIsPressed()) {

            rightBeaconServoOut();

        } else if (d2BIsPressed()) {

            rightBeaconServoIn();
        }

        if (gamepad1.dpad_down && (time.time() > DELAY)){
            reversed = !reversed;
            time.reset();
        }

        clipPowerLevels();
        setMotorPowerLevels(powerLevels);
        telemetry.addData("FrontRight: ", powerLevels.frontRightPower);
        telemetry.addData("FrontLeft: ", powerLevels.frontLeftPower);
        telemetry.addData("BackRight: ", powerLevels.backRightPower);
        telemetry.addData("BackLeft: ", powerLevels.backLeftPower);
        telemetry.addData("isDiagonal: ", isDiagonal());
        telemetry.addData("isStrafing: ", isStrafing());
    }
}