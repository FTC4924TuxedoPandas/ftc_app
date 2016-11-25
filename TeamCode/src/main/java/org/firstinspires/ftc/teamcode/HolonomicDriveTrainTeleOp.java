package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

/**
 * Created by 4924_Users on 10/8/2016.
 */

@TeleOp(name = "HolonomicDriveTrainTeleOp")
public class HolonomicDriveTrainTeleOp extends VelocityBase {

    protected int triggerDirection = 1;

    @Override
    public void loop() {

        driveStickX = gamepad1.left_stick_x;
        driveStickY = gamepad1.left_stick_y;

        if (dpadUpIsPressed()) {

            raiseThrowingArm();

        } else if (dpadDownIsPressed()) {

            lowerThrowingArm();

        } else {

            stopMovingThrowingArm();
        }

        if (isDiagonal()) {

            setPowerForDiagonalMove((Math.abs(driveStickX) + Math.abs(driveStickY)) / 2);

        } else if (isStrafing()) {

            setPowerForMecanumStrafe(driveStickX);

        } else {

            setPowerForLinearMove(driveStickY);

            if (leftTriggerValue() > 0.1f) {

                setPowerForTurning(leftTriggerValue() * triggerDirection);
            }

            if (rightTriggerValue() > 0.1f) {

                setPowerForTurning(-rightTriggerValue() * triggerDirection);
            }
        }

        if (d1BIsPressed()) {

            openGateLow();

        } else if (d1YIsPressed()) {

            openGateHigh();

        } else if (d1AIsPressed()) {

            closeGate();
        }

        if (collectionIn()) {

            collectionIntake();

        } else if (collectionOut()) {

            collectionRelease();

        } else {

            collectionOff();
        }

        resolveBeaconServos();

        if (gamepad1.dpad_down && (time.time() > DELAY)){
            reversed = !reversed;
            time.reset();
        }

        if (d1DownIsPressed()) {

            driveStickX = -gamepad1.left_stick_x;
            driveStickY = -gamepad1.left_stick_y;
            triggerDirection = -triggerDirection;
        }

        // clip servo values
        rightBeaconServoPosition = Range.clip(rightBeaconServoPosition, -1.0f, 1.0f);
        leftBeaconServoPosition = Range.clip(leftBeaconServoPosition, -1.0f, 1.0f);

        rightBeaconServo.setPosition(rightBeaconServoPosition);
        leftBeaconServo.setPosition(leftBeaconServoPosition);

        clipPowerLevels();
        setMotorPowerLevels(powerLevels);
        telemetry.addData("Left Servo", leftBeaconServoPosition);
        telemetry.addData("Right Servo", rightBeaconServoPosition);
        telemetry.addData("FrontRight: ", powerLevels.frontRightPower);
        telemetry.addData("FrontLeft: ", powerLevels.frontLeftPower);
        telemetry.addData("BackRight: ", powerLevels.backRightPower);
        telemetry.addData("BackLeft: ", powerLevels.backLeftPower);
        telemetry.addData("isDiagonal: ", isDiagonal());
        telemetry.addData("isStrafing: ", isStrafing());
    }
}