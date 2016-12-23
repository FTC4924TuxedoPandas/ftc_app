package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

/**
 * Created by 4924_Users on 10/8/2016.
 */

@TeleOp(name = "HolonomicDriveTrainTeleOp")
public class HolonomicDriveTrainTeleOp extends VelocityBase {

    public int reverseFactor = 1;

    @Override
    public void loop() {

        driveStickX = gamepad1.left_stick_x;
        driveStickY = gamepad1.left_stick_y;

        if (d2DPadUpIsPressed()) {

            raiseThrowingArm();

        } else if (d2DPadDownIsPressed()) {

            lowerThrowingArm();

        } else {

            stopMovingThrowingArm();
        }

        if (isDiagonal()) {

            setPowerForDiagonalMove((Math.abs(driveStickX) + Math.abs(driveStickY)) / 2 * reverseFactor);

        } else if (isStrafing()) {

            setPowerForMecanumStrafe(driveStickX * reverseFactor, turningGyro.getHeading());

        } else {

            setPowerForLinearMove(driveStickY * reverseFactor);

            if (leftTriggerValue() > 0.1f) {

                setPowerForTurning(leftTriggerValue());
            }

            if (rightTriggerValue() > 0.1f) {

                setPowerForTurning(-rightTriggerValue());
            }
        }

        if (d2BIsPressed()) {

            openGateLow();

        } else if (d2YIsPressed()) {

            openGateHigh();

        } else if (d2AIsPressed()) {

            closeGate();
        }

        if (collectionIn()) {

            collectionIntake();

        } else if (collectionOut()) {

            collectionRelease();

        } else {

            collectionOff();
        }

        if (d1DPadLeftIsPressed() && d2DPadLeftIsPressed()) {

            shovelLockServo.setPosition(1.0f);
        }

        resolveBeaconServos();

        if (gamepad1.dpad_down && (time.time() > DELAY)){

            reverseFactor *= -1;
            time.reset();
        }

        // clip servo values
        rightBeaconServoPosition = Range.clip(rightBeaconServoPosition, -1.0f, 1.0f);
        leftBeaconServoPosition = Range.clip(leftBeaconServoPosition, -1.0f, 1.0f);

        rightBeaconServo.setPosition(rightBeaconServoPosition);
        leftBeaconServo.setPosition(leftBeaconServoPosition);
        collectionGateServo.setPosition(gateServoPosition);

        clipPowerLevels();
        setMotorPowerLevels(powerLevels);

        telemetry.addData("On Line", lineSensor.getRawLightDetected() >= 0.5f);

        //telemetry.addData("Left Servo", leftBeaconServoPosition);
        //telemetry.addData("Right Servo", rightBeaconServoPosition);
        //telemetry.addData("FrontRight: ", powerLevels.frontRightPower);
        //telemetry.addData("FrontLeft: ", powerLevels.frontLeftPower);
        //telemetry.addData("BackRight: ", powerLevels.backRightPower);
        //telemetry.addData("BackLeft: ", powerLevels.backLeftPower);
        //telemetry.addData("isDiagonal: ", isDiagonal());
        //telemetry.addData("isStrafing: ", isStrafing());
    }
}