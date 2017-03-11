package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;

/**
 * Created by 4924_Users on 12/17/2016.
 * Latest fixes made on 12/23/2016
 */

@TeleOp(name = "FullHolonomic")
public class FullHolonomic extends RevolutionVelocityBase {

    private boolean throwing;
    private double throwStartTime;
    private double throwInterval = 0.0;
    private double switchModeStartTime;
    private final double THROW_INPUT_DELAY = 0.7;
    private final double BOUNCE_DELAY = 0.2;
    private float driveCoeff;

    public boolean gyroCorrecting = false;

    @Override
    public void init() {

        super.init();

        driveDirection = -1;
        driveCoeff = 1;
        turningGyro.calibrate();

        throwing = false;
    }

    @Override
    public void init_loop() {

        rightBeaconServo.setPosition(BEACON_SERVO_POSITION_IN);
        leftBeaconServo.setPosition(BEACON_SERVO_POSITION_IN);
        shovelLockServo.setPosition(LOCK_SERVO_POSITION_CLOSED);
        collectionGateServo.setPosition(GATE_SERVO_POSITION_CLOSED);

    }

    private void setPowerForFullHolonomic(float x, float y, int heading, float leftTurnPower, float rightTurnPower, int driveDirection) {

        x *= driveDirection;
        y *= driveDirection;

        int headingDifference = steadyHeading - heading;

        if (isTurningLeft || isTurningRight) {

            powerLevels.frontLeftPower = y - x + leftTurnPower - rightTurnPower;
            powerLevels.backLeftPower = y + x + leftTurnPower - rightTurnPower;
            powerLevels.backRightPower = y - x - leftTurnPower + rightTurnPower;
            powerLevels.frontRightPower = y + x - leftTurnPower + rightTurnPower;

        } else {

            if (gyroCorrecting) {

                if (steadyHeading - heading >= 0) {

                    headingDifference = steadyHeading - heading;
                }

                if (heading - steadyHeading >= 0) {

                    headingDifference = 0 - heading + steadyHeading;
                }

                if (headingDifference < 0) {

                    powerLevels.frontLeftPower = (y - x) - Math.abs(headingDifference / 10);
                    powerLevels.backLeftPower = (y + x) - Math.abs(headingDifference / 10);
                    powerLevels.backRightPower = y - x;
                    powerLevels.frontRightPower = y + x;

                } else {

                    powerLevels.frontLeftPower = y - x;
                    powerLevels.backLeftPower = y + x;
                    powerLevels.backRightPower = (y - x) - (headingDifference / 10);
                    powerLevels.frontRightPower = (y + x) - (headingDifference / 10);
                }

            } else {

                powerLevels.frontLeftPower = y - x;
                powerLevels.backLeftPower = y + x;
                powerLevels.backRightPower = y - x;
                powerLevels.frontRightPower = y + x;
            }
        }

        setCoeffPowerLevels(driveDirection, driveCoeff);
    }

    private void setCoeffPowerLevels(int driveDirection, float driveCoeff) {
        powerLevels.frontLeftPower *= driveCoeff;
        powerLevels.backLeftPower *= driveCoeff;
        powerLevels.frontRightPower *= driveCoeff;
        powerLevels.backRightPower *= driveCoeff;
    }

    @Override
    public void loop() {
        int currentHeading = turningGyro.getHeading();

        float x = -gamepad1.left_stick_x;
        float y = -gamepad1.left_stick_y;

        isTurningLeft = leftTriggerValue() > 0.01f;
        isTurningRight = rightTriggerValue() > 0.01f;

        winchPowerLevel = -gamepad2.left_stick_y;

        if (d1DPadDownIsPressed()) {

            driveDirection = -1;

        } else if (d1DPadUpIsPressed()) {

            driveDirection = 1;
        }

        if (d1XIsPressed()) {

            leftBeaconServoOut();

        } else if (d1YIsPressed()) {

            leftBeaconServoIn();
        }

        if (d1AIsPressed() && !d1StartIsPressed()) {

            rightBeaconServoOut();

        } else if (d1BIsPressed()) {

            rightBeaconServoIn();
        }

        if (d2DPadUpIsPressed()) {

            raiseThrowingArm();

        } else if (d2DPadDownIsPressed()) {

            lowerThrowingArm();

        } else {

            stopMovingThrowingArm();
        }

        if (collectionIn()) {

            collectionIntake();

        } else if (collectionOut()) {

            collectionRelease();

        } else {

            collectionOff();
        }

        if (d2XIsPressed()) {

            openGate();

        } else if (d2AIsPressed()) {

            closeGate();
        }
/*
        if (d1BackIsPressed() && ((time.time() - switchModeStartTime) > BOUNCE_DELAY)) {

            gyroCorrecting = !gyroCorrecting;
            switchModeStartTime = time.time();
        }
*/
        if (d2YIsPressed() && ((time.time() - throwStartTime) > THROW_INPUT_DELAY)) {

            throwing = true;
            throwInterval = 0.4;
            throwStartTime = time.time();
        }

        if (throwing && ((time.time() - throwStartTime) < throwInterval)) {

            throwingArmPowerLevel = 1.0f;

        } else if (throwing) {

            throwing = false;
            throwingArmPowerLevel = 0.0f;
        }

        if (d1DPadLeftIsPressed() && d2DPadLeftIsPressed()) {

            lockShovel();
        }

        if (d1DPadRightIsPressed() && d2DPadRightIsPressed()) {

            unlockShovel();
        }

        if (d2RightStickIsLeft()) {

            spinLeft();

        } else if (d2RightStickIsRight()) {

            spinRight();

        } else {

            spinStop();
        }

        if (d1LeftBumperIsPressed()) {

            driveCoeff = 1f;
        }

        if (d1RightBumperIsPressed()) {

            driveCoeff = 0.4f;
        }

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
        telemetry.addData("SpinningServo Stop", spinningServoPosition);

        // clip servo values
        rightBeaconServoPosition = Range.clip(rightBeaconServoPosition, -1.0f, 1.0f);
        leftBeaconServoPosition = Range.clip(leftBeaconServoPosition, -1.0f, 1.0f);

        rightBeaconServo.setPosition(rightBeaconServoPosition);
        leftBeaconServo.setPosition(leftBeaconServoPosition);
        shovelLockServo.setPosition(shovelLockServoPosition);
        collectionGateServo.setPosition(gateServoPosition);
        spinningServo.setPosition(spinningServoPosition);

        if (headingSet || isTurningLeft || isTurningRight) {

            setPowerForFullHolonomic(x, y, currentHeading, leftTriggerValue(), rightTriggerValue(), driveDirection);
            setMotorPowerLevels(powerLevels);

        } else {

            TurnOffAllDriveMotors();
        }

        throwingArm.setPower(throwingArmPowerLevel);
        collectionMotor.setPower(collectionPowerLevel);
        winchMotorOne.setPower(winchPowerLevel);
        winchMotorTwo.setPower(winchPowerLevel);
    }

}
