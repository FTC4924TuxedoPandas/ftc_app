package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
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
    private double throwInterval = 0.2;
    private final double THROW_INPUT_DELAY = 0.7;

    public boolean gyroCorrecting = true;

    @Override
    public void clipPowerLevels() {

        powerLevels.backRightPower = Range.clip(powerLevels.backRightPower, -1.0f, 1.0f);
        powerLevels.backLeftPower = Range.clip(powerLevels.backLeftPower, -1.0f, 1.0f);
        powerLevels.frontRightPower = Range.clip(powerLevels.frontRightPower, -1.0f, 1.0f);
        powerLevels.frontLeftPower = Range.clip(powerLevels.frontLeftPower, -1.0f, 1.0f);
        throwingArmPowerLevel = Range.clip(throwingArmPowerLevel, -1.0f, 1.0f);
        collectionPowerLevel = Range.clip(collectionPowerLevel, -1.0f, 1.0f);
    }


    @Override
    public void init() {

        super.init();
        winchMotor = hardwareMap.dcMotor.get("winchMotor");
        winchMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        highSensitivity = false;
        lowSensitivity = false;

        driveDirection = 1;
        driveCoeff = 1;
        turningGyro.calibrate();

        throwing = false;
        throwInterval = 0.0;
    }

    @Override
    public void init_loop() {

        rightBeaconServo.setPosition(BEACON_SERVO_POSITION_IN);
        leftBeaconServo.setPosition(BEACON_SERVO_POSITION_IN);
        shovelLockServo.setPosition(LOCK_SERVO_POSITION_CLOSED);
        collectionGateServo.setPosition(GATE_SERVO_POSITION_CLOSED);

    }

    private void setPowerForFullHolonomic(float x, float y, int heading, float turningPower, int driveDirection) {

        int headingDifference = steadyHeading - heading;

        if (isTurningLeft) {

            powerLevels.frontLeftPower = y - x - turningPower;
            powerLevels.backLeftPower = y + x - turningPower;
            powerLevels.backRightPower = y - x;
            powerLevels.frontRightPower = y + x;

        } else if (isTurningRight) {

            powerLevels.frontLeftPower = y - x;
            powerLevels.backLeftPower = y + x;
            powerLevels.backRightPower = y - x - turningPower;
            powerLevels.frontRightPower = y + x - turningPower;

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
        powerLevels.frontLeftPower *= driveDirection * driveCoeff;
        powerLevels.backLeftPower *= driveDirection * driveCoeff;
        powerLevels.frontRightPower *= driveDirection * driveCoeff;
        powerLevels.backRightPower *= driveDirection * driveCoeff;
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

        if (d1AIsPressed()) {

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

        if (d2YIsPressed() && ((time.time() - throwStartTime) > THROW_INPUT_DELAY)) {

            throwing = true;
            throwInterval = 0.2;
            throwStartTime = time.time();
        }

        if (d2BIsPressed() && ((time.time() - throwStartTime) > THROW_INPUT_DELAY)) {

            throwing = true;
            throwInterval = 0.3;
            throwStartTime = time.time();
        }

        if (throwing && ((time.time() - throwStartTime) < throwInterval)) {

            throwingArmPowerLevel = 0.9f;
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

        // clip servo values
        rightBeaconServoPosition = Range.clip(rightBeaconServoPosition, -1.0f, 1.0f);
        leftBeaconServoPosition = Range.clip(leftBeaconServoPosition, -1.0f, 1.0f);

        rightBeaconServo.setPosition(rightBeaconServoPosition);
        leftBeaconServo.setPosition(leftBeaconServoPosition);
        shovelLockServo.setPosition(shovelLockServoPosition);
        collectionGateServo.setPosition(gateServoPosition);

        if (headingSet || isTurningLeft || isTurningRight) {

            if (isTurningLeft) {

                setPowerForFullHolonomic(x, y, currentHeading, leftTriggerValue(), driveDirection);

            } else {

                setPowerForFullHolonomic(x, y, currentHeading, rightTriggerValue(), driveDirection);
            }

            setMotorPowerLevels(powerLevels);

        } else {

            TurnOffAllDriveMotors();
        }

        throwingArm.setPower(throwingArmPowerLevel);
        collectionMotor.setPower(collectionPowerLevel);
        winchMotor.setPower(winchPowerLevel);
    }

}
