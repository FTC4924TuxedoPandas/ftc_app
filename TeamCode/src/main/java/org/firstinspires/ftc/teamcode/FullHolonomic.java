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
    }

    @Override
    public void init_loop() {

        rightBeaconServo.setPosition(BEACON_SERVO_POSITION_IN);
        leftBeaconServo.setPosition(BEACON_SERVO_POSITION_IN);
        shovelLockServo.setPosition(LOCK_SERVO_POSITION_CLOSED);
        collectionGateServo.setPosition(GATE_SERVO_POSITION_CLOSED);

    }

    private void setPowerForFullHolonomic(float x, float y, int heading, float leftTurnPower, float rightTurnPower, int driveDirection) {

        int headingDifference = steadyHeading - heading;

        if (isTurningLeft || isTurningRight) {

            powerLevels.frontLeftPower = y - x - leftTurnPower + rightTurnPower;
            powerLevels.backLeftPower = y + x - leftTurnPower + rightTurnPower;
            powerLevels.backRightPower = y - x + leftTurnPower - rightTurnPower;
            powerLevels.frontRightPower = y + x + leftTurnPower - rightTurnPower;

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
/*
        if (lowSensitivity) {
            powerLevels.frontLeftPower = getSensitivePowerLevel(powerLevels.frontLeftPower);
            powerLevels.backLeftPower = getSensitivePowerLevel(powerLevels.backLeftPower);
            powerLevels.backRightPower = getSensitivePowerLevel(powerLevels.backRightPower);
            powerLevels.frontRightPower = getSensitivePowerLevel(powerLevels.frontRightPower);
        }

        if (highSensitivity) {
            powerLevels.frontLeftPower = getSensitivePowerLevel(powerLevels.frontLeftPower);
            powerLevels.backLeftPower = getSensitivePowerLevel(powerLevels.backLeftPower);
            powerLevels.backRightPower = getSensitivePowerLevel(powerLevels.backRightPower);
            powerLevels.frontRightPower = getSensitivePowerLevel(powerLevels.frontRightPower);
        }
 */
    }
/*
    private float getSensitivePowerLevel(float motorPower) { //returns square of value; if below 1, will be lowered exponentially; if above 1, will be clipped to 1 later
        int direction = 1;
        if (motorPower < 0) {
            direction = -1;
        }
        return (float) ((Math.pow((double) motorPower, 2.0)) * direction);
    }

    private void setLowSensitivity() {

        lowSensitivity = true;
    }

    private void setHighSensitivity() {

        highSensitivity = true;
    }

    private void resetSensitivity() {

        lowSensitivity = false;
        highSensitivity = false;
    }
*/

    private void setCoeffPowerLevels(int driveDirection, float driveCoeff) {
        powerLevels.frontLeftPower *= driveDirection * driveCoeff;
        powerLevels.backLeftPower *= driveDirection * driveCoeff;
        powerLevels.frontRightPower *= driveDirection * driveCoeff;
        powerLevels.backRightPower *= driveDirection * driveCoeff;
    }

    private void throwBalls(double throwTime) {

        time.reset();
        while (time.time() <= throwTime) {

            telemetry.addData("time",time.time());
            throwingArm.setPower(0.9f);
        }
        throwingArm.setPower(0.0f);
    }

    @Override
    public void loop() {
        int currentHeading = turningGyro.getHeading();

        float x = -gamepad1.left_stick_x;
        float y = -gamepad1.left_stick_y;

        isTurningLeft = leftTriggerValue() > 0.01f;
        isTurningRight = rightTriggerValue() > 0.01f;

        winchPowerLevel = -gamepad2.left_stick_y;
/*
        if (winchUp()) {

            winchIntake();

        } else if (winchDown()) {

            winchRelease();

        } else {

            winchOff();
        }
*/
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

        if (d2YIsPressed()) {

            throwBalls(0.2);
        }

        if (d2BIsPressed()) {

            throwBalls(0.3);
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

            setPowerForFullHolonomic(x, y, currentHeading, leftTriggerValue(), rightTriggerValue(), driveDirection);
            setMotorPowerLevels(powerLevels);

        } else {

            TurnOffAllDriveMotors();
        }

        throwingArm.setPower(throwingArmPowerLevel);
        collectionMotor.setPower(collectionPowerLevel);
        winchMotor.setPower(winchPowerLevel);
    }

}
