package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.configuration.MatrixConstants;
import com.qualcomm.robotcore.util.Range;

/**
 * Created by 4924_Users on 12/17/2016.
 * Latest fixes made on 12/23/2016
 */

@TeleOp(name = "FullHolonomic")
public class FullHolonomic extends TeleopBase {

    @Override
    public void clipPowerLevels() {

        powerLevels.backRightPower = Range.clip(powerLevels.backRightPower, -1.0f, 1.0f);
        powerLevels.backLeftPower = Range.clip(powerLevels.backLeftPower, -1.0f, 1.0f);
        powerLevels.frontRightPower = Range.clip(powerLevels.frontRightPower, -1.0f, 1.0f);
        powerLevels.frontLeftPower = Range.clip(powerLevels.frontLeftPower, -1.0f, 1.0f);
        throwingArmPowerLevel = Range.clip(throwingArmPowerLevel, -1.0f, 1.0f);
        collectionPowerLevel = Range.clip(collectionPowerLevel, -1.0f, 1.0f);
        linearSlidePowerLevel = Range.clip(linearSlidePowerLevel, -1.0f, 1.0f);

    }


    @Override
    public void init() {

        super.init();
        linearSlideMotor = hardwareMap.dcMotor.get("linearSlideMotor");
        linearSlideMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        highSensitivity = false;
        lowSensitivity = false;

        driveDirection = 1;
    }

    @Override
    public void init_loop() {

        rightBeaconServo.setPosition(BEACON_SERVO_POSITION_IN);
        leftBeaconServo.setPosition(BEACON_SERVO_POSITION_IN);

    }

    private void setPowerForFullHolonomic(float x, float y, int heading, float leftTurnPower, float rightTurnPower, int driveDirection) {

        int headingDifference = steadyHeading - heading;

        if (isTurningLeft || isTurningRight) {

            powerLevels.frontLeftPower = y - x - leftTurnPower + rightTurnPower;
            powerLevels.backLeftPower = y + x - leftTurnPower + rightTurnPower;
            powerLevels.backRightPower = y - x + leftTurnPower - rightTurnPower;
            powerLevels.frontRightPower = y + x + leftTurnPower - rightTurnPower;

        } else {

            if (steadyHeading - heading >= 180) {

                headingDifference = steadyHeading - 360 - heading;
            }

            if (heading - steadyHeading >= 180) {

                headingDifference = 360 - heading + steadyHeading;
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
        }

        setCoeffPowerLevels(driveDirection);

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
    }

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

    @Override
    public void loop() {
        int currentHeading = turningGyro.getHeading();

        float x = -gamepad1.left_stick_x;
        float y = -gamepad1.left_stick_y;

        isTurningLeft = leftTriggerValue() > 0.01f;
        isTurningRight = rightTriggerValue() > 0.01f;

        if (d1DPadDownIsPressed()) {

            driveDirection = -1;

        } else if (d1DPadUpIsPressed()) {

            driveDirection = 1;

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

        if (d1XIsPressed()) {

            setLowSensitivity();
        }

        if (d1BIsPressed()) {

            setHighSensitivity();
        }

        if (d1YIsPressed()) {

            resetSensitivity();
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
        telemetry.addData("Collection", collectionIn());
        telemetry.addData("Throwing Arm", d2DPadUpIsPressed());
        telemetry.addData("Collection", collectionOut());
        telemetry.addData("Throwing Arm", d2DPadDownIsPressed());
        telemetry.addData("Collection", collectionPowerLevel);
        telemetry.addData("leftBeaconServo", d2XIsPressed());
        telemetry.addData("leftBeaconServo", d2YIsPressed());
        telemetry.addData("rightBeaconServo", d2AIsPressed());
        telemetry.addData("rightBeaconServo", d2BIsPressed());

        // clip servo values
        rightBeaconServoPosition = Range.clip(rightBeaconServoPosition, -1.0f, 1.0f);
        leftBeaconServoPosition = Range.clip(leftBeaconServoPosition, -1.0f, 1.0f);

        rightBeaconServo.setPosition(rightBeaconServoPosition);
        leftBeaconServo.setPosition(leftBeaconServoPosition);

        if (headingSet || isTurningLeft || isTurningRight) {

            setPowerForFullHolonomic(x, y, currentHeading, leftTriggerValue(), rightTriggerValue(), driveDirection);
            setMotorPowerLevels(powerLevels);

        } else {

            TurnOffAllDriveMotors();
        }

        throwingArm.setPower(throwingArmPowerLevel);
        collectionMotor.setPower(collectionPowerLevel);
    }

    private void setCoeffPowerLevels(int driveDirection) {
        powerLevels.frontLeftPower *= driveDirection;
        powerLevels.backLeftPower *= driveDirection;
        powerLevels.frontRightPower *= driveDirection;
        powerLevels.backRightPower *= driveDirection;
    }
}


