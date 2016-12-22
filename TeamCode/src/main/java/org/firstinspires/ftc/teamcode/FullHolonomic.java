package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.configuration.MatrixConstants;

/**
 * Created by 4924_Users on 12/17/2016.
 */

@TeleOp(name = "FullHolonomic")
public class FullHolonomic extends VelocityBase {

    @Override
    public void init() {

        frontRightMotor = hardwareMap.dcMotor.get("frontRightMotor");
        frontLeftMotor = hardwareMap.dcMotor.get("frontLeftMotor");
        backRightMotor = hardwareMap.dcMotor.get("backRightMotor");
        backLeftMotor = hardwareMap.dcMotor.get("backLeftMotor");
        throwingArm = hardwareMap.dcMotor.get("throwingArm");
        collectionMotor = hardwareMap.dcMotor.get("collectionMotor");

        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        frontLeftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        throwingArm.setDirection(DcMotorSimple.Direction.FORWARD);
        collectionMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        turningGyro = hardwareMap.gyroSensor.get("gyroSensor");
        currentState = VelocityBase.State.STATE_INITIAL;

        useRunUsingEncoders();
        countsPerInch = (COUNTS_PER_REVOLUTION / (Math.PI * WHEEL_DIAMETER)) * GEAR_RATIO * CALIBRATION_FACTOR;
        frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

    @Override
    public void init_loop() {

    }

    public void setPowerForFullHolonomic(float x, float y, int heading, float turnPower) {

        int headingDifference = steadyHeading - heading;

        if (isTurningLeft) {

            powerLevels.frontLeftPower = y - x - turnPower;
            powerLevels.backLeftPower = y + x - turnPower;
            powerLevels.backRightPower = y - x + turnPower;
            powerLevels.frontRightPower = y + x + turnPower;

        } else {

            if (isTurningRight) {

                powerLevels.frontLeftPower = y - x + turnPower;
                powerLevels.backLeftPower = y + x + turnPower;
                powerLevels.backRightPower = y - x - turnPower;
                powerLevels.frontRightPower = y + x - turnPower;

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
        }
    }

    public void setPowerLevels(PowerLevels powerLevels) {

        frontLeftMotor.setPower(powerLevels.frontLeftPower);
        backLeftMotor.setPower(powerLevels.backLeftPower);
        backRightMotor.setPower(powerLevels.backRightPower);
        frontRightMotor.setPower(powerLevels.frontRightPower);
    }

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

                setPowerForFullHolonomic(x, y, currentHeading, leftTriggerValue());
                setPowerLevels(powerLevels);

            } else {

                if (isTurningRight) {

                    setPowerForFullHolonomic(x, y, currentHeading, rightTriggerValue());

                } else {

                    setPowerForFullHolonomic(x, y, currentHeading, 0.0f);
                }

                setPowerLevels(powerLevels);
            }

        } else {

            TurnOffAllDriveMotors();
        }
    }

    public boolean collectionIn() {
        return gamepad2.right_bumper;
    }

    public boolean collectionOut() {
        return gamepad2.left_bumper;
    }

    public boolean d2XIsPressed() {
        return gamepad2.x;
    }

    public boolean d2AIsPressed() { return gamepad2.a; }

    public boolean d2YIsPressed() {
        return gamepad2.y;
    }

    public boolean d2BIsPressed() { return gamepad2.b; }

}

