package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.configuration.MatrixConstants;

/**
 * Created by 4924_Users on 12/17/2016.
 */

@TeleOp(name = "FullHolonomic")
public abstract class FullHolonomic extends OpMode {

    GyroSensor turningGyro;
    DcMotor frontRightMotor;
    DcMotor frontLeftMotor;
    DcMotor backRightMotor;
    DcMotor backLeftMotor;
    
    public boolean isTurningLeft = false;
    public boolean isTurningRight = false;
    public boolean headingSet = false;
    public int steadyHeading = 0;

    public float leftTriggerValue() { return gamepad1.left_trigger; }
    public float rightTriggerValue() { return gamepad1.right_trigger; }
    PowerLevels powerLevels = new PowerLevels(0.0f, 0.0f, 0.0f, 0.0f);
    public PowerLevels zeroPowerLevels = new PowerLevels(0.0f, 0.0f, 0.0f, 0.0f);

    public void setPowerForFullRangeHolonomic(float x, float y, int heading, float turnPower) {

        int headingDifference = steadyHeading - heading;

        if (isTurningLeft) {

            powerLevels.frontLeftPower = y - x - turnPower;
            powerLevels.backLeftPower = y + x - turnPower;
            powerLevels.backRightPower = y - x;
            powerLevels.frontRightPower = y + x;

        } else {

            if (isTurningRight) {

                powerLevels.frontLeftPower = y - x;
                powerLevels.backLeftPower = y + x;
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

    public void setMotorPowerLevels(PowerLevels powerLevels) {

        frontLeftMotor.setPower(powerLevels.frontLeftPower);
        backLeftMotor.setPower(powerLevels.backLeftPower);
        backRightMotor.setPower(powerLevels.backRightPower);
        frontRightMotor.setPower(powerLevels.frontRightPower);
    }

    public void TurnOffAllDriveMotors() {
        SetDriveMotorPowerLevels(zeroPowerLevels);
    }

    public void SetDriveMotorPowerLevels(PowerLevels levels) {

        frontRightMotor.setPower(levels.frontRightPower);
        frontLeftMotor.setPower(levels.frontLeftPower);
        backRightMotor.setPower(levels.backRightPower);
        backLeftMotor.setPower(levels.backLeftPower);
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
