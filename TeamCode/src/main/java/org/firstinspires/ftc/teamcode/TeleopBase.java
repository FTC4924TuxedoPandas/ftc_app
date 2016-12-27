package org.firstinspires.ftc.teamcode;

/**
 * Created by btipt on 12/27/2016.
 */

public class TeleopBase extends VelocityBase {

    public final float DIAGONAL_MARGIN_OF_ERROR = 1.0f;
    public final float STICK_THRESHOLD = 0.1f;

    @Override
    public void init() {

        super.init();

    }

    public void setPowerForFullRangeHolonomic(float x, float y, int heading, float turnPower) {

        float headingDifference = steadyHeading - heading;

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

                    powerLevels.frontLeftPower = (y - x) - Math.abs(headingDifference / 10.0f);
                    powerLevels.backLeftPower = (y + x) - Math.abs(headingDifference / 10.0f);
                    powerLevels.backRightPower = y - x;
                    powerLevels.frontRightPower = y + x;

                } else {

                    powerLevels.frontLeftPower = y - x;
                    powerLevels.backLeftPower = y + x;
                    powerLevels.backRightPower = (y - x) - (headingDifference / 10.0f);
                    powerLevels.frontRightPower = (y + x) - (headingDifference / 10.0f);
                }
            }
        }
    }

    public boolean d2XIsPressed() {return gamepad2.x;}

    public boolean d2AIsPressed() { return gamepad2.a; }

    public boolean d2YIsPressed() {
        return gamepad2.y;
    }

    public boolean d2BIsPressed() { return gamepad2.b; }

    public boolean d1AIsPressed() { return gamepad1.a; }

    public boolean d1BIsPressed() { return gamepad1.b; }

    public boolean d1YIsPressed() { return gamepad1.y; }

    public boolean d1XIsPressed() { return gamepad1.x; }

    public boolean d1DownIsPressed() { return gamepad1.dpad_down; }

    public boolean d1DPadLeftIsPressed() { return gamepad1.dpad_left; }

    public boolean d2DPadLeftIsPressed() { return gamepad2.dpad_left; }

    public boolean d2DPadrightIsPressed() { return gamepad2.dpad_right; }

    public float leftTriggerValue() { return gamepad1.left_trigger; }

    public float rightTriggerValue() { return gamepad1.right_trigger; }

    public void resolveBeaconServos() {

        if (d1XIsPressed() && buttonDelay.time() >= 0.3f) {

            leftBeaconServoOut();
            buttonDelay.reset();

        } else if (d1YIsPressed() && buttonDelay.time() >= 0.3f) {

            leftBeaconServoIn();
            buttonDelay.reset();
        }

        if (d1AIsPressed() && buttonDelay.time() >= 0.3f) {

            rightBeaconServoOut();
            buttonDelay.reset();

        } else if (d1BIsPressed() && buttonDelay.time() >= 0.3f) {

            rightBeaconServoIn();
            buttonDelay.reset();
        }
    }

    public boolean d1DPadUpIsPressed() { return gamepad1.dpad_up; }

    public boolean d1DPadDownIsPressed() { return gamepad1.dpad_down; }

    public boolean d2DPadDownIsPressed() { return gamepad2.dpad_down; }

    public boolean d2DPadUpIsPressed() { return gamepad2.dpad_up; }

    public boolean collectionIn() {return gamepad2.right_bumper;}

    public boolean collectionOut() {
        return gamepad2.left_bumper;
    }

    public boolean isDiagonal() {

        if (Math.abs(driveStickX) >= STICK_THRESHOLD || Math.abs(driveStickY) >= STICK_THRESHOLD) {

            return Math.abs(Math.abs(driveStickX) - Math.abs(driveStickY)) <=
                    DIAGONAL_MARGIN_OF_ERROR * ((Math.abs(driveStickX) + Math.abs(driveStickY)) / 2);
        }

        return false;
    }

    public boolean isStrafing() {

        return Math.abs(driveStickX) >= STICK_THRESHOLD;
    }
}
