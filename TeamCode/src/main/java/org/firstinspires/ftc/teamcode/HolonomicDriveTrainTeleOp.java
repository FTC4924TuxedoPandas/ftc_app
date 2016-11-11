package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * Created by 4924_Users on 10/8/2016.
 */

@TeleOp(name = "HolonomicDriveTrainTeleOp")
public class HolonomicDriveTrainTeleOp extends VelocityBase {

    @Override
    public void loop() {

        if (aIsPressed()) {

            RaiseThrowingArm();

        } else if (bIsPressed()) {

            LowerThrowingArm();

        } else {

            StopMovingThrowingArm();
        }

        isStrafingLeft = gamepad1.left_bumper;
        isStrafingRight = gamepad1.right_bumper;

        if (isStrafing()) {

            setPowerForMecanumStrafe();

        } else {

            setPowerForTankDrive();
        }

        if (collectionIn()) {

            collectionIntake();

        } else if (collectionOut()) {

            collectionRelease();
        }

        if (extendLeftBeaconBumper()) {

            leftBeaconServoOut();

        } else if (resetLeftBeaconBumper()) {

            leftBeaconServoIn();
        }

        if (extendRightBeaconBumper()) {

            rightBeaconServoOut();

        } else if (resetRightBeaconBumper()) {

            rightBeaconServoIn();
        }
        if (gamepad1.right_bumper && (time.time() > DELAY)){
            reversed = !reversed;
            time.reset();
        }

        clipPowerLevels();
        setMotorPowerLevels(powerLevels);
        telemetry.addData("FrontRight: ", powerLevels.frontRightPower);
        telemetry.addData("FrontLeft: ", powerLevels.frontLeftPower);
        telemetry.addData("BackRight: ", powerLevels.backRightPower);
        telemetry.addData("BackLeft: ", powerLevels.backLeftPower);
    }

    private boolean isStrafing() {

        return gamepad1.left_bumper || gamepad1.right_bumper;
    }

    private boolean bIsPressed() {

        return gamepad1.b;
    }

    private boolean aIsPressed() {
        return gamepad1.a;
    }

    private boolean collectionIn() {
        return gamepad2.right_bumper;
    }

    private boolean collectionOut() {
        return gamepad2.left_bumper;
    }

    private boolean extendLeftBeaconBumper() {
        return gamepad2.x;
    }

    private boolean extendRightBeaconBumper() {
        return gamepad2.a;
    }

    private boolean resetLeftBeaconBumper() {
        return gamepad2.y;
    }

    private boolean resetRightBeaconBumper() {
        return gamepad2.b;
    }
}