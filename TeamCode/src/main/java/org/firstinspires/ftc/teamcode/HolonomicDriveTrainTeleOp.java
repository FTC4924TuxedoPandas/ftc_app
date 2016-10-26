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

        } else {

            collectionOff();
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

    private boolean d2XIsPressed() {
        return gamepad2.x;
    }

    private boolean d2AIsPressed() {
        return gamepad2.a;
    }

    private boolean d2YIsPressed() {
        return gamepad2.y;
    }

    private boolean d2BIsPressed() {
        return gamepad2.b;
    }
}