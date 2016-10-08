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

        clipPowerLevels();
        setMotorPowerLevels(powerLevels);
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


}