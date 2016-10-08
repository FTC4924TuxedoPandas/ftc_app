package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;

import static android.R.attr.name;

/**
 * Created by 4924_Users on 10/8/2016.
 */

@TeleOp(name = "HolonomicDriveTrainTeleOp")
public class HolonomicDriveTrainTeleOp extends VelocityBase {




    @Override
    public void loop() {

        if (gamepad1.a) {

            throwingArmPowerLevel = ARM_POWER;

        } else if (gamepad1.b) {

           throwingArmPowerLevel = -ARM_POWER;

        } else {

            throwingArmPowerLevel = 0.0f;
        }

        isStrafingLeft = gamepad1.left_bumper;
        isStrafingRight = gamepad1.right_bumper;

        if (isStrafingLeft || isStrafingRight) {

            setPowerForMecanumStrafe();

        } else {

            setPowerForTankDrive();
        }

        clipPowerLevels();
        setMotorPowerLevels(powerLevels);
    }


}