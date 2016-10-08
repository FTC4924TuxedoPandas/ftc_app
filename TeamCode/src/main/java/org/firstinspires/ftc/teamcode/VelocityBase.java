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


public abstract class VelocityBase extends OpMode {

    DcMotor frontRightMotor;
    DcMotor frontLeftMotor;
    DcMotor backRightMotor;
    DcMotor backLeftMotor;
    DcMotor throwingArm;

    boolean isStrafingLeft = false;
    boolean isStrafingRight = false;

    PowerLevels powerLevels = new PowerLevels();
    float throwingArmPowerLevel = 0.0f;

    public final float ARM_POWER = 1.0f;
    public final float BASE_HOLONOMIC_DRIVE_POWER = 0.5f;

    @Override
    public void init() {

        frontRightMotor = hardwareMap.dcMotor.get("frontRightMotor");
        frontLeftMotor = hardwareMap.dcMotor.get("frontLeftMotor");
        backRightMotor = hardwareMap.dcMotor.get("backRightMotor");
        backLeftMotor = hardwareMap.dcMotor.get("backLeftMotor");
        throwingArm = hardwareMap.dcMotor.get("throwingArm");

        frontRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        throwingArm.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    @Override
    public void init_loop() {

    }

    @Override
    public void start() {

    }

    @Override
    public void loop() {


    }

    public void setPowerForTankDrive() {
        float leftStick = 0.0f;
        float rightStick = 0.0f;

        leftStick = Range.clip(gamepad1.left_stick_y,-1.0f,1.0f);
        rightStick = Range.clip(gamepad1.right_stick_y,-1.0f,1.0f);

        powerLevels.frontLeftPower = leftStick;
        powerLevels.backLeftPower = leftStick;
        powerLevels.backRightPower = rightStick;
        powerLevels.frontRightPower = rightStick;
    }

    public void setPowerForMecanumStrafe() {

        if (isStrafingLeft) {

            powerLevels.frontLeftPower = BASE_HOLONOMIC_DRIVE_POWER;
            powerLevels.backLeftPower = -BASE_HOLONOMIC_DRIVE_POWER;
            powerLevels.backRightPower = BASE_HOLONOMIC_DRIVE_POWER;
            powerLevels.frontRightPower = -BASE_HOLONOMIC_DRIVE_POWER;

        } else if (isStrafingRight) {

            powerLevels.frontLeftPower = -BASE_HOLONOMIC_DRIVE_POWER;
            powerLevels.backLeftPower = BASE_HOLONOMIC_DRIVE_POWER;
            powerLevels.backRightPower = -BASE_HOLONOMIC_DRIVE_POWER;
            powerLevels.frontRightPower = BASE_HOLONOMIC_DRIVE_POWER;
        }
    }

    public void clipPowerLevels() {

        throwingArmPowerLevel = Range.clip (throwingArmPowerLevel,-1.0f,1.0f);
        powerLevels.backRightPower = Range.clip(powerLevels.backRightPower, -1.0f, 1.0f);
        powerLevels.backLeftPower = Range.clip(powerLevels.backLeftPower, -1.0f, 1.0f);
        powerLevels.frontRightPower = Range.clip(powerLevels.frontRightPower, -1.0f, 1.0f);
        powerLevels.frontLeftPower = Range.clip(powerLevels.frontLeftPower, -1.0f, 1.0f);
    }

    public void setMotorPowerLevels(PowerLevels PowerLevels) {

        throwingArm.setPower(throwingArmPowerLevel);
        frontLeftMotor.setPower(PowerLevels.frontLeftPower);
        backLeftMotor.setPower(PowerLevels.backLeftPower);
        backRightMotor.setPower(PowerLevels.backRightPower);
        frontRightMotor.setPower(PowerLevels.frontRightPower);
    }
}