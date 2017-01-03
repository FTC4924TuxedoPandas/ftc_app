package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/**
 * Created by 4924_Users on 10/8/2016.
 */

public abstract class RevolutionVelocityBase extends OpMode {

    DcMotor frontRightMotor;
    DcMotor frontLeftMotor;
    DcMotor backRightMotor;
    DcMotor backLeftMotor;
    DcMotor throwingArm;
    DcMotor collectionMotor;
    DcMotor winchMotor;

    Servo leftBeaconServo;
    Servo rightBeaconServo;
    Servo collectionGateServo;
    Servo shovelLockServo;
    Servo autonomousBallServo;

    OpticalDistanceSensor lineSensor;
    ColorSensor rightBeaconSensor;
    ColorSensor leftBeaconSensor;

    boolean isStrafingLeft = false;
    boolean isStrafingRight = false;
    public boolean isTurningLeft = false;
    public boolean isTurningRight = false;
    public boolean headingSet = false;
    public int steadyHeading = 0;

    PowerLevels powerLevels = new PowerLevels(0.0f, 0.0f, 0.0f, 0.0f);
    float throwingArmPowerLevel = 0.0f;
    float collectionPowerLevel = 0.0f;
    float winchPowerLevel = 0.0f;

    public final float ARM_POWER = 1.0f;
    public final float COLLECTION_POWER = 1.0f;

    public final float BEACON_SERVO_POSITION_IN = 0.2f;
    public final float BEACON_SERVO_POSITION_OUT = 0.7f;

    public final float GATE_SERVO_POSITION_CLOSED = 0.0f;
    public final float GATE_SERVO_POSITION_OPEN = 0.6f;

    public final float LOCK_SERVO_POSITION_CLOSED = 1.0f;
    public final float LOCK_SERVO_POSITION_OPEN = 0.5f;

    int turnStartValueLeft;
    int turnStartValueRight;
    int driveDirection;
    float driveCoeff;
    GyroSensor turningGyro;
    public PowerLevels zeroPowerLevels = new PowerLevels(0.0f, 0.0f, 0.0f, 0.0f);
    public float leftBeaconServoPosition = BEACON_SERVO_POSITION_IN;
    public float rightBeaconServoPosition = BEACON_SERVO_POSITION_IN;
    public float gateServoPosition = GATE_SERVO_POSITION_CLOSED;
    public float shovelLockServoPosition = LOCK_SERVO_POSITION_CLOSED;
    public static int angleOffset = 0;
    boolean lowSensitivity;
    boolean highSensitivity;
    ElapsedTime time = new ElapsedTime();

    @Override
    public void init() {

        frontRightMotor = hardwareMap.dcMotor.get("frontRightMotor");
        frontLeftMotor = hardwareMap.dcMotor.get("frontLeftMotor");
        backRightMotor = hardwareMap.dcMotor.get("backRightMotor");
        backLeftMotor = hardwareMap.dcMotor.get("backLeftMotor");
        throwingArm = hardwareMap.dcMotor.get("throwingArm");
        collectionMotor = hardwareMap.dcMotor.get("collectionMotor");

        leftBeaconServo = hardwareMap.servo.get("leftBeaconServo");
        rightBeaconServo = hardwareMap.servo.get("rightBeaconServo");
        collectionGateServo = hardwareMap.servo.get("collectionGateServo");
        shovelLockServo = hardwareMap.servo.get("shovelLockServo");

        lineSensor = hardwareMap.opticalDistanceSensor.get("lineSensor");
        rightBeaconSensor = hardwareMap.colorSensor.get("rightBeaconSensor");
        leftBeaconSensor = hardwareMap.colorSensor.get("leftBeaconSensor");

        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        frontLeftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        throwingArm.setDirection(DcMotorSimple.Direction.FORWARD);
        collectionMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        turningGyro = hardwareMap.gyroSensor.get("gyroSensor");

        runWithoutEncoders();
        turningGyro.calibrate();
    }

    @Override
    public void init_loop() {

        rightBeaconServo.setPosition(BEACON_SERVO_POSITION_IN);
        leftBeaconServo.setPosition(BEACON_SERVO_POSITION_IN);
        collectionGateServo.setPosition(GATE_SERVO_POSITION_OPEN);
        shovelLockServo.setPosition(0.0f);
        autonomousBallServo.setPosition(0.0f);
    }

    @Override
    public void start() {

        angleOffset = turningGyro.getHeading();
    }

    @Override
    public void loop() {


    }

    public void setPowerForMecanumStrafe(float power, int heading) {

        int headingDifference = steadyHeading - heading;

        if (steadyHeading - heading >= 180) {

            headingDifference = steadyHeading - 360 - heading;
        }

        if (heading - steadyHeading >= 180) {

            headingDifference = 360 - heading + steadyHeading;
        }

        if (headingDifference < 0) {

            powerLevels.frontLeftPower = power - Math.abs(headingDifference * power);
            powerLevels.backLeftPower = -power - Math.abs(headingDifference * power);;
            powerLevels.backRightPower = power;
            powerLevels.frontRightPower = -power;

        } else {

            powerLevels.frontLeftPower = power;
            powerLevels.backLeftPower = -power;
            powerLevels.backRightPower = power - Math.abs(headingDifference * power);;
            powerLevels.frontRightPower = -power - Math.abs(headingDifference * power);;
        }
    }

    public void setPowerForLinearMove(float power) {

        powerLevels.frontLeftPower = -power;
        powerLevels.backLeftPower = -power;
        powerLevels.backRightPower = -power;
        powerLevels.frontRightPower = -power;
    }

    public void setMotorPowerLevels(PowerLevels powerLevels) {

        frontLeftMotor.setPower(powerLevels.frontLeftPower);
        backLeftMotor.setPower(powerLevels.backLeftPower);
        backRightMotor.setPower(powerLevels.backRightPower);
        frontRightMotor.setPower(powerLevels.frontRightPower);
        throwingArm.setPower(throwingArmPowerLevel);
        collectionMotor.setPower(collectionPowerLevel);
        winchMotor.setPower(winchPowerLevel);
    }

    protected void stopMovingThrowingArm() { throwingArmPowerLevel = 0.0f; }
        //stops the throwing arm motor for the throwing arm (sets power to 0)
    protected void lowerThrowingArm() {
        throwingArmPowerLevel = -ARM_POWER/10.0f;
    }
        //lowers the throwing arm motor (set to negative, 1/10 of raising power)
    protected void raiseThrowingArm() { throwingArmPowerLevel = ARM_POWER; }

    protected void collectionIntake() {
        collectionPowerLevel = COLLECTION_POWER;
    }

    protected void collectionRelease() {
        collectionPowerLevel = -COLLECTION_POWER;
    }

    protected void collectionOff() {collectionPowerLevel = 0.0f;}

    public void runWithoutEncoders() {

        setDriveMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void setDriveMode(DcMotor.RunMode mode) {

        if (frontLeftMotor.getMode() != mode)
            frontLeftMotor.setMode(mode);
        if (frontRightMotor.getMode() != mode)
            frontRightMotor.setMode(mode);
        if (backLeftMotor.getMode() != mode)
            backLeftMotor.setMode(mode);
        if (backRightMotor.getMode() != mode)
            backRightMotor.setMode(mode);
    }

    public void SetDriveMotorPowerLevels(PowerLevels levels) {

        frontRightMotor.setPower(levels.frontRightPower);
        frontLeftMotor.setPower(levels.frontLeftPower);
        backRightMotor.setPower(levels.backRightPower);
        backLeftMotor.setPower(levels.backLeftPower);
    }

    public int getRightPosition() {

        return frontRightMotor.getCurrentPosition();
    }

    public int getLeftPosition() {

        return frontLeftMotor.getCurrentPosition();
    }

    public boolean isPastTarget(int position, int target, float distanceToMove) {

        if (distanceToMove < 0) {

            return position < target;
        }

        return position > target;
    }

    public void TurnOffAllDriveMotors() {
        SetDriveMotorPowerLevels(zeroPowerLevels);
    }

    public void leftBeaconServoOut() {

        leftBeaconServoPosition = BEACON_SERVO_POSITION_OUT;
    }

    public void rightBeaconServoOut() {

        rightBeaconServoPosition = BEACON_SERVO_POSITION_OUT;
    }

    public void leftBeaconServoIn() {

        leftBeaconServoPosition = BEACON_SERVO_POSITION_IN;
    }

    public void rightBeaconServoIn() {

        rightBeaconServoPosition = BEACON_SERVO_POSITION_IN;
    }

    public void closeGate() {

        gateServoPosition = GATE_SERVO_POSITION_CLOSED;
    }

    public void openGate() {

        gateServoPosition = GATE_SERVO_POSITION_OPEN;
    }

    public void unlockShovel() {

        shovelLockServoPosition = LOCK_SERVO_POSITION_OPEN;
    }

    public void lockShovel() {

        shovelLockServoPosition = LOCK_SERVO_POSITION_CLOSED;
    }

    public boolean d1DPadUpIsPressed() { return gamepad1.dpad_up; }

    public boolean d1DPadDownIsPressed() { return gamepad1.dpad_down; }

    public boolean d2DPadDownIsPressed() { return gamepad2.dpad_down; }

    public boolean d2DPadUpIsPressed() { return gamepad2.dpad_up; }

    public boolean collectionIn() {return gamepad2.right_bumper;}

    public boolean collectionOut() {
        return gamepad2.left_bumper;
    }

    public boolean d2XIsPressed() {return gamepad2.x;}

    public boolean d2AIsPressed() { return gamepad2.a; }

    public boolean d2YIsPressed() { return gamepad2.y; }

    public boolean d1AIsPressed() { return gamepad1.a; }

    public boolean d1BIsPressed() { return gamepad1.b; }

    public boolean d1YIsPressed() { return gamepad1.y; }

    public boolean d1XIsPressed() { return gamepad1.x; }

    public boolean d1DPadLeftIsPressed() { return gamepad1.dpad_left; }

    public boolean d2DPadLeftIsPressed() { return gamepad2.dpad_left; }

    public boolean d2DPadRightIsPressed() { return gamepad2.dpad_right; }

    public boolean d2BIsPressed() { return gamepad2.b; }

    public boolean d1DPadRightIsPressed() { return gamepad1.dpad_right; }

    public boolean d1LeftBumperIsPressed() { return gamepad1.left_bumper; }

    public boolean d1RightBumperIsPressed() { return gamepad1.right_bumper; }

    public boolean d1StartIsPressed() { return gamepad1.start; }

    public float leftTriggerValue() { return gamepad1.left_trigger; }

    public float rightTriggerValue() { return gamepad1.right_trigger; }
}