package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

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
    DcMotor winchMotorOne;
    DcMotor winchMotorTwo;

    Servo leftBeaconServo;
    Servo rightBeaconServo;
    Servo collectionGateServo;
    Servo shovelLockServo;
    Servo autonomousBallServo;
    Servo capBallServo;
    Servo ballClampServo;

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

    public final float GATE_SERVO_POSITION_CLOSED = 0.03f;
    public final float GATE_SERVO_POSITION_OPEN = 0.6f;

    public final float CAP_BALL_SERVO_POSITION_STOPPED = 0.5f;
    public final float CAP_BALL_SERVO_POSITION_FWD = 1.0f;
    public final float CAP_BALL_SERVO_POSITION_BKWD = 0.0f;

    public final float LOCK_SERVO_POSITION_CLOSED = 1.0f;
    public final float LOCK_SERVO_POSITION_OPEN = 0.35f;

    public final float SPINNING_SERVO_POSITION_LEFT = -0.6f;
    public final float SPINNING_SERVO_POSITION_RIGHT = 0.6f;
    public final float SPINNING_SERVO_POSITION_STOP = 0.5f;

    public final float BALL_CLAMP_SERVO_POSITION_FWD = 0.0f;
    public final float BALL_CLAMP_SERVO_POSITION_STOPPED = 0.5f;
    public final float BALL_CLAMP_SERVO_POSITION_BKWD = 1.0f;

    int turnStartValueLeft;
    int turnStartValueRight;
    int driveDirection;
    GyroSensor turningGyro;
    TouchSensor rightBumper;
    TouchSensor leftBumper;
    public PowerLevels zeroPowerLevels = new PowerLevels(0.0f, 0.0f, 0.0f, 0.0f);
    public float leftBeaconServoPosition = BEACON_SERVO_POSITION_IN;
    public float rightBeaconServoPosition = BEACON_SERVO_POSITION_IN;
    public float gateServoPosition = GATE_SERVO_POSITION_CLOSED;
    public float shovelLockServoPosition = LOCK_SERVO_POSITION_CLOSED;
    public float spinningServoPosition = SPINNING_SERVO_POSITION_STOP;
    public float capBallServoPosition = CAP_BALL_SERVO_POSITION_STOPPED;
    public float ballClampServoPosition = BALL_CLAMP_SERVO_POSITION_STOPPED;
    public static int angleOffset = 0;
    ElapsedTime time = new ElapsedTime();

    @Override
    public void init() {

        frontRightMotor = hardwareMap.dcMotor.get("frontRightMotor");
        frontLeftMotor = hardwareMap.dcMotor.get("frontLeftMotor");
        backRightMotor = hardwareMap.dcMotor.get("backRightMotor");
        backLeftMotor = hardwareMap.dcMotor.get("backLeftMotor");
        throwingArm = hardwareMap.dcMotor.get("throwingArm");
        collectionMotor = hardwareMap.dcMotor.get("collectionMotor");
        winchMotorOne = hardwareMap.dcMotor.get("winchMotorOne");
        winchMotorTwo = hardwareMap.dcMotor.get("winchMotorTwo");

        leftBeaconServo = hardwareMap.servo.get("leftBeaconServo");
        rightBeaconServo = hardwareMap.servo.get("rightBeaconServo");
        collectionGateServo = hardwareMap.servo.get("collectionGateServo");
        shovelLockServo = hardwareMap.servo.get("shovelLockServo");
        capBallServo = hardwareMap.servo.get("capBallServo");
        ballClampServo = hardwareMap.servo.get("ballClampServo");

        lineSensor = hardwareMap.opticalDistanceSensor.get("lineSensor");
        rightBeaconSensor = hardwareMap.colorSensor.get("rightBeaconSensor");
        leftBeaconSensor = hardwareMap.colorSensor.get("leftBeaconSensor");
        //rightBumper = hardwareMap.touchSensor.get("rightBumper");
        //leftBumper = hardwareMap.touchSensor.get("leftBumper");

        frontRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        throwingArm.setDirection(DcMotorSimple.Direction.FORWARD);
        collectionMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        winchMotorOne.setDirection(DcMotorSimple.Direction.FORWARD);
        winchMotorTwo.setDirection(DcMotorSimple.Direction.FORWARD);

        turningGyro = hardwareMap.gyroSensor.get("gyroSensor");

        ballClampServo.setPosition(BALL_CLAMP_SERVO_POSITION_STOPPED);

        runWithoutEncoders();
        rightBeaconSensor.enableLed(false);
        leftBeaconSensor.enableLed(false);
        turningGyro.calibrate();
    }

    @Override
    public void init_loop() {

        ballClampServo.setPosition(BALL_CLAMP_SERVO_POSITION_STOPPED);
        rightBeaconServo.setPosition(BEACON_SERVO_POSITION_IN);
        leftBeaconServo.setPosition(BEACON_SERVO_POSITION_IN);
        collectionGateServo.setPosition(GATE_SERVO_POSITION_OPEN);
        shovelLockServo.setPosition(1.0f);
        autonomousBallServo.setPosition(0.0f);
        capBallServo.setPosition(CAP_BALL_SERVO_POSITION_STOPPED);
    }

    @Override
    public void start() {

        turningGyro.resetZAxisIntegrator();
        angleOffset = turningGyro.getHeading();
    }

    @Override
    public void loop() {

    }

    @Override
    public void stop() {

        leftBeaconServoIn();
        rightBeaconServoIn();
        capBallServoStop();
        ballClampServoStop();
    }

    public void setPowerForLinearMove(float power) {

        powerLevels.frontLeftPower = power;
        powerLevels.backLeftPower = power;
        powerLevels.backRightPower = power;
        powerLevels.frontRightPower = power;
    }

    public void setMotorPowerLevels(PowerLevels powerLevels) {

        frontLeftMotor.setPower(powerLevels.frontLeftPower);
        backLeftMotor.setPower(powerLevels.backLeftPower);
        backRightMotor.setPower(powerLevels.backRightPower);
        frontRightMotor.setPower(powerLevels.frontRightPower);
        throwingArm.setPower(throwingArmPowerLevel);
        collectionMotor.setPower(collectionPowerLevel);
        winchMotorOne.setPower(winchPowerLevel);
        winchMotorTwo.setPower(winchPowerLevel);
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
/*
    public int getRightPosition() {

        return frontRightMotor.getCurrentPosition();
    }

    public int getLeftPosition() {

        return frontLeftMotor.getCurrentPosition();
    }
*/
    public boolean isPastTarget(int position, int target, float distanceToMove) {

        if (distanceToMove < 0) {

            return position < target;
        }

        return position > target;
    }

    public void TurnOffAllDriveMotors() {
        SetDriveMotorPowerLevels(zeroPowerLevels);
        powerLevels.backLeftPower = 0.0f;
        powerLevels.frontLeftPower = 0.0f;
        powerLevels.backRightPower = 0.0f;
        powerLevels.frontRightPower = 0.0f;
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

    public void capBallServoStop() {

        capBallServoPosition = CAP_BALL_SERVO_POSITION_STOPPED;
    }

    public void capBallServoFwd() {

        capBallServoPosition = CAP_BALL_SERVO_POSITION_FWD;
    }

    public void capBallServoBkwd() {

        capBallServoPosition = CAP_BALL_SERVO_POSITION_BKWD;
    }

    public void ballClampServoStop() {

        ballClampServoPosition = BALL_CLAMP_SERVO_POSITION_STOPPED;
    }

    public void ballClampServoFwd() {

        ballClampServoPosition = BALL_CLAMP_SERVO_POSITION_FWD;
    }

    public void ballClampServoBkwd() {

        ballClampServoPosition = BALL_CLAMP_SERVO_POSITION_BKWD;
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

    public void spinLeft () {

        spinningServoPosition = SPINNING_SERVO_POSITION_LEFT;
    }

    public void spinRight() {

        spinningServoPosition = SPINNING_SERVO_POSITION_RIGHT;
    }

    public void spinStop() {

        spinningServoPosition = SPINNING_SERVO_POSITION_STOP;
    }

    public boolean d2RightStickIsLeft() {return gamepad2.right_stick_x < -0.5f;}

    public boolean d2RightStickIsRight() {return gamepad2.right_stick_x > 0.5f;}

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

    public boolean d2StartIsPressed() { return gamepad2.start; }

    public boolean d1BackIsPressed() { return gamepad1.back; }

    public float leftTriggerValue() { return gamepad1.left_trigger; }

    public float rightTriggerValue() { return gamepad1.right_trigger; }

    public float d2LeftTriggerValue() { return gamepad2.left_trigger; }

    public float d2RightTriggerValue() { return gamepad2.right_trigger; }

}