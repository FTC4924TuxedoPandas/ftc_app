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


public abstract class VelocityBase extends OpMode {

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
    float linearSlidePowerLevel = 0.0f;

    public final float ARM_POWER = 1.0f;
    public final float COLLECTION_POWER = 0.5f;
    public final float BASE_HOLONOMIC_DRIVE_POWER = 0.5f;

    public final float BEACON_SERVO_POSITION_IN = 0.3f;
    public final float BEACON_SERVO_POSITION_OUT = 0.7f;

    public final float GATE_SERVO_POSITION_CLOSED = 0.0f;
    public final float GATE_SERVO_POSITION_LOW = 0.6f;
    public final float GATE_SERVO_POSITION_HIGH = 1.0f;

    public final float LOCK_SERVO_POSITION_CLOSED = 0.0f;
    public final float LOCK_SERVO_POSITION_OPEN = 0.5f;

    public ElapsedTime buttonDelay = new ElapsedTime();
    int turnStartValueLeft;
    int turnStartValueRight;
    int driveDirection;
    GyroSensor turningGyro;
    static final float DELAY = 1.0f;
    public float driveStickX = 0.0f;
    public float driveStickY = 0.0f;
    public float leftBeaconServoPosition = BEACON_SERVO_POSITION_IN;
    public float rightBeaconServoPosition = BEACON_SERVO_POSITION_IN;
    public float gateServoPosition = GATE_SERVO_POSITION_CLOSED;
    public float shovelLockServoPosition = LOCK_SERVO_POSITION_CLOSED;
    boolean reversed;
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
        autonomousBallServo = hardwareMap.servo.get("autonomousBallServo");

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
        collectionGateServo.setPosition(GATE_SERVO_POSITION_LOW);
        shovelLockServo.setPosition(LOCK_SERVO_POSITION_CLOSED);
        autonomousBallServo.setPosition(0.0f);
    }

    @Override
    public void loop() {


    }

    public void setPowerForTankDrive() {

        float leftStick = 0.0f;
        float rightStick = 0.0f;

        leftStick = Range.clip(-gamepad1.left_stick_y,-1.0f,1.0f);
        rightStick = Range.clip(-gamepad1.right_stick_y,-1.0f,1.0f);

        powerLevels.frontLeftPower = leftStick;
        powerLevels.backLeftPower = leftStick;
        powerLevels.backRightPower = rightStick;
        powerLevels.frontRightPower = rightStick;
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

            powerLevels.frontLeftPower = power - Math.abs(headingDifference / 10);
            powerLevels.backLeftPower = -power - Math.abs(headingDifference  / 10);
            powerLevels.backRightPower = power;
            powerLevels.frontRightPower = -power;

        } else {

            powerLevels.frontLeftPower = power;
            powerLevels.backLeftPower = -power;
            powerLevels.backRightPower = power - Math.abs(headingDifference / 10);
            powerLevels.frontRightPower = -power - Math.abs(headingDifference / 10);
        }
    }

    public void setPowerForLinearMove(float power) {

        powerLevels.frontLeftPower = power;
        powerLevels.backLeftPower = power;
        powerLevels.backRightPower = power;
        powerLevels.frontRightPower = power;
    }

    public void setPowerForDiagonalMove(float power) {

        if (driveStickX > 0.0f && driveStickY > 0.0f || driveStickX < 0.0f && driveStickY < 0.0f) {

            if (driveStickY > 0.0f) {

                powerLevels.frontLeftPower = 0.0f;
                powerLevels.backLeftPower = -power;
                powerLevels.backRightPower = 0.0f;
                powerLevels.frontRightPower = -power;

            } else {

                powerLevels.frontLeftPower = 0.0f;
                powerLevels.backLeftPower = power;
                powerLevels.backRightPower = 0.0f;
                powerLevels.frontRightPower = power;
            }

        } else {

            if (driveStickY > 0.0f) {

                powerLevels.frontLeftPower = -power;
                powerLevels.backLeftPower = 0.0f;
                powerLevels.backRightPower = -power;
                powerLevels.frontRightPower = 0.0f;

            } else {

                powerLevels.frontLeftPower = power;
                powerLevels.backLeftPower = 0.0f;
                powerLevels.backRightPower = power;
                powerLevels.frontRightPower = 0.0f;
            }
        }
    }

    public void setPowerForTurning(float power) {

        powerLevels.frontLeftPower = -power;
        powerLevels.backLeftPower = -power;
        powerLevels.backRightPower = power;
        powerLevels.frontRightPower = power;
    }

    public void clipPowerLevels() {

        powerLevels.backRightPower = Range.clip(powerLevels.backRightPower, -1.0f, 1.0f);
        powerLevels.backLeftPower = Range.clip(powerLevels.backLeftPower, -1.0f, 1.0f);
        powerLevels.frontRightPower = Range.clip(powerLevels.frontRightPower, -1.0f, 1.0f);
        powerLevels.frontLeftPower = Range.clip(powerLevels.frontLeftPower, -1.0f, 1.0f);
        throwingArmPowerLevel = Range.clip(throwingArmPowerLevel, -1.0f, 1.0f);
        collectionPowerLevel = Range.clip(collectionPowerLevel, -1.0f, 1.0f);
    }

    public void setMotorPowerLevels(PowerLevels powerLevels) {

        frontLeftMotor.setPower(powerLevels.frontLeftPower);
        backLeftMotor.setPower(powerLevels.backLeftPower);
        backRightMotor.setPower(powerLevels.backRightPower);
        frontRightMotor.setPower(powerLevels.frontRightPower);
        throwingArm.setPower(throwingArmPowerLevel);
        collectionMotor.setPower(collectionPowerLevel);
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

    protected void collectionOff() {
        collectionPowerLevel = 0.0f;
    }

        //raises (to shoot) the throwing arm motor (high positive power)


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

    public void TurnOffAllDriveMotors() {

        powerLevels = new PowerLevels(0.0f, 0.0f, 0.0f, 0.0f);
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

    public void openGateLow() {

        gateServoPosition = GATE_SERVO_POSITION_LOW;
    }

    public void openGateHigh() {

        gateServoPosition = GATE_SERVO_POSITION_HIGH;
    }

    public void unlockShovel() {

        shovelLockServoPosition = LOCK_SERVO_POSITION_OPEN;
    }

    public void lockShovel() {

        shovelLockServoPosition = LOCK_SERVO_POSITION_CLOSED;
    }
}

