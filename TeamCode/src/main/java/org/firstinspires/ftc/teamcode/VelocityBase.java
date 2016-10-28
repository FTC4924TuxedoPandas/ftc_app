package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import static android.R.attr.name;

/**
 * Created by 4924_Users on 10/8/2016.
 */


public abstract class VelocityBase extends OpMode {

    public enum State {

        STATE_INITIAL,
        STATE_DRIVE,
        STATE_STOP,
        STATE_LAUNCH_BALL,
        STATE_POSITION_FOR_BALL,
        STATE_KNOCKCAP
    }

    DcMotor frontRightMotor;
    DcMotor frontLeftMotor;
    DcMotor backRightMotor;
    DcMotor backLeftMotor;
    DcMotor throwingArm;
    DcMotor collectionMotor;

    Servo leftBeaconServo;
    Servo rightBeaconServo;

    boolean isStrafingLeft = false;
    boolean isStrafingRight = false;

    PowerLevels powerLevels = new PowerLevels(0.0f, 0.0f, 0.0f, 0.0f);
    float throwingArmPowerLevel = 0.0f;
    float collectionPowerLevel = 0.0f;

    public final float ARM_POWER = 1.0f;
    public final float COLLECTION_POWER = 0.5f;
    public final float BASE_HOLONOMIC_DRIVE_POWER = 0.5f;
    public int currentPathSegmentIndex = 0;
    DrivePathSegment segment;
    public EncoderTargets zeroEncoderTargets = new EncoderTargets(0, 0);
    EncoderTargets currentEncoderTargets = zeroEncoderTargets;

    public final float BEACON_SERVO_POSITION_OUT = 0.5f;
    public final float BEACON_SERVO_POSITION_IN = 0.0f;

    public DrivePathSegment[] currentPath = new DrivePathSegment[] {

            new DrivePathSegment(0.0f, 0.0f, 0.0f),
    };

    public DrivePathSegment[] launchPositioningPath = new DrivePathSegment[] {

            new DrivePathSegment(0.0f, 0.0f, 0.0f),
    };

    public DrivePathSegment[] beaconPath = new DrivePathSegment[] {

            new DrivePathSegment(0.0f, 0.0f, 0.0f),
    };

    public DrivePathSegment[] knockCapBallPath = new DrivePathSegment[] {

            new DrivePathSegment(0.0f, 0.0f, 0.0f),
    };

    double countsPerInch;
    public ElapsedTime elapsedTimeForCurrentSegment = new ElapsedTime();
    public ElapsedTime elapsedTimeForCurrentState = new ElapsedTime();
    int turnStartValueLeft;
    int turnStartValueRight;
    GyroSensor turningGyro;
    public State currentState;
    static final float TURNING_ANGLE_MARGIN = 2.0f;
    static final int ENCODER_TARGET_MARGIN = 10;
    public PowerLevels zeroPowerLevels = new PowerLevels(0.0f, 0.0f, 0.0f, 0.0f);
    final int COUNTS_PER_REVOLUTION = 1120;
    final double WHEEL_DIAMETER = 4.0f;
    final double GEAR_RATIO = 1.0f;
    final double CALIBRATION_FACTOR = 1.93f;

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

        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        frontLeftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        throwingArm.setDirection(DcMotorSimple.Direction.FORWARD);
        collectionMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        turningGyro = hardwareMap.gyroSensor.get("gyroSensor");
        currentState = State.STATE_INITIAL;

        runWithoutEncoders();
        countsPerInch = (COUNTS_PER_REVOLUTION / (Math.PI * WHEEL_DIAMETER)) * GEAR_RATIO * CALIBRATION_FACTOR;
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

        leftStick = Range.clip(-gamepad1.left_stick_y,-1.0f,1.0f);
        rightStick = Range.clip(-gamepad1.right_stick_y,-1.0f,1.0f);

        powerLevels.frontLeftPower = leftStick;
        powerLevels.backLeftPower = leftStick;
        powerLevels.backRightPower = rightStick;
        powerLevels.frontRightPower = rightStick;
    }

    public void setPowerForMecanumStrafe() {

        if (isStrafingRight) {

            powerLevels.frontLeftPower = BASE_HOLONOMIC_DRIVE_POWER;
            powerLevels.backLeftPower = -BASE_HOLONOMIC_DRIVE_POWER;
            powerLevels.backRightPower = BASE_HOLONOMIC_DRIVE_POWER;
            powerLevels.frontRightPower = -BASE_HOLONOMIC_DRIVE_POWER;

        } else if (isStrafingLeft) {

            powerLevels.frontLeftPower = -BASE_HOLONOMIC_DRIVE_POWER;
            powerLevels.backLeftPower = BASE_HOLONOMIC_DRIVE_POWER;
            powerLevels.backRightPower = -BASE_HOLONOMIC_DRIVE_POWER;
            powerLevels.frontRightPower = BASE_HOLONOMIC_DRIVE_POWER;
        }
    }

    public void clipPowerLevels() {

        powerLevels.backRightPower = Range.clip(powerLevels.backRightPower, -1.0f, 1.0f);
        powerLevels.backLeftPower = Range.clip(powerLevels.backLeftPower, -1.0f, 1.0f);
        powerLevels.frontRightPower = Range.clip(powerLevels.frontRightPower, -1.0f, 1.0f);
        powerLevels.frontLeftPower = Range.clip(powerLevels.frontLeftPower, -1.0f, 1.0f);
        throwingArmPowerLevel = Range.clip (throwingArmPowerLevel,-1.0f,1.0f);
        collectionPowerLevel = Range.clip (collectionPowerLevel,-1.0f,1.0f);
    }

    public void setMotorPowerLevels(PowerLevels powerLevels) {

        frontLeftMotor.setPower(powerLevels.frontLeftPower);
        backLeftMotor.setPower(powerLevels.backLeftPower);
        backRightMotor.setPower(powerLevels.backRightPower);
        frontRightMotor.setPower(powerLevels.frontRightPower);
        throwingArm.setPower(throwingArmPowerLevel);
        collectionMotor.setPower(collectionPowerLevel);
    }

    protected void StopMovingThrowingArm() {
        throwingArmPowerLevel = 0.0f;
    }
        //stops the throwing arm motor for the throwing arm (sets power to 0)
    protected void LowerThrowingArm() {
        throwingArmPowerLevel = -ARM_POWER/10.0f;
    }
        //lowers the throwing arm motor (set to negative, 1/10 of raising power)
    protected void RaiseThrowingArm() {
        throwingArmPowerLevel = ARM_POWER;
    }

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
    public void startSeg() {

        segment = currentPath[currentPathSegmentIndex];
        elapsedTimeForCurrentSegment.reset();

        if (currentPath != null) {

            if (segment.isTurn) {

                turnStartValueLeft = getLeftPosition();
                turnStartValueRight = getRightPosition();

                runWithoutEncoders();
                double currentAngle = turningGyro.getHeading();

                if (counterclockwiseTurnNeeded(currentAngle)) {

                    segment.rightPower = -segment.leftPower;

                } else {

                    segment.leftPower = -segment.rightPower;
                }

            } else {

                if (segment.isDelay) {

                    runWithoutEncoders();
                    segment.leftPower = 0.0f;
                    segment.rightPower = 0.0f;

                } else {

                    int moveCounts = (int) (segment.LeftSideDistance * countsPerInch);

                    useRunUsingEncoders();
                    addEncoderTarget(moveCounts, moveCounts);

                    if (moveCounts < 0) {

                        segment.leftPower *= -1;
                        segment.rightPower *= -1;
                    }
                }
            }

            PowerLevels powerLevels =
                    new PowerLevels(segment.leftPower, segment.rightPower, segment.leftPower, segment.rightPower);
            SetDriveMotorPowerLevels(powerLevels);

            currentPathSegmentIndex++;
        }
    }

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

    public void useRunUsingEncoders() {

        frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void SetDriveMotorPowerLevels(PowerLevels levels) {

        frontRightMotor.setPower(levels.frontRightPower);
        frontLeftMotor.setPower(levels.frontLeftPower);
        backRightMotor.setPower(levels.backRightPower);
        backLeftMotor.setPower(levels.backLeftPower);
    }

    public void addEncoderTarget(int leftEncoderAdder, int rightEncoderAdder) {

        currentEncoderTargets.frontLeftTarget += leftEncoderAdder;
        currentEncoderTargets.frontRightTarget += rightEncoderAdder;
        currentEncoderTargets.backLeftTarget += leftEncoderAdder;
        currentEncoderTargets.backRightTarget += rightEncoderAdder;
    }

    public boolean counterclockwiseTurnNeeded(double currentAngle) {

        telemetry.addData("Angle: ", currentAngle);

        if (currentAngle < Math.abs(segment.Angle)) {

            return (Math.abs(segment.Angle) - currentAngle) >= 180.0f;
        }

        return (currentAngle - Math.abs(segment.Angle)) <= 180.0f;
    }

    public int getRightPosition() {

        return frontRightMotor.getCurrentPosition();
    }

    public int getLeftPosition() {

        return frontLeftMotor.getCurrentPosition();
    }

    public void startPath(DrivePathSegment[] path) {

        currentPath = path;
        currentPathSegmentIndex = 0;
        setEncoderTargetsToCurrentPosition();
        useRunUsingEncoders();
        startSeg();
    }

    public void setEncoderTargetsToCurrentPosition() {

        currentEncoderTargets.frontLeftTarget = getLeftPosition();
        currentEncoderTargets.frontRightTarget = getRightPosition();
        currentEncoderTargets.backLeftTarget = getLeftPosition();
        currentEncoderTargets.backRightTarget = getRightPosition();
    }

    public boolean pathComplete() {
        // Wait for this Segement to end and then see what's next.
        if (segmentComplete()) {
            // Start next Segment if there is one.
            if (currentPathSegmentIndex < currentPath.length) {

                TurnOffAllDriveMotors();
                startSeg();

            } else {

                currentPath = null;
                currentPathSegmentIndex = 0;
                TurnOffAllDriveMotors();
                return true;
            }
        }

        return false;
    }

    public boolean segmentComplete() {

        if (segment.isTurn) {

            return turnComplete();

        } else {

            if (segment.isDelay) {

                return delayComplete();

            } else {

                return linearMoveComplete();
            }
        }
    }

    public boolean turnComplete() {

        return Math.abs(segment.Angle) <= turningGyro.getHeading() + TURNING_ANGLE_MARGIN &&
                Math.abs(segment.Angle) >= turningGyro.getHeading() - TURNING_ANGLE_MARGIN;
    }

    public boolean delayComplete() {

        return elapsedTimeForCurrentSegment.time() >= segment.delayTime;
    }

    public boolean linearMoveComplete() {

        int leftPosition = getLeftPosition();
        int leftTarget = currentEncoderTargets.frontLeftTarget;
        int rightPosition = getRightPosition();
        int rightTarget = currentEncoderTargets.frontRightTarget;

        return (isPositionClose(leftPosition, leftTarget, segment.LeftSideDistance) &&
                isPositionClose(rightPosition, rightTarget, segment.LeftSideDistance)) ||
                (isPastTarget(leftPosition, leftTarget, segment.LeftSideDistance) &&
                        isPastTarget(rightPosition, rightTarget, segment.LeftSideDistance));
    }

    public boolean isPositionClose(int position, int target, float distanceToMove) {

        if (distanceToMove < 0) {

            return position - target < ENCODER_TARGET_MARGIN;
        }

        return target - position < ENCODER_TARGET_MARGIN;
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

    public void throwBall(ElapsedTime ElapsedThrowingTime, float throwingTime) {

        if (ElapsedThrowingTime.time() >= throwingTime) {

            lowerAutoThrowingArm(ElapsedThrowingTime, throwingTime);

        } else {

            throwingArm.setPower(ARM_POWER);
        }
    }

    public void lowerAutoThrowingArm(ElapsedTime ElapsedThrowingTime, float throwingTime) {

        if (ElapsedThrowingTime.time() >= throwingTime * 2) {

            throwingArm.setPower(0.0f);

        } else {

            throwingArm.setPower(-ARM_POWER);
        }
    }

    public void leftBeaconServoOut() {

        leftBeaconServo.setPosition(BEACON_SERVO_POSITION_OUT);
    }

    public void rightBeaconServoOut() {

        leftBeaconServo.setPosition(BEACON_SERVO_POSITION_OUT);
    }

    public void leftBeaconServoIn() {

        leftBeaconServo.setPosition(BEACON_SERVO_POSITION_IN);
    }

    public void rightBeaconServoIn() {

        rightBeaconServo.setPosition(BEACON_SERVO_POSITION_IN);
    }
}