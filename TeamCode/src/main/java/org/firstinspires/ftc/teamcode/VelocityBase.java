package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
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
    public int currentPathSegmentIndex = 0;
    DrivePathSegment segment;
    public EncoderTargets zeroEncoderTargets = new EncoderTargets(0, 0);
    EncoderTargets currentEncoderTargets = zeroEncoderTargets;
    public DrivePathSegment[] currentPath;
    double countsPerInch;
    public ElapsedTime elapsedTimeForCurrentSegment = new ElapsedTime();
    int turnStartValueLeft;
    int turnStartValueRight;
    GyroSensor turningGyro;

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

        turningGyro = hardwareMap.gyroSensor.get("gyroSensor");
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

    protected void StopMovingThrowingArm() {
        throwingArmPowerLevel = 0.0f;
    }

    protected void LowerThrowingArm() {
        throwingArmPowerLevel = -ARM_POWER/10.0f;
    }

    protected void RaiseThrowingArm() {
        throwingArmPowerLevel = ARM_POWER;
    }

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

                    segment.rightPower = 0.0f;

                } else {

                    segment.leftPower = 0.0f;
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

            FourWheelDrivePowerLevels powerLevels =
                    new FourWheelDrivePowerLevels(segment.leftPower, segment.rightPower);
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
    }

    public void useRunUsingEncoders() {

        frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void SetDriveMotorPowerLevels(FourWheelDrivePowerLevels levels) {

        frontRightMotor.setPower(levels.frontRight);
        frontLeftMotor.setPower(levels.frontLeft);
    }

    public void addEncoderTarget(int leftEncoderAdder, int rightEncoderAdder) {

        currentEncoderTargets.LeftTarget += leftEncoderAdder;
        currentEncoderTargets.RightTarget += rightEncoderAdder;
    }

    private boolean counterclockwiseTurnNeeded(double currentAngle) {

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
}