package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Created by 4924_Users on 10/22/2016.
 */

public abstract class AutonomousBase extends VelocityBase {

    public enum State {

        STATE_INITIAL,
        STATE_DRIVE,
        STATE_STOP,
        STATE_LAUNCH_BALL,
        STATE_POSITION_FOR_BALL,
        STATE_KNOCK_CAP_BALL,
        STATE_FIND_WHITE_LINE,
        STATE_PUSH_BEACON,
        STATE_LOAD_BALL,
        STATE_WAIT_FOR_BALL,
        STATE_START_BEACON_PATH,
        STATE_START_LAUNCH_PATH,
        STATE_START_CAP_BALL_PATH,
        STATE_DROP_GATE,
        STATE_lINE_UP_TO_BEACON
    }

    final float THROWING_TIME = 0.2f;
    public int stateIndex = 0;
    public ElapsedTime elapsedTimeForMove = new ElapsedTime();
    public boolean isPushing = false;
    public boolean isSecondBeacon = false;
    public int currentPathSegmentIndex = 0;
    DrivePathSegment segment = new DrivePathSegment();
    public EncoderTargets zeroEncoderTargets = new EncoderTargets(0, 0);
    EncoderTargets currentEncoderTargets = zeroEncoderTargets;

    public DrivePathSegment[] currentPath = new DrivePathSegment[] {

            new DrivePathSegment(0.0f, 0.0f, DrivePathSegment.LINEAR),
    };

    public DrivePathSegment[] launchPositioningPath = new DrivePathSegment[] {

            new DrivePathSegment(0.0f, 0.0f, DrivePathSegment.LINEAR),
    };

    public DrivePathSegment[] beaconPath = new DrivePathSegment[] {

            new DrivePathSegment(0.0f, 0.0f, DrivePathSegment.LINEAR),
    };

    public DrivePathSegment[] knockCapBallPath = new DrivePathSegment[] {

            new DrivePathSegment(0.0f, 0.0f, DrivePathSegment.LINEAR),
    };

    double countsPerInch;
    public ElapsedTime elapsedTimeForCurrentSegment = new ElapsedTime();
    public ElapsedTime elapsedTimeForCurrentState = new ElapsedTime();
    public State currentState;
    static final float TURNING_ANGLE_MARGIN = 2.0f;
    static final int ENCODER_TARGET_MARGIN = 15;
    final int COUNTS_PER_REVOLUTION = 1120;
    final double WHEEL_DIAMETER = 4.0f;
    final double GEAR_RATIO = 1.0f;
    final double CALIBRATION_FACTOR = 1.93f;
    public static int angleOffset = 0;

    @Override
    public void init() {

        super.init();
        shovelLockServo = hardwareMap.servo.get("shovelLockServo");
        currentState = State.STATE_INITIAL;
        countsPerInch = (COUNTS_PER_REVOLUTION / (Math.PI * WHEEL_DIAMETER)) * GEAR_RATIO * CALIBRATION_FACTOR;
    }

    @Override
    public void init_loop() {

        rightBeaconServo.setPosition(BEACON_SERVO_POSITION_IN);
        leftBeaconServo.setPosition(BEACON_SERVO_POSITION_IN);
        collectionGateServo.setPosition(GATE_SERVO_POSITION_LOW);
        shovelLockServo.setPosition(0.0f);
        autonomousBallServo.setPosition(0.0f);
    }

    @Override
    public void start() {

        angleOffset = turningGyro.getHeading();
    }

    @Override
    public void loop() {

        telemetry.addData("currentState: ", currentState);
        //telemetry.addData("Gyro", turningGyro.getHeading());
        //telemetry.addData("Target", segment.Angle);
        telemetry.addData("F L", powerLevels.frontLeftPower);
        telemetry.addData("F R", powerLevels.frontRightPower);
        telemetry.addData("B L", powerLevels.backLeftPower);
        telemetry.addData("B R", powerLevels.backRightPower);
        int heading = turningGyro.getHeading();

        switch (currentState) {

            case STATE_INITIAL:

                switchToNextState();

                break;

            case STATE_START_LAUNCH_PATH:

                startPath(launchPositioningPath);
                switchToNextState();

                break;

            case STATE_POSITION_FOR_BALL:

                if (pathComplete()) {

                    TurnOffAllDriveMotors();
                    switchToNextState();
                }

                break;

            case STATE_DROP_GATE:

                if (elapsedTimeForCurrentState.time() >= 2.0f) {

                    collectionMotor.setPower(0.0f);
                    switchToNextState();

                } else {

                    collectionMotor.setPower(-1.0f);
                }

                break;

            case STATE_LAUNCH_BALL:

                throwBall(elapsedTimeForCurrentState, THROWING_TIME);

                if (elapsedTimeForCurrentState.time() >= 0.8f) {

                    switchToNextState();
                }

                break;

            case STATE_LOAD_BALL:

                autonomousBallServo.setPosition(1.0f);
                switchToNextState();

                break;

            case STATE_WAIT_FOR_BALL:

                if (elapsedTimeForCurrentState.time() >= 2.0f) {

                    autonomousBallServo.setPosition(0.0f);
                    switchToNextState();
                }

                break;

            case STATE_START_BEACON_PATH:

                startPath(beaconPath);
                switchToNextState();

                break;

            case STATE_DRIVE:

                if (pathComplete()) {

                    TurnOffAllDriveMotors();
                    switchToNextState();
                }

                //telemetry.addData("RightPosition", getRightPosition());
                //telemetry.addData("RightTarget", currentEncoderTargets.frontRightTarget);
                //telemetry.addData("LeftPosition", getLeftPosition());
                //telemetry.addData("LeftTarget", currentEncoderTargets.frontLeftTarget);

                break;

            case STATE_FIND_WHITE_LINE:

                telemetry.addData("LineSensor", lineSensor.getRawLightDetected());

                if (lineSensor.getRawLightDetected() >= 0.5f && elapsedTimeForCurrentState.time() >= 1.0f) {

                    TurnOffAllDriveMotors();
                    switchToNextState();

                } else {

                    /*if (isPushing) {

                        if (elapsedTimeForMove.time() >= 1.0f) {

                            isPushing = !isPushing;
                            elapsedTimeForMove.reset();
                        }

                    } else {

                        if (elapsedTimeForMove.time() >= 5.0f) {

                            isPushing = !isPushing;
                            elapsedTimeForMove.reset();
                        }
                    }*/

                    if (isRed()) {

                        setPowerForMecanumStrafe(0.2f, heading);

                    } else {

                        setPowerForMecanumStrafe(-0.2f, heading);
                    }

                    /*if (isPushing && !isRed()) {

                        powerLevels.frontRightPower= 0.3f;
                        powerLevels.backRightPower= 0.3f;
                        powerLevels.frontLeftPower= 0.2f;
                        powerLevels.backLeftPower= 0.2f;
                    }*/
                }

                break;

            case STATE_lINE_UP_TO_BEACON:

                if (!isRed() && isSecondBeacon) {

                    if (elapsedTimeForCurrentState.time() >= 1.0f) {

                        TurnOffAllDriveMotors();
                        switchToNextState();

                    } else {

                        powerLevels.frontRightPower= 0.3f;
                        powerLevels.backRightPower= 0.3f;
                        powerLevels.frontLeftPower= 0.2f;
                        powerLevels.backLeftPower= 0.2f;
                    }

                } else {

                    switchToNextState();
                }

                break;

            case STATE_PUSH_BEACON:

                if (isRed()) {

                    pushBeaconButton(leftBeaconSensor.red(), rightBeaconSensor.red());

                } else {

                    pushBeaconButton(leftBeaconSensor.blue(), rightBeaconSensor.blue());
                }

                if (isRed()) {

                    if (leftBeaconSensor.red() >= 3 && rightBeaconSensor.red() >= 3) {

                        rightBeaconServoIn();
                        leftBeaconServoIn();
                        TurnOffAllDriveMotors();
                        elapsedTimeForMove.reset();
                        isPushing = false;
                        isSecondBeacon = true;
                        switchToNextState();
                    }

                } else {

                    if (leftBeaconSensor.blue() >= 3 && rightBeaconSensor.blue() >= 3) {

                        rightBeaconServoIn();
                        leftBeaconServoIn();
                        TurnOffAllDriveMotors();
                        elapsedTimeForMove.reset();
                        isPushing = false;
                        isSecondBeacon = true;
                        switchToNextState();
                    }
                }

                break;

            case STATE_START_CAP_BALL_PATH:

                startPath(knockCapBallPath);
                switchToNextState();

                break;

            case STATE_KNOCK_CAP_BALL:

                if (pathComplete()) {

                    TurnOffAllDriveMotors();
                    switchToNextState();
                }

                break;

            case STATE_STOP:

                break;
        }

        rightBeaconServo.setPosition(rightBeaconServoPosition);
        leftBeaconServo.setPosition(leftBeaconServoPosition);
        setMotorPowerLevels(powerLevels);
    }

    public void switchToNextState() {

        elapsedTimeForCurrentState.reset();
        stateIndex++;

        if (stateIndex >= stateList().length) {

            stateIndex = stateList().length - 1;
        }

        if (stateIndex < 0) {

            stateIndex = 0;
        }

        currentState = stateList()[stateIndex];
    }

    public void startSeg() {

        segment = currentPath[currentPathSegmentIndex];
        elapsedTimeForCurrentSegment.reset();

        int heading = turningGyro.getHeading();
        steadyHeading = heading;

        if (currentPath != null) {

            if (segment.isTurn) {

                segment.Angle += angleOffset;

                if (segment.Angle >= 360) {

                    segment.Angle -= 360;
                }

                turnStartValueLeft = getLeftPosition();
                turnStartValueRight = getRightPosition();

                runWithoutEncoders();
                double currentAngle = heading;

                if (counterclockwiseTurnNeeded(currentAngle)) {

                    segment.leftPower = -segment.rightPower;

                } else {

                    segment.rightPower = -segment.leftPower;
                }

                powerLevels = new PowerLevels(segment.leftPower, segment.rightPower, segment.leftPower, segment.rightPower);

            } else {

                if (segment.isDelay) {

                    runWithoutEncoders();
                    segment.leftPower = 0.0f;
                    segment.rightPower = 0.0f;

                    powerLevels = new PowerLevels(segment.leftPower, segment.rightPower, segment.leftPower, segment.rightPower);

                } else {

                    if (segment.isHolonomic) {

                        int moveCounts = (int) (segment.LeftSideDistance * countsPerInch);

                        useRunUsingEncoders();
                        addEncoderTarget(moveCounts, moveCounts);

                        if (segment.RightSideDistance < 0.0f) {

                            isStrafingLeft = true;
                            isStrafingRight = false;

                        } else {

                            isStrafingLeft = false;
                            isStrafingRight = true;
                        }

                        setPowerForMecanumStrafe(segment.rightPower, heading);

                    } else {

                        int moveCounts = (int) (segment.LeftSideDistance * countsPerInch);

                        useRunUsingEncoders();
                        addEncoderTarget(moveCounts, moveCounts);

                        if (moveCounts < 0) {

                            segment.leftPower *= -1;
                            segment.rightPower *= -1;
                        }

                        powerLevels = new PowerLevels(segment.leftPower, segment.rightPower, segment.leftPower, segment.rightPower);
                    }
                }
            }

            // SetDriveMotorPowerLevels(powerLevels);

            currentPathSegmentIndex++;
        }
    }

    public void startPath(DrivePathSegment[] path) {

        currentPath = path;
        currentPathSegmentIndex = 0;
        setEncoderTargetsToCurrentPosition();
        useRunUsingEncoders();
        startSeg();
    }

    public boolean pathComplete() {
        // Wait for this Segment to end and then see what's next.
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

        } else {

            if (!segment.isTurn) {

                if (segment.isHolonomic) {

                    setPowerForMecanumStrafe(segment.rightPower, turningGyro.getHeading());

                } else {

                    if (segment.isDelay) {

                        TurnOffAllDriveMotors();

                    } else {

                        setPowerForLinearMove(segment.rightPower);
                    }
                }
            }
        }

        return false;
    }

    public int getRightPosition() {

        return frontRightMotor.getCurrentPosition();
    }

    public int getLeftPosition() {

        return frontLeftMotor.getCurrentPosition();
    }

    public void setEncoderTargetsToCurrentPosition() {

        currentEncoderTargets.frontLeftTarget = getLeftPosition();
        currentEncoderTargets.frontRightTarget = getRightPosition();
        currentEncoderTargets.backLeftTarget = getLeftPosition();
        currentEncoderTargets.backRightTarget = getRightPosition();
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

        int heading = turningGyro.getHeading();

        return Math.abs(segment.Angle) <= heading + TURNING_ANGLE_MARGIN &&
                Math.abs(segment.Angle) >= heading - TURNING_ANGLE_MARGIN;
    }

    public boolean delayComplete() {

        return elapsedTimeForCurrentSegment.time() >= segment.delayTime;
    }

    public boolean linearMoveComplete() {

        int leftPosition = getLeftPosition();
        int leftTarget = currentEncoderTargets.frontLeftTarget;
        int rightPosition = getRightPosition();
        int rightTarget = currentEncoderTargets.frontRightTarget;

        return (isPositionClose(leftPosition, leftTarget, segment.LeftSideDistance) ||
                isPositionClose(rightPosition, rightTarget, segment.LeftSideDistance)) ||
                (isPastTarget(leftPosition, leftTarget, segment.LeftSideDistance) ||
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

    public void pushBeaconButton(int leftSensorRead, int rightSensorRead) {

        if (rightSensorRead >= 3 && leftSensorRead >= 3) {

            return;

        } else {

            if (rightSensorRead <= 3 && leftSensorRead <= 3) {

                leftBeaconServoOut();

            } else {

                if (rightSensorRead >= 3) {

                    rightBeaconServoOut();

                } else {

                    leftBeaconServoOut();
                }
            }
        }
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

    public void useRunUsingEncoders() {

        frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void throwBall(ElapsedTime ElapsedThrowingTime, float throwingTime) {

        if (ElapsedThrowingTime.time() >= throwingTime) {

            lowerAutoThrowingArm(ElapsedThrowingTime, throwingTime);

        } else {

            throwingArm.setPower(0.9f);
        }
    }

    public void lowerAutoThrowingArm(ElapsedTime ElapsedThrowingTime, float throwingTime) {

        if (ElapsedThrowingTime.time() >= throwingTime * 4) {

            throwingArm.setPower(0.0f);

        } else {

            throwingArm.setPower(-0.9f);
        }
    }

    public abstract boolean isRed();

    public abstract State[] stateList();
}
