package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Created by 4924_Users on 10/22/2016.
 */

public abstract class AutonomousBase extends VelocityBase {

    final float THROWING_TIME = 0.2f;
    public int stateIndex = 0;
    public ElapsedTime elapsedTimeForMove = new ElapsedTime();
    public boolean isPushing = false;
    public boolean isSecondBeacon = false;

    @Override
    public void loop() {

        telemetry.addData("currentState: ", currentState);
        telemetry.addData("Gyro", turningGyro.getHeading());
        telemetry.addData("Target", segment.Angle);

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

                telemetry.addData("RightPosition", getRightPosition());
                telemetry.addData("RightTarget", currentEncoderTargets.frontRightTarget);
                telemetry.addData("LeftPosition", getLeftPosition());
                telemetry.addData("LeftTarget", currentEncoderTargets.frontLeftTarget);

                break;

            case STATE_FIND_WHITE_LINE:

                telemetry.addData("LineSensor", lineSensor.getRawLightDetected());

                if (lineSensor.getRawLightDetected() >= 0.5f && elapsedTimeForCurrentState.time() >= 1.0f) {

                    TurnOffAllDriveMotors();
                    switchToNextState();

                } else {

                    if (isPushing) {

                        if (elapsedTimeForMove.time() >= 1.0f) {

                            isPushing = !isPushing;
                            elapsedTimeForMove.reset();
                        }

                    } else {

                        if (elapsedTimeForMove.time() >= 5.0f) {

                            isPushing = !isPushing;
                            elapsedTimeForMove.reset();
                        }
                    }

                    if (isRed()) {

                        setPowerForMecanumStrafe(0.2f);

                    } else {

                        setPowerForMecanumStrafe(-0.2f);
                    }

                    if (isPushing && !isRed()) {

                        powerLevels.frontRightPower= 0.3f;
                        powerLevels.backRightPower= 0.3f;
                        powerLevels.frontLeftPower= 0.2f;
                        powerLevels.backLeftPower= 0.2f;
                    }

                    setMotorPowerLevels(powerLevels);
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
                        setMotorPowerLevels(powerLevels);
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

    public abstract boolean isRed();

    public abstract State[] stateList();
}
