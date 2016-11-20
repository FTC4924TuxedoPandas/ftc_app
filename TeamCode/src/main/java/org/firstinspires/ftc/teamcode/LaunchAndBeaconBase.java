package org.firstinspires.ftc.teamcode;

/**
 * Created by 4924_Users on 10/22/2016.
 */

public abstract class LaunchAndBeaconBase extends VelocityBase {

    final float THROWING_TIME = 0.2f;

    @Override
    public void loop() {

        telemetry.addData("currentState: ", currentState);

        switch (currentState) {

            case STATE_INITIAL:

                currentState = State.STATE_POSITION_FOR_BALL;
                startPath(launchPositioningPath);

                break;

            case STATE_POSITION_FOR_BALL:

                if (pathComplete()) {

                    TurnOffAllDriveMotors();
                    currentState = State.STATE_LAUNCH_FIRST_BALL;
                    elapsedTimeForCurrentState.reset();
                }

                break;

            case STATE_LAUNCH_FIRST_BALL:

                throwBall(elapsedTimeForCurrentState, THROWING_TIME);

                if (elapsedTimeForCurrentState.time() >= 3.0f) {

                    currentState = State.STATE_LOAD_BALL;
                    collectionGateServo.setPosition(GATE_SERVO_POSITION_LOW);
                    elapsedTimeForCurrentState.reset();
                }

                break;

            case STATE_LOAD_BALL:

                if (elapsedTimeForCurrentState.time() >= 1.0f) {

                    currentState = State.STATE_LOAD_BALL;
                    elapsedTimeForCurrentState.reset();
                }

                break;

            case STATE_LAUNCH_SECOND_BALL:

                if (elapsedTimeForCurrentState.time() >= 1.0f) {

                    throwBall(elapsedTimeForCurrentState, THROWING_TIME);
                }

                if (elapsedTimeForCurrentState.time() >= 3.0f) {

                    startPath(beaconPath);
                    currentState = State.STATE_DRIVE;
                }

                break;

            case STATE_DRIVE:

                if (pathComplete()) {

                    TurnOffAllDriveMotors();
                    currentState = State.STATE_FIND_WHITE_LINE;
                }

                telemetry.addData("RightPosition", getRightPosition());
                telemetry.addData("RightTarget", currentEncoderTargets.frontRightTarget);
                telemetry.addData("LeftPosition", getLeftPosition());
                telemetry.addData("LeftTarget", currentEncoderTargets.frontLeftTarget);

                break;

            case STATE_FIND_WHITE_LINE:

                if (lineSensor.getRawLightDetected() >= 1.0f) {

                    elapsedTimeForCurrentState.reset();
                    TurnOffAllDriveMotors();
                    currentState = State.STATE_PUSH_BEACON;

                } else {

                    if (isRed()) {

                        setPowerForMecanumStrafe(-1.0f);

                    } else {

                        setPowerForMecanumStrafe(1.0f);
                    }

                    setMotorPowerLevels(powerLevels);
                }

                break;

            case STATE_PUSH_BEACON:

                if (elapsedTimeForCurrentState.time() <= 3.0f) {

                    if (isRed()) {

                        pushBeaconButton(leftBeaconSensor.red(), rightBeaconSensor.red());

                    } else {

                        pushBeaconButton(leftBeaconSensor.blue(), rightBeaconSensor.blue());
                    }

                } else {

                    rightBeaconServo.setPosition(0.0f);
                    leftBeaconServo.setPosition(0.0f);
                    currentState = State.STATE_STOP;
                }

                break;

            case STATE_STOP:

                break;
        }
    }

    public abstract boolean isRed();
}
