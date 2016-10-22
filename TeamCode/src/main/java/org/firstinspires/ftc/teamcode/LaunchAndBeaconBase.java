package org.firstinspires.ftc.teamcode;

/**
 * Created by 4924_Users on 10/22/2016.
 */

public class LaunchAndBeaconBase extends VelocityBase {

    final float THROWING_TIME = 0.5f;

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
                    currentState = State.STATE_LAUNCH_BALL;
                    elapsedTimeForCurrentState.reset();
                }

                break;

            case STATE_LAUNCH_BALL:

                throwBall(elapsedTimeForCurrentState, THROWING_TIME);

                if (elapsedTimeForCurrentState.time() >= THROWING_TIME * 2) {

                    startPath(beaconPath);
                    currentState = State.STATE_DRIVE;
                }

                break;

            case STATE_DRIVE:

                if (pathComplete()) {

                    TurnOffAllDriveMotors();
                    currentState = State.STATE_STOP;
                }

                break;

            case STATE_STOP:


                break;
        }
    }
}
