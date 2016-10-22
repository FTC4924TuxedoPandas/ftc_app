package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * Created by 4924_Users on 10/22/2016.
 */

@Autonomous(name = "RedRampLaunchAndBeacon")
public class RedRampLaunchAndBeacon extends VelocityBase {

    public RedRampLaunchAndBeacon() {

        currentPath = new DrivePathSegment[]{

                new DrivePathSegment(37.0f, 37.0f, 50.0f),
                //new DrivePathSegment(90.0f, 50.0f),
                //new DrivePathSegment(5.0f),
                //new DrivePathSegment(0.0f, 50.0f),
                //new DrivePathSegment(-20.0f, -20.0f, 50.0f)
        };
    }

    @Override
    public void loop() {

        telemetry.addData("currentState: ", currentState);

        switch (currentState) {

            case STATE_INITIAL:

                currentState = State.STATE_LAUNCH_BALL;
                elapsedTimeForCurrentState.reset();

                break;

            case STATE_LAUNCH_BALL:

                if (elapsedTimeForCurrentState.time() >= 0.5f) {

                    throwingArm.setPower(0.0f);
                    startPath(currentPath);
                    currentState = State.STATE_DRIVE;

                } else {

                    throwingArm.setPower(ARM_POWER);
                }

                break;

            case STATE_DRIVE:

                if (pathComplete()) {

                    TurnOffAllDriveMotors();
                    currentState = State.STATE_STOP;
                }

                break;

            case STATE_STOP:

                telemetry.addData("STATE_STOP", 0);

                break;
        }
    }
}
