package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * Created by 4924_Users on 10/15/2016.
 */

//@Autonomous(name = "TestAutonomous")
public class TestAutonomous extends AutonomousBase {

    public TestAutonomous() {

        currentPath = new DrivePathSegment[]{

                new DrivePathSegment(36.0f, 1.0f, DrivePathSegment.LINEAR)
        };
    }

    @Override
    public void loop() {

        telemetry.addData("currentState: ", currentState);

        switch (currentState) {

            case STATE_INITIAL:

                startPath(currentPath);
                currentState = State.STATE_DRIVE;

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

    @Override
    public boolean isRed() {
        return false;
    }

    @Override
    public State[] stateList() {
        return new State[0];
    }
}
