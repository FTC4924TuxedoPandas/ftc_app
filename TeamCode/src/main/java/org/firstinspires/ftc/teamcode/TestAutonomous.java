package org.firstinspires.ftc.teamcode;

import android.widget.Switch;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * Created by 4924_Users on 10/15/2016.
 */

@Autonomous(name = "TestAutonomous")
public class TestAutonomous extends VelocityBase {


    public TestAutonomous() {

    }

    @Override
    public void loop() {

        telemetry.addData("currentState: ", currentState);

        switch (currentState) {

            case STATE_INITIAL:

                startPath(currentPath);
                telemetry.addData("startPath complete", 0);
                currentState = State.STATE_STOP;
                telemetry.addData("set state to stop", 0);

                break;

            case STATE_LAUNCH:

                telemetry.addData("STATE_LAUNCH", 0);

                break;

            case STATE_STOP:

                telemetry.addData("STATE_STOP", 0);
                if (pathComplete()) {

                    TurnOffAllDriveMotors();
                }

                break;
        }
    }
}
