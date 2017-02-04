package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * Created by 4924_Users on 2/4/2017.
 */
@Autonomous(name = "BeaconLogicTest")
public class BeaconLogicTest extends RevolutionAutonomousBase {

    public BeaconLogicTest() {

        beaconPath = new DrivePathSegment[] {

                new DrivePathSegment(5.0f, 0.2f, DrivePathSegment.LINEAR),
                new DrivePathSegment(3.0f, 0.05f, DrivePathSegment.LINEAR),
        };
    }

    public boolean isRed() {

        return true;
    }

    public State[] stateList() {

        return new State[] {

                State.STATE_INITIAL,
                State.STATE_START_BEACON_PATH,
                State.STATE_DRIVE,
                State.STATE_FIND_WHITE_LINE,
                State.STATE_LINE_UP_TO_BEACON,
                State.STATE_SQUARE_ON_WALL,
                State.STATE_PUSH_BEACON,
                State.STATE_FIND_WHITE_LINE,
                State.STATE_LINE_UP_TO_BEACON,
                State.STATE_SQUARE_ON_WALL,
                State.STATE_PUSH_BEACON,
                State.STATE_STOP
        };
    }
}