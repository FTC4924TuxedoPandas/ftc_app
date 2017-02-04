package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * Created by 4924_Users on 2/4/2017.
 */
@Autonomous(name = "BeaconLogicTest")
public class BeaconLogicTest extends RevolutionAutonomousBase {

    public boolean isRed() {

        return true;
    }

    public State[] stateList() {

        return new State[] {

                State.STATE_INITIAL,
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