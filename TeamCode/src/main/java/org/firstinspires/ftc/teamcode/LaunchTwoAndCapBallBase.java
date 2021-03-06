package org.firstinspires.ftc.teamcode;

/**
 * Created by 4924_Users on 11/27/2016.
 */

public abstract class LaunchTwoAndCapBallBase extends AutonomousBase {

    @Override
    public State[] stateList() {

        return new State[] {

                State.STATE_INITIAL,
                State.STATE_START_LAUNCH_PATH,
                State.STATE_POSITION_FOR_BALL,
                State.STATE_DROP_GATE,
                State.STATE_LAUNCH_BALL,
                State.STATE_LOAD_BALL,
                State.STATE_WAIT_FOR_BALL,
                State.STATE_LAUNCH_BALL,
                State.STATE_START_CAP_BALL_PATH,
                State.STATE_KNOCK_CAP_BALL,
                State.STATE_START_BEACON_PATH,
                State.STATE_DRIVE,
                State.STATE_FIND_WHITE_LINE,
                State.STATE_lINE_UP_TO_BEACON,
                State.STATE_PUSH_BEACON,
                State.STATE_STOP
        };
    }
}
