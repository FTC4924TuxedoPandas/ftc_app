package org.firstinspires.ftc.teamcode;

/**
 * Created by 4924_Users on 11/22/2016.
 */

public abstract class LaunchTwiceAndBeaconBase extends AutonomousBase {

    @Override
    public State[] stateList() {

        return new State[] {

                State.STATE_INITIAL,
                State.STATE_POSITION_FOR_BALL,
                State.STATE_LAUNCH_FIRST_BALL,
                State.STATE_LOAD_BALL,
                State.STATE_LAUNCH_SECOND_BALL,
                State.STATE_DRIVE,
                State.STATE_FIND_WHITE_LINE,
                State.STATE_PUSH_BEACON,
                State.STATE_STOP
        };
    }
}
