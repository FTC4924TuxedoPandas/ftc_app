package org.firstinspires.ftc.teamcode;

/**
 * Created by 4924_Users on 1/13/2017.
 */

public abstract class RevLaunchOneAndBeaconBase extends RevolutionAutonomousBase{
    @Override
    public State[] stateList() {

        return new State[] {

                State.STATE_INITIAL,
                State.STATE_START_LAUNCH_PATH,
                State.STATE_POSITION_FOR_BALL,
                State.STATE_LAUNCH_BALL,
                State.STATE_START_BEACON_PATH,
                State.STATE_DRIVE,
                State.STATE_FIND_WHITE_LINE,
                State.STATE_LINE_UP_TO_BEACON,
                State.STATE_PUSH_BEACON,
                State.STATE_FIND_WHITE_LINE,
                State.STATE_LINE_UP_TO_BEACON,
                State.STATE_PUSH_BEACON,
                State.STATE_STOP
        };
    }
}
