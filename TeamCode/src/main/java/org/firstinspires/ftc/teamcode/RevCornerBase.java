package org.firstinspires.ftc.teamcode;

/**
 * Created by 4924_Users on 3/11/2017.
 */

public abstract class RevCornerBase extends RevolutionAutonomousBase {
    @Override
    public State[] stateList() {

        return new State[] {

                State.STATE_INITIAL,
                State.STATE_START_LAUNCH_PATH,
                State.STATE_POSITION_FOR_BALL,
                State.STATE_LAUNCH_BALL,
                State.STATE_LOAD_BALL,
                State.STATE_WAIT_FOR_BALL,
                State.STATE_LAUNCH_BALL,
                State.STATE_START_AFTER_THROWING_PATH,
                State.STATE_DRIVE_AFTER_THROWING,
                State.STATE_STOP
        };
    }
}
