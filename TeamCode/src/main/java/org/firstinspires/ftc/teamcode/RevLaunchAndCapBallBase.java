package org.firstinspires.ftc.teamcode;

/**
 * Created by 4924_Users on 1/6/2017.
 */

public abstract class RevLaunchAndCapBallBase extends RevolutionAutonomousBase {
    @Override
    public State[] stateList() {

        return new State[] {

                State.STATE_INITIAL,
                State.STATE_START_LAUNCH_PATH,
                State.STATE_POSITION_FOR_BALL,
                State.STATE_LAUNCH_BALL,
                State.STATE_DROP_GATE,
                State.STATE_LOAD_BALL,
                State.STATE_WAIT_FOR_BALL,
                State.STATE_LAUNCH_BALL,
                State.STATE_START_CAP_BALL_PATH,
                State.STATE_KNOCK_CAP_BALL,
                State.STATE_STOP
        };
    }
}
