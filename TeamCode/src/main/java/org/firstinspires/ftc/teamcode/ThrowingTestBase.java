package org.firstinspires.ftc.teamcode;

/**
 * Created by 4924_Users on 12/23/2016.
 */

public abstract class ThrowingTestBase extends AutonomousBase {

    @Override
    public State[] stateList() {

        return new State[] {

                State.STATE_INITIAL,
                State.STATE_LAUNCH_BALL,
                State.STATE_LOAD_BALL,
                State.STATE_WAIT_FOR_BALL,
                State.STATE_LAUNCH_BALL,
                State.STATE_STOP
        };
    }
}
