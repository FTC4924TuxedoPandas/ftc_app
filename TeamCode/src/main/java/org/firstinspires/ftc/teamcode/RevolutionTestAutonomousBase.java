package org.firstinspires.ftc.teamcode;

/**
 * Created by 4924_Users on 1/3/2017.
 */

public abstract class RevolutionTestAutonomousBase extends  RevolutionAutonomousBase {

    @Override
    public State[] stateList() {

        return new State[] {

                State.STATE_INITIAL,
                State.STATE_START_BEACON_PATH,
                State.STATE_DRIVE,
                State.STATE_STOP,
        };
    }
}
