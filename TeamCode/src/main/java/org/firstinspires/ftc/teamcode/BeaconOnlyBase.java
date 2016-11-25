package org.firstinspires.ftc.teamcode;

/**
 * Created by 4924_Users on 11/25/2016.
 */

public abstract class BeaconOnlyBase extends AutonomousBase {

    @Override
    public State[] stateList() {

        return new State[] {

                State.STATE_INITIAL,
                State.STATE_DRIVE,
                State.STATE_FIND_WHITE_LINE,
                State.STATE_PUSH_BEACON,
                State.STATE_STOP
        };
    }
}
