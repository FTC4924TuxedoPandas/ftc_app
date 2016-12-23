package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * Created by 4924_Users on 12/23/2016.
 */
@Autonomous(name = "AutonomousTestMoves")
public class AutonomousTestMoves extends TestMovesBase {

    public AutonomousTestMoves() {

        beaconPath = new DrivePathSegment[] {

                new DrivePathSegment(10000.0f, 0.2f, DrivePathSegment.HOLONOMIC),
        };
    }

    @Override
    public boolean isRed() {

        return false;
    }
}
