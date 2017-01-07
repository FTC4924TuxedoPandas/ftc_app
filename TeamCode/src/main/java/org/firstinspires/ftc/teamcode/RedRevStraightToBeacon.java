package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * Created by 4924_Users on 1/7/2017.
 */

@Autonomous(name = "RedRevStraightToBeacon")
public class RedRevStraightToBeacon extends RevStraightToBeaconBase {

    public RedRevStraightToBeacon() {

        beaconPath = new DrivePathSegment[]{

                new DrivePathSegment(4.0f, 0.3f, DrivePathSegment.LINEAR),
                new DrivePathSegment(315.0f, 0.3f, DrivePathSegment.TURN),
                new DrivePathSegment(45.0f, 0.3f, DrivePathSegment.LINEAR),
                new DrivePathSegment(270.0f, 0.3f, DrivePathSegment.TURN),
                new DrivePathSegment(18.0f, 0.1f, DrivePathSegment.LINEAR),
        };
    }

    @Override
    public boolean isRed() {

        return true;
    }
}
