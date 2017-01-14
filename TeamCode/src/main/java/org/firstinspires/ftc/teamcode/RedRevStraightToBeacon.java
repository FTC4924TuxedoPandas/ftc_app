package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * Created by 4924_Users on 1/7/2017.
 */

@Autonomous(name = "RedRevStraightToBeacon", group = "Revolution")
public class RedRevStraightToBeacon extends RevStraightToBeaconBase {

    public RedRevStraightToBeacon() {

        beaconPath = new DrivePathSegment[]{
//dist. ratio at 0.3 power is approx. 1 : 3.4
                new DrivePathSegment(0.71f, 0.3f, DrivePathSegment.LINEAR),
                new DrivePathSegment(315.0f, 0.3f, DrivePathSegment.TURN),
                new DrivePathSegment(17.1f, 0.3f, DrivePathSegment.LINEAR),
                new DrivePathSegment(270.0f, 0.3f, DrivePathSegment.TURN),
                new DrivePathSegment(4.0f, 0.1f, DrivePathSegment.LINEAR),
        };
    }

    @Override
    public boolean isRed() {

        return true;
    }
}
