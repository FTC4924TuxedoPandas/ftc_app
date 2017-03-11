package org.firstinspires.ftc.teamcode;

/**
 * Created by 4924_Users on 1/14/2017.
 */

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "BlueRevStraightToBeacon")
public class BlueRevStraightToBeacon extends RevStraightToBeaconBase {

    public BlueRevStraightToBeacon() {

        beaconPath = new DrivePathSegment[]{

                new DrivePathSegment(0.0f, 0.2f, DrivePathSegment.TURN),
                new DrivePathSegment(-2.0f, 0.4f, DrivePathSegment.LINEAR),
                new DrivePathSegment(38.0f, 0.3f, DrivePathSegment.TURN),
                new DrivePathSegment(13.0f, 0.4f, DrivePathSegment.LINEAR),
                new DrivePathSegment(78.0f, 0.3f, DrivePathSegment.TURN),
                new DrivePathSegment(11.0f, 0.2f, DrivePathSegment.LINEAR),
                new DrivePathSegment(5.0f, 0.05f, DrivePathSegment.LINEAR),
        };
    }

    public boolean isRed() {

        return true;
    }

    public float throwingPower() {

        return 0.9f;
    }
}
