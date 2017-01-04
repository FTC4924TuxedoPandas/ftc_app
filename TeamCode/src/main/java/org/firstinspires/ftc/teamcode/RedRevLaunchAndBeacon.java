package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * Created by 4924_Users on 1/4/2017.
 */

@Autonomous(name = "RedRevLaunchAndBeacon")
public class RedRevLaunchAndBeacon extends RevLaunchAndBeaconBase {

    public RedRevLaunchAndBeacon() {

        launchPositioningPath = new DrivePathSegment[]{

                new DrivePathSegment(6.5f, 0.3f, DrivePathSegment.LINEAR),
                new DrivePathSegment(1.0f),
        };

        beaconPath = new DrivePathSegment[]{

                new DrivePathSegment(315.0f, 0.3f, DrivePathSegment.TURN),
                new DrivePathSegment(25.0f, 0.2f, DrivePathSegment.LINEAR),
                new DrivePathSegment(270.0f, 0.3f, DrivePathSegment.TURN),
                new DrivePathSegment(6.0f, 0.2f, DrivePathSegment.LINEAR),
                new DrivePathSegment(8.0f, 0.1f, DrivePathSegment.LINEAR),
        };
    }

    @Override
    public boolean isRed() {

        return true;
    }
}
