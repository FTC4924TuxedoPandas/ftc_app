package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * Created by 4924_Users on 10/22/2016.
 */

@Autonomous(name = "BlueLaunchTwoAndTwoBeacons")
public class BlueLaunchTwoAndTwoBeacons extends LaunchTwoAndTwoBeaconsBase {

    public BlueLaunchTwoAndTwoBeacons() {

        launchPositioningPath = new DrivePathSegment[]{

                new DrivePathSegment(6.0f, 1.0f, DrivePathSegment.LINEAR),
                new DrivePathSegment(0.5f),
        };

        beaconPath = new DrivePathSegment[]{

                new DrivePathSegment(50.0f, 0.2f, DrivePathSegment.TURN),
                new DrivePathSegment(45.0f, 1.0f, DrivePathSegment.LINEAR),
                new DrivePathSegment(90.0f, 0.2f, DrivePathSegment.TURN),
                new DrivePathSegment(5.0f, 0.1f, DrivePathSegment.LINEAR),
                //new DrivePathSegment(90.0f, 0.2f, DrivePathSegment.TURN),
                //new DrivePathSegment(5.0f, 0.1f, DrivePathSegment.LINEAR),
        };
    }
    @Override
    public boolean isRed() {

        return false;

    }
}