package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * Created by 4924_Users on 10/22/2016.
 */

@Autonomous(name = "RedRampLaunchAndBeacon")
public class BlueRampLaunchAndBeacon extends LaunchAndBeaconBase {

    public BlueRampLaunchAndBeacon() {

        launchPositioningPath = new DrivePathSegment[]{

                new DrivePathSegment(0.0f, 0.0f, 0.0f),
        };

        beaconPath = new DrivePathSegment[]{

                new DrivePathSegment(0.0f, 0.0f, 0.0f),
        };
    }
}
