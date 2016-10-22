package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * Created by 4924_Users on 10/22/2016.
 */

@Autonomous(name = "RedRampLaunchAndBeacon")
public class RedRampLaunchAndBeacon extends LaunchAndBeaconBase {

    public RedRampLaunchAndBeacon() {

        currentPath = new DrivePathSegment[]{

                new DrivePathSegment(37.0f, 37.0f, 1.0f),
        };

        launchPositioningPath = new DrivePathSegment[]{

                new DrivePathSegment(0.0f, 0.0f, 0.0f),
        };
    }
}
