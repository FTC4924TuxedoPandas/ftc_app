package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * Created by 4924_Users on 10/22/2016.
 */

@Autonomous(name = "BlueRampLaunchAndBeacon")
public class BlueRampLaunchAndBeacon extends LaunchAndBeaconBase {

    public BlueRampLaunchAndBeacon() {

        launchPositioningPath = new DrivePathSegment[]{

                new DrivePathSegment(0.0f, 0.0f, DrivePathSegment.LINEAR),
        };

        beaconPath = new DrivePathSegment[]{

                new DrivePathSegment(45.0f, 1.0f, DrivePathSegment.LINEAR),
                new DrivePathSegment(40.0f, 1.0f, DrivePathSegment.HOLONOMIC),
        };
    }

    @Override
    public boolean isRed() {

        return false;
    }
}