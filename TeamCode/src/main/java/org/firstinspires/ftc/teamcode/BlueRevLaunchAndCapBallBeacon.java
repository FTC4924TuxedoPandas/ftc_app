package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * Created by 4924_Users on 1/10/2017.
 */

@Autonomous(name = "BlueRevLaunchAndCapBallBeacon")
public class BlueRevLaunchAndCapBallBeacon extends RevLaunchAndCapBallBeaconBase  {
    public BlueRevLaunchAndCapBallBeacon() {
        launchPositioningPath = new DrivePathSegment[]{

                new DrivePathSegment(2.00f, 0.3f, DrivePathSegment.LINEAR),
                new DrivePathSegment(36.0f, 0.3f, DrivePathSegment.TURN),
                new DrivePathSegment(15.0f, 0.3f, DrivePathSegment.LINEAR),
                new DrivePathSegment(45.0f, 0.3f, DrivePathSegment.TURN),
                new DrivePathSegment(1.0f),


        };

        knockCapBallPath = new DrivePathSegment[] {

                new DrivePathSegment(0.0f, 0.3f, DrivePathSegment.TURN),
                new DrivePathSegment(7.00f, 0.3f, DrivePathSegment.LINEAR),
        };

        beaconPath = new DrivePathSegment[]{

                new DrivePathSegment(-2.0f, 0.4f, DrivePathSegment.LINEAR),
                new DrivePathSegment(38.0f, 0.3f, DrivePathSegment.TURN),
                new DrivePathSegment(13.0f, 0.4f, DrivePathSegment.LINEAR),
                new DrivePathSegment(78.0f, 0.3f, DrivePathSegment.TURN),
                new DrivePathSegment(11.0f, 0.2f, DrivePathSegment.LINEAR),
                new DrivePathSegment(5.0f, 0.05f, DrivePathSegment.LINEAR),
        };

        stop = new DrivePathSegment [] {

                new DrivePathSegment(0.0f, 0.0f, DrivePathSegment.LINEAR),
        };
    }


    @Override
    public boolean isRed() {
        return false;
    }
}


