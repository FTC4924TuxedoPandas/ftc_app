package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * Created by 4924_Users on 1/24/2017.
 */

@Autonomous(name = "RedRevLaunchAndCapBallBeacon", group = "Revolution")
public class RedRevLaunchAndCapBallBeacon extends RevLaunchAndCapBallBeaconBase {
    public RedRevLaunchAndCapBallBeacon() {

        launchPositioningPath = new DrivePathSegment[]{

                new DrivePathSegment(2.00f, 0.3f, DrivePathSegment.LINEAR),
                new DrivePathSegment(341.0f, 0.3f, DrivePathSegment.TURN),
                new DrivePathSegment(16.0f, 0.3f, DrivePathSegment.LINEAR),
                new DrivePathSegment(327.0f, 0.3f, DrivePathSegment.TURN),
                new DrivePathSegment(1.0f),
        };
        knockCapBallPath = new DrivePathSegment[] {

                new DrivePathSegment(33.0f, 0.3f, DrivePathSegment.TURN),
                new DrivePathSegment(20.0f, 0.4f, DrivePathSegment.LINEAR),
                new DrivePathSegment(302.0f, 0.3f, DrivePathSegment.TURN),
                new DrivePathSegment(12.0f, 0.4f, DrivePathSegment.LINEAR),
        };
        stop = new DrivePathSegment [] {

                new DrivePathSegment(0.0f, 0.0f, DrivePathSegment.LINEAR),
        };
    }


    @Override
    public boolean isRed() {
        return true;
    }
}
