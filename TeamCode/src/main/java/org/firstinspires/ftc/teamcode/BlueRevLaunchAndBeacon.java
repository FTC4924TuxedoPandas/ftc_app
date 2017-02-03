package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
/**
 * Created by 4924_Users on 1/5/2017.
 */

@Autonomous(name = "BlueRevLaunchAndBeacon", group = "Revolution")
public class BlueRevLaunchAndBeacon extends RevLaunchAndBeaconBase {

    public BlueRevLaunchAndBeacon() {

        launchPositioningPath = new DrivePathSegment[] {

                new DrivePathSegment(8.5f, 0.4f, DrivePathSegment.LINEAR),
                new DrivePathSegment(14.0f, 0.3f, DrivePathSegment.TURN),
                new DrivePathSegment(0.5f),
        };

        beaconPath = new DrivePathSegment[] {

                new DrivePathSegment(0.0f, 0.2f, DrivePathSegment.TURN),
                new DrivePathSegment(-2.0f, 0.4f, DrivePathSegment.LINEAR),
                new DrivePathSegment(38.0f, 0.3f, DrivePathSegment.TURN),
                new DrivePathSegment(11.0f, 0.4f, DrivePathSegment.LINEAR),
                new DrivePathSegment(78.0f, 0.3f, DrivePathSegment.TURN),
                new DrivePathSegment(13.0f, 0.2f, DrivePathSegment.LINEAR),
                new DrivePathSegment(3.0f, 0.05f, DrivePathSegment.LINEAR),
        };
        leaveBeaconPath = new DrivePathSegment[] {

                new DrivePathSegment(-0.05f, 0.3f, DrivePathSegment.LINEAR),

        };
    }

    @Override
    public boolean isRed() {

        return false;
    }
}

