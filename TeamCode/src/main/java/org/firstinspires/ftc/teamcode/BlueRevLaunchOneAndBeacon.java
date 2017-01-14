package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * Created by 4924_Users on 1/13/2017.
 */
@Autonomous(name = "BlueRevLaunchOneAndBeacon", group = "Revolution")
public class BlueRevLaunchOneAndBeacon extends RevLaunchOneAndBeaconBase{
    public BlueRevLaunchOneAndBeacon() {

        launchPositioningPath = new DrivePathSegment[]{

                new DrivePathSegment(8.5f, 0.4f, DrivePathSegment.LINEAR),
                new DrivePathSegment(8.0f, 0.3f, DrivePathSegment.TURN),
                new DrivePathSegment(1.0f),
        };

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

        return false;
    }
}
