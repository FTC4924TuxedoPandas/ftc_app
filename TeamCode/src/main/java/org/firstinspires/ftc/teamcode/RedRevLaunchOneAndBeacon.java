package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * Created by 4924_Users on 1/13/2017.
 */
@Autonomous(name = "RedRevLaunchOneAndBeacon", group = "Revolution")
public class RedRevLaunchOneAndBeacon extends RevLaunchOneAndBeaconBase{
    public RedRevLaunchOneAndBeacon() {

        launchPositioningPath = new DrivePathSegment[]{

                new DrivePathSegment(8.5f, 0.4f, DrivePathSegment.LINEAR),
                new DrivePathSegment(7.0f, 0.3f, DrivePathSegment.TURN),
                new DrivePathSegment(1.0f),
        };

        beaconPath = new DrivePathSegment[]{

                new DrivePathSegment(0.0f, 0.2f, DrivePathSegment.TURN),
                new DrivePathSegment(-2.0f, 0.4f, DrivePathSegment.LINEAR),
                new DrivePathSegment(322.0f, 0.3f, DrivePathSegment.TURN),
                new DrivePathSegment(15.0f, 0.4f, DrivePathSegment.LINEAR),
                new DrivePathSegment(286.0f, 0.3f, DrivePathSegment.TURN),
                new DrivePathSegment(10.0f, 0.2f, DrivePathSegment.LINEAR),
                new DrivePathSegment(3.0f, 0.05f, DrivePathSegment.LINEAR),
        };
    }

    public boolean isRed() {

        return true;
    }
}

