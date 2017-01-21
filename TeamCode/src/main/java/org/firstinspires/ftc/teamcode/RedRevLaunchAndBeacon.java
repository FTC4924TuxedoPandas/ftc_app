package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
/**
 * Created by 4924_Users on 1/4/2017.
 */

@Autonomous(name = "RedRevLaunchAndBeacon", group = "Revolution")
public class RedRevLaunchAndBeacon extends RevLaunchAndBeaconBase {

    public RedRevLaunchAndBeacon() {

        launchPositioningPath = new DrivePathSegment[] {

                new DrivePathSegment(8.5f, 0.4f, DrivePathSegment.LINEAR),
                new DrivePathSegment(7.0f, 0.3f, DrivePathSegment.TURN),
                new DrivePathSegment(0.5f),
        };

        beaconPath = new DrivePathSegment[] {

                new DrivePathSegment(0.0f, 0.2f, DrivePathSegment.TURN),
                new DrivePathSegment(-2.0f, 0.4f, DrivePathSegment.LINEAR),
                new DrivePathSegment(322.0f, 0.3f, DrivePathSegment.TURN),
                new DrivePathSegment(15.0f, 0.4f, DrivePathSegment.LINEAR),
                new DrivePathSegment(286.0f, 0.3f, DrivePathSegment.TURN),
                new DrivePathSegment(11.0f, 0.2f, DrivePathSegment.LINEAR),
                new DrivePathSegment(2.0f, 0.05f, DrivePathSegment.LINEAR),
                new DrivePathSegment(-2.0f, 0.5f, DrivePathSegment.HOLONOMIC),
        };
    }

    @Override
    public boolean isRed() {

        return true;
    }
}