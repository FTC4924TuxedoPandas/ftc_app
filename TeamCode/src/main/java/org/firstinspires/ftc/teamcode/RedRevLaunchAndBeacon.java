package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
/**
 * Created by 4924_Users on 1/4/2017.
 */

@Autonomous(name = "RedRevLaunchAndBeacon", group = "Revolution")
public class RedRevLaunchAndBeacon extends RevLaunchAndBeaconBase {

    public RedRevLaunchAndBeacon() {

        launchPositioningPath = new DrivePathSegment[] {

                new DrivePathSegment(5.5f, 0.8f, DrivePathSegment.LINEAR),
                new DrivePathSegment(7.0f, 0.8f, DrivePathSegment.TURN),
                new DrivePathSegment(0.25f),
        };

        beaconPath = new DrivePathSegment[] {

                new DrivePathSegment(0.0f, 0.8f, DrivePathSegment.TURN),
                new DrivePathSegment(1.0f, 0.8f, DrivePathSegment.LINEAR),
                new DrivePathSegment(322.0f, 0.8f, DrivePathSegment.TURN),
                new DrivePathSegment(11.0f, 0.8f, DrivePathSegment.LINEAR),
                new DrivePathSegment(286.0f, 0.8f, DrivePathSegment.TURN),
                new DrivePathSegment(3.0f, 0.1f, DrivePathSegment.LINEAR),
                new DrivePathSegment(12.0f, 0.4f, DrivePathSegment.LINEAR),
        };
    }

    @Override
    public boolean isRed() {

        return true;
    }

    public float throwingPower() {

        return 0.85f;
    }
}