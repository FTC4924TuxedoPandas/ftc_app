package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * Created by 4924_Users on 1/4/2017.
 */

@Autonomous(name = "RedRevLaunchAndBeacon")
public class RedRevLaunchAndBeacon extends RevLaunchAndBeaconBase {

    public RedRevLaunchAndBeacon() {

        launchPositioningPath = new DrivePathSegment[]{

                new DrivePathSegment(8.5f, 0.3f, DrivePathSegment.LINEAR),
                new DrivePathSegment(1.0f),
        };

        beaconPath = new DrivePathSegment[]{

                new DrivePathSegment(322.0f, 0.3f, DrivePathSegment.TURN), //I found that the robot was always over rotating its turns (high margin of error) 1/4/17
                new DrivePathSegment(15.0f, 0.2f, DrivePathSegment.LINEAR),
                new DrivePathSegment(280.0f, 0.3f, DrivePathSegment.TURN),
                new DrivePathSegment(6.0f, 0.2f, DrivePathSegment.LINEAR),
                new DrivePathSegment(7.0f, 0.1f, DrivePathSegment.LINEAR),
        };
    }

    @Override
    public boolean isRed() {

        return true;
    }
}
