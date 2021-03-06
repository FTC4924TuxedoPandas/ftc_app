package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * Created by 4924_Users on 12/17/2016.
 */
//@Autonomous(name = "BlueLaunchTwoAndCapBall")
public class BlueLaunchTwoAndCapBall extends LaunchTwoAndCapBallBase {

    public BlueLaunchTwoAndCapBall () {

        launchPositioningPath = new DrivePathSegment[] {

                new DrivePathSegment(5.0f, 1.0f, DrivePathSegment.LINEAR),
                new DrivePathSegment(45.0f, 0.2f, DrivePathSegment.TURN),
                new DrivePathSegment(28.0f, 1.0f, DrivePathSegment.LINEAR),
                new DrivePathSegment(1.0f),
        };

        knockCapBallPath = new DrivePathSegment[] {

                new DrivePathSegment(20.0f, 1.0f, DrivePathSegment.LINEAR),
        };

        beaconPath = new DrivePathSegment[] {

                new DrivePathSegment(90.0f, 0.2f, DrivePathSegment.TURN),
                new DrivePathSegment(30.0f, 1.0f, DrivePathSegment.LINEAR),
                new DrivePathSegment(1.0f),
        };
    }

    @Override
    public boolean isRed() {

        return false;
    }
}
