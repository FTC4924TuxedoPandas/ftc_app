package org.firstinspires.ftc.teamcode;

/**
 * Created by 4924_Users on 11/27/2016.
 */

public class RedLaunchTwoAndCapBall extends LaunchTwoAndCapBallBase {

    public RedLaunchTwoAndCapBall() {

        launchPositioningPath = new DrivePathSegment[]{

                new DrivePathSegment(5.0f, 1.0f, DrivePathSegment.LINEAR),
                new DrivePathSegment(315.0f, 0.2f, DrivePathSegment.TURN),
                new DrivePathSegment(20.0f, 1.0f, DrivePathSegment.LINEAR),
                new DrivePathSegment(1.0f),
        };

        knockCapBallPath = new DrivePathSegment[]{

                new DrivePathSegment(40.0f, 1.0f, DrivePathSegment.LINEAR),
        };
    }

    @Override
    public boolean isRed() {

        return true;
    }
}
