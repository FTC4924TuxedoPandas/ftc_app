package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * Created by 4924_Users on 1/6/2017.
 */
@Autonomous(name = "RedRevLaunchAndCapBall", group = "Revolution")
public class RedRevLaunchAndCapBall extends RevLaunchAndCapBallBase {
    public RedRevLaunchAndCapBall() {

        launchPositioningPath = new DrivePathSegment[]{

                new DrivePathSegment(1.00f, 0.3f, DrivePathSegment.LINEAR),
                new DrivePathSegment(310.0f, 0.3f, DrivePathSegment.TURN),
                new DrivePathSegment(15.0f, 0.3f, DrivePathSegment.LINEAR),
                new DrivePathSegment(50.0f, 0.3f, DrivePathSegment.TURN),
                new DrivePathSegment(1.0f),
        };
        knockCapBallPath = new DrivePathSegment[] {

                new DrivePathSegment(45.0f, 0.3f, DrivePathSegment.TURN),
                new DrivePathSegment(7.0f, 0.3f, DrivePathSegment.LINEAR),
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
