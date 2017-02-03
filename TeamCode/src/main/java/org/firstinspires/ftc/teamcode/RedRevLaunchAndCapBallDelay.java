package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * Created by 4924_Users on 1/21/2017.
 */

@Autonomous(name = "RedRevLaunchAndCapBallDelay")
public class RedRevLaunchAndCapBallDelay extends RevLaunchAndCapBallBase {
    public RedRevLaunchAndCapBallDelay() {

        launchPositioningPath = new DrivePathSegment[]{

                new DrivePathSegment(1.00f),
                new DrivePathSegment(1.00f, 0.3f, DrivePathSegment.LINEAR),
                new DrivePathSegment(336.0f, 0.3f, DrivePathSegment.TURN),
                new DrivePathSegment(16.0f, 0.3f, DrivePathSegment.LINEAR),
                // new DrivePathSegment(45.0f, 0.3f, DrivePathSegment.TURN),
                new DrivePathSegment(1.0f),
        };
        knockCapBallPath = new DrivePathSegment[] {

                //new DrivePathSegment(35.0f, 0.3f, DrivePathSegment.TURN),
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