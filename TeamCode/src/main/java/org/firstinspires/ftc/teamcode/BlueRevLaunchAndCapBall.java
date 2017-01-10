package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * Created by 4924_Users on 1/6/2017.
 */
@Autonomous(name = "BlueRevLaunchAndCapBall")
public class BlueRevLaunchAndCapBall extends RevLaunchAndCapBallBase{
    public BlueRevLaunchAndCapBall() {
        launchPositioningPath = new DrivePathSegment[]{

                new DrivePathSegment(1.0f, 0.3f, DrivePathSegment.LINEAR),
                new DrivePathSegment(48.0f, 0.2f, DrivePathSegment.TURN),
                new DrivePathSegment(17.0f, 0.3f, DrivePathSegment.LINEAR),
                new DrivePathSegment(1.0f),
        };
        knockCapBallPath = new DrivePathSegment[] {

                new DrivePathSegment(0.07f, 0.07f, DrivePathSegment.LINEAR),
        };
    }

    @Override
    public boolean isRed() {

        return false;
    }
}

