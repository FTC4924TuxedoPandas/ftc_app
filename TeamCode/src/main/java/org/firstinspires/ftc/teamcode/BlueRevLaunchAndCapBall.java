package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * Created by 4924_Users on 1/6/2017.
 */
@Autonomous(name = "BlueRevLaunchAndCapBall")
public class BlueRevLaunchAndCapBall extends RevLaunchAndCapBallBase{
    public BlueRevLaunchAndCapBall() {
        launchPositioningPath = new DrivePathSegment[]{

                    new DrivePathSegment(8.5f, 0.3f, DrivePathSegment.LINEAR),
                    new DrivePathSegment(7.0f, 0.3f, DrivePathSegment.TURN),
                    new DrivePathSegment(1.0f),
        };

        beaconPath = new DrivePathSegment[]{

                new DrivePathSegment(0.0f, 0.2f, DrivePathSegment.TURN),
                new DrivePathSegment(-2.0f, 0.3f, DrivePathSegment.LINEAR),
                new DrivePathSegment(322.0f, 0.3f, DrivePathSegment.TURN),
                new DrivePathSegment(18.3f, 0.2f, DrivePathSegment.LINEAR),
                new DrivePathSegment(282.0f, 0.3f, DrivePathSegment.TURN),
                new DrivePathSegment(8.0f, 0.2f, DrivePathSegment.LINEAR),
                new DrivePathSegment(2.0f, 0.1f, DrivePathSegment.LINEAR),
        };
    }

    @Override
    public boolean isRed() {

        return false;
    }
}

