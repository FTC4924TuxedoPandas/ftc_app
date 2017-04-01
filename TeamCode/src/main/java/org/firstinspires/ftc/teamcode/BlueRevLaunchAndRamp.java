package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * Created by 4924_Users on 3/11/2017.
 */
@Autonomous(name = "BlueRevLaunchAndRamp", group = "Revolution")
public class BlueRevLaunchAndRamp extends RevCornerBase {

    public BlueRevLaunchAndRamp(){

        launchPositioningPath = new DrivePathSegment[]{

                new DrivePathSegment(10.0f),
                new DrivePathSegment(2.00f, 0.8f, DrivePathSegment.LINEAR),
                new DrivePathSegment(36.0f, 0.5f, DrivePathSegment.TURN),
                new DrivePathSegment(13.0f, 0.8f, DrivePathSegment.LINEAR),
                new DrivePathSegment(45.0f, 0.5f, DrivePathSegment.TURN),
                new DrivePathSegment(1.0f),
        };

        postThrowingPath = new DrivePathSegment[] {

                new DrivePathSegment(2.0f, 0.8f, DrivePathSegment.LINEAR),
                new DrivePathSegment(7.0f, 0.8f, DrivePathSegment.HOLONOMIC),
                new DrivePathSegment(8.0f, 0.8f, DrivePathSegment.LINEAR),
                new DrivePathSegment(100.0f, 0.5f, DrivePathSegment.TURN),
                new DrivePathSegment(14.0f, 0.8f, DrivePathSegment.LINEAR),
                new DrivePathSegment(6.0f, 0.4f, DrivePathSegment.LINEAR),
        };


    }

    @Override
    public boolean isRed() {

        return false;
    }

    public float throwingPower() {

        return 0.85f;
    }
}

