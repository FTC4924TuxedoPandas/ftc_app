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
                new DrivePathSegment(2.00f, 0.3f, DrivePathSegment.LINEAR),
                new DrivePathSegment(36.0f, 0.3f, DrivePathSegment.TURN),
                new DrivePathSegment(15.0f, 0.3f, DrivePathSegment.LINEAR),
                new DrivePathSegment(75.0f, 0.3f, DrivePathSegment.TURN),
                new DrivePathSegment(1.0f),
        };

        postThrowingPath = new DrivePathSegment[] {

                new DrivePathSegment(7.0f, 0.3f, DrivePathSegment.HOLONOMIC),
                new DrivePathSegment(8.0f, 0.3f, DrivePathSegment.LINEAR),
                new DrivePathSegment(165.0f, 0.3f, DrivePathSegment.TURN),
                new DrivePathSegment(5.0f, 0.3f, DrivePathSegment.LINEAR),
        };


    }

    @Override
    public boolean isRed() {

        return false;
    }

    public float throwingPower() {

        return 0.9f;
    }
}

