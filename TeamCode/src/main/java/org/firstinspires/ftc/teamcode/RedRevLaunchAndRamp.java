package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * Created by 4924_Users on 2/18/2017.
 */
@Autonomous(name = "RedRevLaunchAndRamp", group = "Revolution")
public class RedRevLaunchAndRamp extends RevCornerBase {

    public RedRevLaunchAndRamp(){

        launchPositioningPath = new DrivePathSegment[]{

                new DrivePathSegment(10.0f),
                new DrivePathSegment(2.00f, 0.8f, DrivePathSegment.LINEAR),
                new DrivePathSegment(324.0f, 0.5f, DrivePathSegment.TURN),
                new DrivePathSegment(11.0f, 0.8f, DrivePathSegment.LINEAR),
                new DrivePathSegment(334.0f, 0.5f, DrivePathSegment.TURN),
                new DrivePathSegment(1.0f),
        };

        postThrowingPath = new DrivePathSegment[] {

                new DrivePathSegment(4.0f, 0.8f, DrivePathSegment.LINEAR),
                new DrivePathSegment(-7.0f, 0.8f, DrivePathSegment.HOLONOMIC),
                new DrivePathSegment(8.0f, 0.8f, DrivePathSegment.LINEAR),
                new DrivePathSegment(260.0f, 0.5f, DrivePathSegment.TURN),
                new DrivePathSegment(22.0f, 0.8f, DrivePathSegment.LINEAR),
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
