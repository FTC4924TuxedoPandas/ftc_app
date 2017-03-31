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
                new DrivePathSegment(15.0f, 0.8f, DrivePathSegment.LINEAR),
                new DrivePathSegment(310.0f, 0.5f, DrivePathSegment.TURN),
                new DrivePathSegment(1.0f),
        };

        postThrowingPath = new DrivePathSegment[] {

                new DrivePathSegment(-7.0f, 0.8f, DrivePathSegment.HOLONOMIC),
                new DrivePathSegment(8.0f, 0.8f, DrivePathSegment.LINEAR),
                new DrivePathSegment(260.0f, 0.5f, DrivePathSegment.TURN),
                new DrivePathSegment(14.0f, 0.8f, DrivePathSegment.LINEAR),
                new DrivePathSegment(2.0f, 0.4f, DrivePathSegment.LINEAR),
        };
    }
    @Override
    public boolean isRed() {

        return true;
    }

    public float throwingPower() {

        return 0.9f;
    }
}
