package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * Created by 4924_Users on 1/6/2017.
 */
@Autonomous(name = "RedRevLaunch", group = "Revolution")
public class RedRevLaunch extends RevCornerBase {

    public RedRevLaunch() {

        launchPositioningPath = new DrivePathSegment[]{

                new DrivePathSegment(10.0f),
                new DrivePathSegment(2.00f, 0.3f, DrivePathSegment.LINEAR),
                new DrivePathSegment(324.0f, 0.3f, DrivePathSegment.TURN),
                new DrivePathSegment(14.0f, 0.3f, DrivePathSegment.LINEAR),
                new DrivePathSegment(330.0f, 0.3f, DrivePathSegment.TURN),
                new DrivePathSegment(1.0f),
        };
        
        postThrowingPath = new DrivePathSegment[] {

                new DrivePathSegment(-5.0f, 0.3f, DrivePathSegment.LINEAR),
        };

        stop = new DrivePathSegment [] {

                new DrivePathSegment(0.0f, 0.0f, DrivePathSegment.LINEAR),
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
