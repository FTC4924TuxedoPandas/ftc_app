package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * Created by 4924_Users on 12/17/2016.
 */

@Autonomous(name = "BlueLaunchTwoAndCapBall")
public class BlueLaunchTwoAndCapBall extends LaunchTwoAndCapBallBase {

    public BlueLaunchTwoAndCapBall (){
        launchPositioningPath = new DrivePathSegment[]{

                //new DrivePathSegment(26.0f, 1.0f, DrivePathSegment.LINEAR),
                new DrivePathSegment(45.0f, 0.2f, DrivePathSegment.TURN),
        };
        knockCapBallPath = new DrivePathSegment[]{
                new DrivePathSegment(0.0f, 0.0f, DrivePathSegment.LINEAR),

        };

    }
    @Override
    public boolean isRed() {

        return false;
    }
}
