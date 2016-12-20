package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * Created by 4924_Users on 12/17/2016.
 */

@Autonomous(name = "BlueLaunchTwoAndCapBallAndOneBeacon") //right side of the robot should be on the edge of the 3rd tile from the ramp when starting
public class BlueLaunchTwoAndCapBallAndOneBeacon extends LaunchTwoAndCapBallAndOneBeaconBase {

    public BlueLaunchTwoAndCapBallAndOneBeacon (){
        launchPositioningPath = new DrivePathSegment[]{

                new DrivePathSegment(4.0f, 1.0f, DrivePathSegment.LINEAR),
                new DrivePathSegment(330.0f, 1.0f, DrivePathSegment.TURN),
                new DrivePathSegment(6.0f, 1.0f, DrivePathSegment.LINEAR),
                new DrivePathSegment(1.0f),
        };
        beaconPath = new DrivePathSegment[]{

                /*new DrivePathSegment(50.0f, 0.2f, DrivePathSegment.TURN),
                new DrivePathSegment(45.0f, 1.0f, DrivePathSegment.LINEAR),
                new DrivePathSegment(90.0f, 0.2f, DrivePathSegment.TURN),
                new DrivePathSegment(5.0f, 0.1f, DrivePathSegment.LINEAR), */
        };
        knockCapBallPath = new DrivePathSegment[]{

        };

    }
    @Override
    public boolean isRed() {

        return false;
    }
}
