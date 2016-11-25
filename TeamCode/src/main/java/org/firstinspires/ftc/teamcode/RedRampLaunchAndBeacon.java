package org.firstinspires.ftc.teamcode;

        import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * Created by 4924_Users on 10/22/2016.
 */

@Autonomous(name = "RedRampLaunchAndBeacon")
public class RedRampLaunchAndBeacon extends LaunchTwiceAndClaimBeacon {

    public RedRampLaunchAndBeacon() {

        launchPositioningPath = new DrivePathSegment[]{

                new DrivePathSegment(12.0f, 1.0f, DrivePathSegment.LINEAR),
        };

        beaconPath = new DrivePathSegment[]{

                new DrivePathSegment(12.0f, 1.0f, DrivePathSegment.LINEAR),
                new DrivePathSegment(315.0f, 0.2f, DrivePathSegment.TURN),
                new DrivePathSegment(40.0f, 1.0f, DrivePathSegment.LINEAR),
                new DrivePathSegment(270.0f, 0.2f, DrivePathSegment.TURN),
                new DrivePathSegment(32.0f, 1.0f, DrivePathSegment.LINEAR),
        };
    }


    @Override
    public boolean isRed() {

        return true;
    }
}
