package org.firstinspires.ftc.teamcode;

        import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * Created by 4924_Users on 10/22/2016.
 */

@Autonomous(name = "RedRampLaunchAndBeacon")
public class RedRampLaunchAndBeacon extends LaunchAndBeaconBase {

    public RedRampLaunchAndBeacon() {

        launchPositioningPath = new DrivePathSegment[]{

                new DrivePathSegment(0.0f, 0.0f, 0.0f),
        };

        beaconPath = new DrivePathSegment[]{

                new DrivePathSegment(40.0f, 40.0f, 1.0f),
                new DrivePathSegment(-40.0f, -40.0f, 1.0f),
        };
    }
}