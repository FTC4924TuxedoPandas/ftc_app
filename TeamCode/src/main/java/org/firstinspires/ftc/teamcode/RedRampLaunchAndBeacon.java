package org.firstinspires.ftc.teamcode;

        import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * Created by 4924_Users on 10/22/2016.
 */

@Autonomous(name = "RedRampLaunchAndBeacon")
public class RedRampLaunchAndBeacon extends LaunchAndBeaconBase {

    public RedRampLaunchAndBeacon() {

        launchPositioningPath = new DrivePathSegment[]{

                new DrivePathSegment(0.0f, 0.0f, DrivePathSegment.LINEAR),
        };

        beaconPath = new DrivePathSegment[]{

                new DrivePathSegment(45.0f, 1.0f, DrivePathSegment.LINEAR),
                new DrivePathSegment(-20.0f, 1.0f, DrivePathSegment.HOLONOMIC),
                new DrivePathSegment(180.0f, 1.0f, DrivePathSegment.TURN),
                new DrivePathSegment(20.0f, 1.0f, DrivePathSegment.LINEAR),
        };
    }


    @Override
    public boolean isRed() {

        return true;
    }
}
