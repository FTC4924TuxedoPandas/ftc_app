package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * Created by 4924_Users on 11/25/2016.
 */

@Autonomous(name = "BeaconOnly")
public class BeaconOnly extends BeaconOnlyBase {

    public BeaconOnly() {

        launchPositioningPath = new DrivePathSegment[]{

                new DrivePathSegment(0.0f, 0.0f, DrivePathSegment.LINEAR),
        };

        beaconPath = new DrivePathSegment[]{

                new DrivePathSegment(0.0f, 0.0f, DrivePathSegment.LINEAR),
        };
    }

    @Override
    public boolean isRed() {
        return false;
    }
}
