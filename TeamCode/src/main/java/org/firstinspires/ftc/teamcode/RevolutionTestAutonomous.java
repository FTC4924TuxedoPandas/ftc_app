package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * Created by 4924_Users on 1/3/2017.
 */

@Autonomous(name = "RevolutionTestAutonomous")
public class RevolutionTestAutonomous extends RevolutionTestAutonomousBase {

    public RevolutionTestAutonomous() {

        beaconPath = new DrivePathSegment[] {

                new DrivePathSegment(10.0f, 0.2f, DrivePathSegment.LINEAR),
                new DrivePathSegment(-10.0f, 0.2f, DrivePathSegment.LINEAR),
        };
    }

    @Override
    public boolean isRed() {

        return false;
    }

    @Override
    public float throwingPower() {

        return 0.9f;
    }
}
