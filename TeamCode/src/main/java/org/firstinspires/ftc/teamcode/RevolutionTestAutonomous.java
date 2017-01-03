package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * Created by 4924_Users on 1/3/2017.
 */

@Autonomous(name = "RevolutionTestAutonomous")
public class RevolutionTestAutonomous extends RevolutionTestAutonomousBase {

    public RevolutionTestAutonomous() {

        beaconPath = new DrivePathSegment[] {

                new DrivePathSegment(10.0f, 1.0f, DrivePathSegment.HOLONOMIC),
                new DrivePathSegment(90.0f, 0.2f, DrivePathSegment.TURN),
                new DrivePathSegment(-10.0f, 0.5f, DrivePathSegment.HOLONOMIC),
                new DrivePathSegment(0.0f, 0.2f, DrivePathSegment.TURN),
        };
    }

    @Override
    public boolean isRed() {

        return false;
    }
}
