package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * Created by 4924_Users on 11/25/2016.
 */

@Autonomous(name = "BeaconOnly")
public class BeaconOnly extends BeaconOnlyBase {

    @Override
    public boolean isRed() {

        return true;
    }
}
