package org.firstinspires.ftc.teamcode.auto.routines;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.common.RobotState;

@Autonomous(name = "Red Park only, Garden wall, tile 3", group = "Red")
public class RedParkOnlyGardenWall extends ParkOnly {
    public RedParkOnlyGardenWall() {
        super(RobotState.Alliance.RED, true);
    }
}
