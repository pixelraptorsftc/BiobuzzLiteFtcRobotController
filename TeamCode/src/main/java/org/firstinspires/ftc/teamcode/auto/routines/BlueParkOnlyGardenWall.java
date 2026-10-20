package org.firstinspires.ftc.teamcode.auto.routines;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.common.RobotState;

@Autonomous(name = "Blue Park only, Garden wall, tile 4", group = "Blue")
public class BlueParkOnlyGardenWall extends ParkOnly {
    public BlueParkOnlyGardenWall() {
        super(RobotState.Alliance.BLUE, true);
    }
}
