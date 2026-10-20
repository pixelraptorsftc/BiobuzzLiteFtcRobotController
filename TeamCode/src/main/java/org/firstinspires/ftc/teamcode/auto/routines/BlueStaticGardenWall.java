package org.firstinspires.ftc.teamcode.auto.routines;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.common.RobotState;

@Autonomous(name = "Blue Static Garden Wall", group = "Blue")
public class BlueStaticGardenWall extends StaticGardenWall {
    public BlueStaticGardenWall() {
        super(RobotState.Alliance.BLUE);
    }
}
