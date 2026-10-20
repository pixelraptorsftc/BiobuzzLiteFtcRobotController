package org.firstinspires.ftc.teamcode.auto.routines;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.common.RobotState;

@Autonomous(name = "Red Static Garden Wall", group = "Red")
public class RedStaticGardenWall extends StaticGardenWall {
    public RedStaticGardenWall() {
        super(RobotState.Alliance.RED);
    }
}
