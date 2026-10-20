package org.firstinspires.ftc.teamcode.auto.routines;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.common.RobotState;

@Autonomous(name = "Red Park only, Flower wall, tile 2", group = "Red")
public class RedParkOnlyFlowerWall extends ParkOnly {
    public RedParkOnlyFlowerWall() {
        super(RobotState.Alliance.RED, false);
    }
}
