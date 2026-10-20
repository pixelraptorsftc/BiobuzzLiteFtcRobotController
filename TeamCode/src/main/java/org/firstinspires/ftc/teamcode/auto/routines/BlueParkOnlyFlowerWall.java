package org.firstinspires.ftc.teamcode.auto.routines;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.common.RobotState;

@Autonomous(name = "Blue Park only, Flower wall, tile 5", group = "Blue")
public class BlueParkOnlyFlowerWall extends ParkOnly {
    public BlueParkOnlyFlowerWall() {
        super(RobotState.Alliance.BLUE, false);
    }
}
