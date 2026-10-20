package org.firstinspires.ftc.teamcode.auto.routines;

import com.pedropathing.math.Pose;

import org.firstinspires.ftc.teamcode.auto.AutoRoutine;
import org.firstinspires.ftc.teamcode.common.RobotState;
import org.firstinspires.ftc.teamcode.subsystems.Intake;

/**
 * From the garden wall: launch the pre-loads, take POLLEN from both FLOWERS with the ramp and from our GARDEN,
 * launching after each, then park. Poses are RED's; BLUE's are turned.
 */
public abstract class StaticGardenWall extends AutoRoutine {
    protected StaticGardenWall(RobotState.Alliance alliance) {
        super(alliance);
    }

    @Override
    protected Pose startPose() {
        return pose(54.75, 8.5, 90.0);
    }

    @Override
    protected Pose park() {
        return pose(11.0, 89.5, 0.0);
    }

    @Override
    protected void buildRoute() {
        Pose start = startPose();
        Pose leg1 = pose(58.0, 11.5, 90.0);
        Pose leg2 = pose(22.0, 46.5, 180.0);
        Pose leg3 = pose(15.0, 47.0, 180.0);
        Pose leg4 = pose(55.0, 123.0, -85.0);
        Pose leg5 = pose(47.5, 118.0, 90.0);
        Pose leg6 = pose(47.5, 126.5, 90.0);
        Pose leg7 = pose(59.5, 119.5, -90.0);
        Pose leg8 = pose(10.0, 16.5, -90.0);
        Pose leg9 = pose(8.5, 11.0, -90.0);
        Pose leg10 = pose(58.0, 12.5, 90.0);
        Pose leg11 = park();

        leg("Leg 1", line(start, leg1))
                .thenLaunch(4);
        leg("flower 1", curve(leg1, leg2, 45.7, 29.94))
                .atEnd(this::intakeOn)
                .atEnd(this::rampDown);
        leg("flower intake", line(leg2, leg3))
                .thenWait(1000);
        leg("Leg 4", curve(leg3, leg4, 22.5, 113.5))
                .at(0.2, this::rampUp)
                .at(0.5, this::intakeOff)
                .thenLaunch(4);
        leg("flower 2 approach", line(leg4, leg5))
                .atStart(this::rampDown)
                .atEnd(this::intakeOn);
        leg("flower intake", line(leg5, leg6))
                .thenWait(1000);
        leg("Launch 3", curve(leg6, leg7, 51.5, 109.5))
                .thenLaunch(4);
        leg("garden approach", curve(leg7, leg8, 5.0, 126.0, 43.5, 57.0))
                .at(0.5, this::rampDown)
                .at(0.5, this::intakeOn);
        leg("Leg 9", line(leg8, leg9))
                .thenWait(1000);
        leg("Leg 10", curve(leg9, leg10, 32.5, 31.0))
                .thenLaunch(4);
        leg("Leg 11", curve(leg10, leg11, 19.0, 41.0))
                .at(0.5, this::intakeOff)
                .at(0.5, this::rampUp);
    }

    private void intakeOn() {
        robot.setIntake(Intake.INTAKE_POWER);
    }

    private void intakeOff() {
        robot.setIntake(0);
    }

    private void rampDown() {
        robot.setRampDown(true);
    }

    private void rampUp() {
        robot.setRampDown(false);
    }
}
