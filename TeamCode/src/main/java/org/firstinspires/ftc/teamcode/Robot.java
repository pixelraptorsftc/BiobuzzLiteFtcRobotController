package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;


import com.pedropathing.math.Pose;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.common.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Gate;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Ramp;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;

/**
 * Every mechanism but the drive, which is Pedro's {@code Follower} in each OpMode. A shot is the flywheel and hood
 * set, then the gate open, then the intake running inward.
 */
public class Robot {
    public final Intake intake;
    public final Shooter shooter;
    public final Gate gate;
    public final Ramp ramp;

    public Robot(HardwareMap hardwareMap) {
        for (LynxModule hub : hardwareMap.getAll(LynxModule.class)) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
        intake = new Intake(hardwareMap);
        shooter = new Shooter(hardwareMap);
        gate = new Gate(hardwareMap);
        ramp = new Ramp(hardwareMap);
    }

    public void setIntake(double power) {
        intake.setPower(power);
    }

    public void setRampDown(boolean down) {
        ramp.setDown(down);
    }

    public void openGate() {
        gate.setOpen(true);
    }

    public void closeGate() {
        gate.setOpen(false);
    }

    /** The CELL a launch from {@code pose} goes into: the nearer HIVE's, at the end the robot is beyond. */
    public static double[] cellOpening(Pose pose) {
        return Constants.cellOpening(Constants.hiveAt(pose.x()), Constants.farEnd(pose.y()));
    }

    public static double distanceToCell(Pose pose) {
        double[] opening = cellOpening(pose);
        return Math.hypot(opening[0] - pose.x(), opening[1] - pose.y());
    }

    /** The heading that faces that CELL's opening from {@code pose}. */
    public static double headingToCell(Pose pose) {
        double[] opening = cellOpening(pose);
        return Math.atan2(opening[1] - pose.y(), opening[0] - pose.x());
    }

    /** Flywheel and hood from the shot table for a shot from {@code pose}. */
    public void aimShooter(Pose pose) {
        shooter.aim(distanceToCell(pose));
    }

    public void stop() {
        intake.stop();
        gate.setOpen(false);
        shooter.stop();
    }
}
