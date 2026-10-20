package org.firstinspires.ftc.teamcode.auto.routines;

import com.pedropathing.math.Pose;

import org.firstinspires.ftc.teamcode.auto.AutoRoutine;
import org.firstinspires.ftc.teamcode.common.Constants;
import org.firstinspires.ftc.teamcode.common.RobotState;

/** Only parks in our LOADING ZONE, from the garden wall's start or the flower wall's. */
public abstract class ParkOnly extends AutoRoutine {
    private final boolean fromGardenWall;

    protected ParkOnly(RobotState.Alliance alliance, boolean fromGardenWall) {
        super(alliance);
        this.fromGardenWall = fromGardenWall;
    }

    @Override
    protected Pose startPose() {
        return fromGardenWall ? pose(54.75, 8.5, 90.0) : pose(30.75, 133.0, -90.0);
    }

    @Override
    protected Pose park() {
        Pose red = Constants.RED_PARK;
        return pose(red.x(), red.y(), Math.toDegrees(red.heading()));
    }

    @Override
    protected void buildRoute() {
        // From the garden wall, bowed out toward our wall to keep clear of the HIVE frame.
        leg("To our LOADING ZONE", fromGardenWall ? curve(startPose(), park(), 20.0, 50.0) : line(startPose(), park()));
    }
}
