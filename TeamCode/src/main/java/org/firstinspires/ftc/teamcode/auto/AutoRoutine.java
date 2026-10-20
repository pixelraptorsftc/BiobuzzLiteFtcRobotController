package org.firstinspires.ftc.teamcode.auto;

import com.pedropathing.api.Paths;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.common.Constants;
import org.firstinspires.ftc.teamcode.common.RobotState;
import org.firstinspires.ftc.teamcode.subsystems.Intake;

import java.util.ArrayList;
import java.util.List;

/**
 * An AUTO: legs driven one after another, with actions part-way along a leg ({@link Leg#at}) and steps after it
 * ends ({@link Leg#thenLaunch}, {@link Leg#thenWait}, {@link Leg#atEnd}). Poses are written for RED and turned for
 * BLUE ({@link #pose}). If the routine runs late, it drops what it is doing in time to drive to {@link #park()}
 * by {@link #parkBySeconds()}. Every loop it saves the pose for TeleOp.
 */
public abstract class AutoRoutine extends OpMode {
    public static double FACING_DEGREES = 3;
    public static double FACING_MS = 1000;
    public static double READY_TIMEOUT_MS = 1500;
    public static double SECONDS_BETWEEN_SHOTS = 0.3;
    /** Placeholders: the speeds park-by plans the drive to the park with, kept slow because late is a lost park. */
    public static double PARK_INCHES_PER_SECOND = 30;
    public static double PARK_DEGREES_PER_SECOND = 108;
    public static double SPARE_SECONDS = 1.5;

    protected final RobotState.Alliance alliance;
    protected Follower follower;
    protected Robot robot;

    private final List<Leg> legs = new ArrayList<>();
    private final ElapsedTime autoTimer = new ElapsedTime();
    private final ElapsedTime legTimer = new ElapsedTime();
    private enum Stage { DRIVING, STEPS, PARKING, DONE }
    private Stage stage = Stage.DONE;
    private int legIndex;
    private int stepIndex;

    protected AutoRoutine(RobotState.Alliance alliance) {
        this.alliance = alliance;
    }

    /** Where the robot is placed, from {@link #pose}. */
    protected abstract Pose startPose();

    /** Adds the legs, from {@link #leg}. */
    protected abstract void buildRoute();

    /** Where it must be parked by {@link #parkBySeconds()}; null for no deadline. */
    protected Pose park() {
        return null;
    }

    protected double parkBySeconds() {
        return 30;
    }

    /** A RED-side pose, x and y inches, heading degrees, for our alliance. */
    protected Pose pose(double x, double y, double headingDeg) {
        return Constants.pose(alliance, x, y, headingDeg);
    }

    protected Path line(Pose from, Pose to) {
        return Paths.line(from, to).linear(from, to);
    }

    /** A Bézier curve through control points (x, y, RED-side) from {@code from} to {@code to}. */
    protected Path curve(Pose from, Pose to, double... controlXY) {
        Pose[] points = new Pose[controlXY.length / 2 + 2];
        points[0] = from;
        for (int i = 0; i < controlXY.length / 2; i++) {
            points[i + 1] = pose(controlXY[2 * i], controlXY[2 * i + 1], 0);
        }
        points[points.length - 1] = to;
        return Paths.curve(points).linear(from, to);
    }

    protected Leg leg(String name, Path path) {
        Leg leg = new Leg(name, path);
        legs.add(leg);
        return leg;
    }

    @Override
    public void init() {
        RobotState.clear();
        follower = Constants.createFollower(hardwareMap);
        robot = new Robot(hardwareMap);
        buildRoute();
    }

    @Override
    public void init_loop() {
        follower.setPose(startPose());
        telemetry.addData("Place the robot", "%s, %s", alliance, startPose());
        if (alliance == RobotState.Alliance.RED ? startPose().x() >= Constants.CENTER : startPose().x() <= Constants.CENTER) {
            telemetry.addLine(">>> START POSE IS ON THE WRONG HALF <<<");
        }
        telemetry.update();
    }

    @Override
    public void start() {
        autoTimer.reset();
        startLeg(0);
    }

    @Override
    public void loop() {
        follower.update();
        Pose pose = follower.pose();
        if (shouldLeaveToPark(pose)) {
            leaveToPark(pose);
        }
        switch (stage) {
            case DRIVING:
                drive();
                break;
            case STEPS:
                runSteps();
                break;
            case PARKING:
                if (!follower.following() || follower.atParametricEnd()) {
                    follower.hold(park());
                    stage = Stage.DONE;
                }
                break;
            case DONE:
                break;
        }
        RobotState.save(follower.pose(), alliance);

        telemetry.addData("Time", "%.1f s", autoTimer.seconds());
        telemetry.addData("Stage", stage == Stage.DRIVING || stage == Stage.STEPS
                ? stage + ": " + legs.get(legIndex).name : stage.toString());
        telemetry.addLine(String.format("Pose: x=%.1f y=%.1f h=%.0f°", pose.x(), pose.y(), Math.toDegrees(pose.heading())));
        telemetry.addData("Flywheel", "%.0f / %.0f rpm", robot.shooter.getRpm(), robot.shooter.getTargetRpm());
        telemetry.update();
    }

    @Override
    public void stop() {
        robot.stop();
        follower.stop();
        // The last pose saved, not a new read: once Stop is pressed the hub's sensors read zero.
        RobotState.writeFile();
    }

    private void startLeg(int index) {
        if (index >= legs.size()) {
            if (!legs.isEmpty()) {
                follower.hold(legs.get(legs.size() - 1).path.endPose());
            }
            stage = Stage.DONE;
            return;
        }
        legIndex = index;
        Leg leg = legs.get(index);
        if (leg.launches()) {
            robot.aimShooter(leg.path.endPose());
        }
        follower.follow(leg.path);
        legTimer.reset();
        stage = Stage.DRIVING;
    }

    private void drive() {
        Leg leg = legs.get(legIndex);
        double length = leg.path.curve.length();
        double progress = length <= 0 ? 1 : 1 - follower.remainingDistance() / length;
        boolean ended = !follower.following() || follower.atParametricEnd() || legTimer.seconds() > leg.timeoutSeconds();
        for (Mark mark : leg.marks) {
            if (!mark.fired && (ended || progress >= mark.fraction)) {
                mark.fired = true;
                mark.action.run();
            }
        }
        if (ended) {
            stepIndex = 0;
            stage = Stage.STEPS;
            runSteps();
        }
    }

    private void runSteps() {
        Leg leg = legs.get(legIndex);
        while (stepIndex < leg.steps.size()) {
            if (!leg.steps.get(stepIndex).update()) {
                return;
            }
            stepIndex++;
        }
        startLeg(legIndex + 1);
    }

    /** Late enough that the drive to the park, at the cautious speeds, would only just make it. */
    private boolean shouldLeaveToPark(Pose pose) {
        Pose park = park();
        if (park == null || stage == Stage.PARKING || stage == Stage.DONE) {
            return false;
        }
        Leg leg = legs.get(legIndex);
        if (legIndex == legs.size() - 1 && stage == Stage.DRIVING && leg.path.endPose().distance(park) < 1) {
            return false;
        }
        double turn = Math.abs(Math.toDegrees(Constants.wrap(park.heading() - pose.heading())));
        double seconds = Math.max(pose.distance(park) / PARK_INCHES_PER_SECOND, turn / PARK_DEGREES_PER_SECOND);
        return autoTimer.seconds() >= parkBySeconds() - seconds - SPARE_SECONDS;
    }

    private void leaveToPark(Pose pose) {
        robot.stop();
        robot.setRampDown(false);
        follower.follow(line(pose, park()));
        stage = Stage.PARKING;
    }

    protected final class Leg {
        final String name;
        final Path path;
        final List<Mark> marks = new ArrayList<>();
        final List<Step> steps = new ArrayList<>();

        Leg(String name, Path path) {
            this.name = name;
            this.path = path;
        }

        /** Runs {@code action} once the robot is {@code fraction} (0 to 1) of the way along; the leg carries on. */
        public Leg at(double fraction, Runnable action) {
            marks.add(new Mark(fraction, action));
            return this;
        }

        public Leg atStart(Runnable action) {
            return at(0, action);
        }

        /** Runs {@code action} when the leg ends, after the steps added before it. */
        public Leg atEnd(Runnable action) {
            steps.add(() -> {
                action.run();
                return true;
            });
            return this;
        }

        /** When the leg ends: turns to the CELL, spins up, opens the gate and feeds {@code shots}. The next leg waits. */
        public Leg thenLaunch(int shots) {
            steps.add(new Launch(shots));
            return this;
        }

        public Leg thenWait(long ms) {
            ElapsedTime timer = new ElapsedTime();
            boolean[] started = {false};
            steps.add(() -> {
                if (!started[0]) {
                    started[0] = true;
                    timer.reset();
                }
                return timer.milliseconds() >= ms;
            });
            return this;
        }

        boolean launches() {
            for (Step step : steps) {
                if (step instanceof Launch) {
                    return true;
                }
            }
            return false;
        }

        double timeoutSeconds() {
            return 2 + path.curve.length() / 20;
        }
    }

    private static final class Mark {
        final double fraction;
        final Runnable action;
        boolean fired;

        Mark(double fraction, Runnable action) {
            this.fraction = fraction;
            this.action = action;
        }
    }

    /** A step after a leg: true once it is done. */
    private interface Step {
        boolean update();
    }

    /** Flywheel and hood first, then the door, then the intake, as TeleOp's LB shot. */
    private final class Launch implements Step {
        private final int shots;
        private final ElapsedTime timer = new ElapsedTime();
        private int state;
        private double facing;

        Launch(int shots) {
            this.shots = shots;
        }

        @Override
        public boolean update() {
            Pose pose = follower.pose();
            switch (state) {
                case 0:
                    robot.aimShooter(pose);
                    facing = Robot.headingToCell(pose);
                    if (offBy(pose.heading(), facing) > FACING_DEGREES) {
                        follower.hold(new Pose(pose.x(), pose.y(), facing));
                    }
                    timer.reset();
                    state = 1;
                    return false;
                case 1:
                    if (offBy(pose.heading(), facing) <= FACING_DEGREES || timer.milliseconds() >= FACING_MS) {
                        timer.reset();
                        state = 2;
                    }
                    return false;
                case 2:
                    if (robot.shooter.isReady() || timer.milliseconds() >= READY_TIMEOUT_MS) {
                        robot.openGate();
                        state = 3;
                    }
                    return false;
                case 3:
                    if (robot.gate.isReady()) {
                        robot.setIntake(Intake.INTAKE_POWER);
                        timer.reset();
                        state = 4;
                    }
                    return false;
                default:
                    if (timer.seconds() >= (shots - 0.5) * SECONDS_BETWEEN_SHOTS) {
                        robot.stop();
                        return true;
                    }
                    return false;
            }
        }

        private double offBy(double heading, double target) {
            return Math.abs(Math.toDegrees(Constants.wrap(heading - target)));
        }
    }
}
