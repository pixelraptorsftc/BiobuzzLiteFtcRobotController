package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.math.Velocity;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.common.Constants;
import org.firstinspires.ftc.teamcode.common.RobotState;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.utils.DriveAssist;

/**
 * Starts where AUTO left the robot, or from zero facing "forward".
 *
 * <p>Gamepad 1:
 * <ul>
 *   <li>Left stick: drive (field-centric). Right stick X: turn. Left stick button: slow.</li>
 *   <li>A: make the way the robot faces "forward". X: align to travel on/off.</li>
 *   <li>RT (held): spin the flywheel up to the target rpm and set the hood.</li>
 *   <li>RB (held): shoot. Spins up too, so RT needn't be held. The driver aims; once the robot is still, the gate
 *       opens and the intake feeds.</li>
 *   <li>LT: intake. LB: outtake.</li>
 *   <li>B: FLOWER ramp down/up. D-pad up/down: nudge the gate's position.</li>
 * </ul>
 * <p>Gamepad 2: X switches the shot between the shot table and the manual rpm/hood. D-pad up/down: hood. RB/LB: rpm.
 * D-pad right/left: feed power. B: shoot at our HIVE's other CELL.
 */
@TeleOp(name = "BIOBUZZ TeleOp", group = "Competition")
public class MainTeleOp extends OpMode {
    public static double SHOOT_MAX_SPEED = 6;
    public static double SHOOT_MAX_TURN_RATE = Math.toRadians(20);
    /** Where RB shoots from without a field pose: V1's furthest measured shot table row. */
    public static double NO_POSE_SHOT_INCHES = 46;

    public static double POWER_STEP = 0.05;
    public static double RPM_STEP = 100;
    public static double MAX_RPM = 6000;
    public static double HOOD_STEP_DEGREES = 0.1 / 0.75 * (Shooter.HOOD_MAX_DEGREES - Shooter.HOOD_MIN_DEGREES);
    public static double GATE_STEP = 0.01;

    private Robot robot;
    private Follower follower;
    private final DriveAssist drive = new DriveAssist();
    /** Null when there was no pose from AUTO: RB then shoots as the driver aims. */
    private RobotState.Alliance alliance;

    private Boolean chosenFar;
    private boolean manualShot = false;
    private boolean feeding = false;
    private String shot = "RB not held";
    private double feedPower = Intake.INTAKE_POWER;
    private double shotRpm = 3000;
    private double hoodDegrees = Shooter.HOOD_MIN_DEGREES;

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        robot = new Robot(hardwareMap);
        if (RobotState.resolve()) {
            alliance = RobotState.getAlliance();
            follower.setPose(RobotState.getRobotPose());
            drive.setForward(alliance == RobotState.Alliance.RED ? 0 : Math.PI);
        } else {
            follower.setPose(Pose.zero());
        }
    }

    @Override
    public void init_loop() {
        drive.zeroSticks(gamepad1);
        telemetry.addData("Start pose", alliance != null
                ? alliance + " " + RobotState.getRobotPose() + " from AUTO (" + RobotState.getResolveSource() + ")"
                : "none from AUTO: starting at zero, forward is the way the robot faces now");
        telemetry.addLine("Leave the sticks alone until start.");
        String warning = drive.stickWarning();
        if (warning != null) {
            telemetry.addLine(">>> " + warning);
        }
        telemetry.update();
    }

    @Override
    public void start() {
        RobotState.clear();
    }

    @Override
    public void loop() {
        follower.update();
        handleDrive();
        handleOperator();
        handleRampAndGate();
        if (gamepad1.right_bumper) {
            handleShoot();
        } else {
            handleIntake();
        }
        showTelemetry();
    }

    @Override
    public void stop() {
        robot.stop();
        follower.stop();
    }

    private void handleDrive() {
        if (gamepad1.aWasPressed()) {
            drive.setForward(follower.pose().heading());
        }
        if (gamepad1.xWasPressed()) {
            drive.toggleAligning();
        }
        drive.drive(follower, gamepad1, Double.NaN);
    }

    private void handleOperator() {
        if (gamepad2.xWasPressed()) {
            manualShot = !manualShot;
        }
        if (gamepad2.dpadRightWasPressed()) {
            feedPower = Math.min(1, feedPower + POWER_STEP);
        } else if (gamepad2.dpadLeftWasPressed()) {
            feedPower = Math.max(0, feedPower - POWER_STEP);
        }
        boolean hoodUp = gamepad2.dpadUpWasPressed();
        boolean hoodDown = !hoodUp && gamepad2.dpadDownWasPressed();
        boolean rpmUp = gamepad2.rightBumperWasPressed();
        boolean rpmDown = !rpmUp && gamepad2.leftBumperWasPressed();
        if (!manualShot && (hoodUp || hoodDown || rpmUp || rpmDown)) {
            // Switch to manual, starting from the shot table's last.
            manualShot = true;
            if (!Double.isNaN(robot.shooter.getHoodDegrees())) {
                hoodDegrees = robot.shooter.getHoodDegrees();
            }
            if (robot.shooter.getTargetRpm() > 0) {
                shotRpm = robot.shooter.getTargetRpm();
            }
        }
        if (hoodUp) {
            hoodDegrees = Math.min(Shooter.HOOD_MAX_DEGREES, hoodDegrees + HOOD_STEP_DEGREES);
        } else if (hoodDown) {
            hoodDegrees = Math.max(Shooter.HOOD_MIN_DEGREES, hoodDegrees - HOOD_STEP_DEGREES);
        }
        if (rpmUp) {
            shotRpm = Math.min(MAX_RPM, shotRpm + RPM_STEP);
        } else if (rpmDown) {
            shotRpm = Math.max(0, shotRpm - RPM_STEP);
        }
        if (gamepad2.bWasPressed()) {
            chosenFar = !shotFar();
        }
    }

    private void handleRampAndGate() {
        if (gamepad1.bWasPressed()) {
            robot.setRampDown(!robot.ramp.isDown());
        }
        if (gamepad1.dpadUpWasPressed()) {
            robot.gate.adjust(GATE_STEP);
        } else if (gamepad1.dpadDownWasPressed()) {
            robot.gate.adjust(-GATE_STEP);
        }
    }

    /** RB not held: gate shut, the flywheel spinning only while RT is held. */
    private void handleIntake() {
        feeding = false;
        shot = "RB not held";
        robot.closeGate();
        if (gamepad1.right_trigger > 0.1) {
            aimShooter();
        } else {
            robot.shooter.stop();
        }
        if (gamepad1.left_trigger > 0.1) {
            robot.setIntake(feedPower);
        } else if (gamepad1.left_bumper) {
            robot.setIntake(Intake.EJECT_POWER);
        } else {
            robot.setIntake(0);
        }
    }

    /** Once still, the gate opens (not waiting for rpm), then feeds until RB is let go. */
    private void handleShoot() {
        double[] opening = shotTarget();
        aimShooter();
        Pose at = follower.pose();
        Velocity moving = follower.velocity();
        if (!feeding) {
            if (opening != null && chosenFar == null && !Constants.beyondEnd(Constants.farEnd(at.y()), at.y())) {
                shot = "drive out past the end of our HIVE";
            } else if (Math.hypot(moving.vx, moving.vy) > SHOOT_MAX_SPEED || Math.abs(moving.omega) > SHOOT_MAX_TURN_RATE) {
                shot = "waiting for the robot to stop";
            } else {
                feeding = true;
            }
        }
        if (feeding) {
            robot.openGate();
            shot = robot.gate.isReady() ? "shooting" : "opening the gate";
            robot.setIntake(robot.gate.isReady() ? feedPower : 0);
        } else {
            robot.closeGate();
            robot.setIntake(0);
        }
    }

    private void aimShooter() {
        if (manualShot) {
            robot.shooter.set(shotRpm, hoodDegrees);
            return;
        }
        double[] opening = shotTarget();
        Pose at = follower.pose();
        robot.shooter.aim(opening == null ? NO_POSE_SHOT_INCHES : Math.hypot(opening[0] - at.x(), opening[1] - at.y()));
    }

    private boolean shotFar() {
        return chosenFar != null ? chosenFar : Constants.farEnd(follower.pose().y());
    }

    private double[] shotTarget() {
        return alliance == null ? null : Constants.cellOpening(alliance, shotFar());
    }

    private void showTelemetry() {
        Pose pose = follower.pose();
        telemetry.addLine(String.format("Pose: x=%.1f y=%.1f h=%.0f°", pose.x(), pose.y(), Math.toDegrees(pose.heading())));
        telemetry.addData("Alliance", alliance == null ? "unknown (no AUTO pose)" : alliance);
        telemetry.addData("Shot (RB)", shot);
        double[] opening = shotTarget();
        if (opening != null) {
            // Positive: turn left to face the CELL.
            telemetry.addData("Off the CELL", "%.0f°",
                    Math.toDegrees(Constants.wrap(Math.atan2(opening[1] - pose.y(), opening[0] - pose.x()) - pose.heading())));
        }
        telemetry.addData("CELL (gp2 B)", "%s%s", shotFar() ? "FAR" : "AUDIENCE", chosenFar != null ? ", picked" : "");
        telemetry.addData("Shot from (gp2 X)", manualShot ? "gamepad 2's rpm and hood" : "the shot table");
        telemetry.addData("Shooter target", "%.0f rpm, hood %.0f deg", robot.shooter.getTargetRpm(), robot.shooter.getHoodDegrees());
        telemetry.addData("Flywheel", "%.0f rpm (%.0f / %.0f)%s", robot.shooter.getRpm(), robot.shooter.getRpm(0),
                robot.shooter.getRpm(1), robot.shooter.isReady() ? ", at speed" : "");
        telemetry.addData("Manual rpm / hood / feed", "%.0f / %.0f / %.2f", shotRpm, hoodDegrees, feedPower);
        telemetry.addData("Align to travel (X)", drive.isAligning() ? "ON" : "off");
        telemetry.addData("Ramp (B)", robot.ramp.isDown() ? "DOWN" : "up");
        telemetry.addData("Gate", "%s at %.2f", robot.gate.isOpen() ? "open" : "closed", robot.gate.getPosition());
        telemetry.update();
    }
}
