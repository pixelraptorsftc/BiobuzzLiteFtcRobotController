package org.firstinspires.ftc.teamcode.utils;

import com.pedropathing.follower.Follower;
import com.pedropathing.follower.ManualDrive;
import com.pedropathing.math.Velocity;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.common.Constants;

/**
 * Field-centric TeleOp driving. Sticks are measured from where they rested in INIT. While the left stick drives and
 * nothing turns the robot, it holds its heading (V1's front leads when it strafes). Aligning turns the nearer end
 * toward the way the stick drives, since a mecanum drives fastest along its length.
 */
public class DriveAssist {
    public static double SLOW_SCALE = 0.35;
    public static double DEADBAND = 0.08;
    public static double SUSPECT_REST = 0.15;

    public static double ALIGN_MIN_STICK = 0.3;
    public static double ALIGN_GAIN = 1.5;
    public static double ALIGN_MAX_TURN = 0.4;
    public static double ALIGN_AGAIN = Math.toRadians(45);
    public static double HOLD_LINE_GAIN = 2.0;
    public static double HOLD_LINE_MAX = Math.toRadians(30);
    public static double HOLD_LINE_MIN_SPEED = 5;

    public static double HEADING_HOLD_GAIN = 2.5;
    public static double HEADING_HOLD_MAX_TURN = 0.5;
    public static double HEADING_HOLD_SETTLED = Math.toRadians(30);
    public static double HEADING_HOLD_SETTLE_SECONDS = 0.3;

    private final double[] rest = new double[3];
    private boolean aligning;
    private Double turnedByHandAt;
    private Double heldHeading;
    private double lastTurnedAt = Double.NEGATIVE_INFINITY;
    private double forwardHeading;

    /** Call every INIT loop with the sticks let go. */
    public void zeroSticks(Gamepad gamepad) {
        rest[0] = gamepad.left_stick_x;
        rest[1] = gamepad.left_stick_y;
        rest[2] = gamepad.right_stick_x;
    }

    public String stickWarning() {
        String[] names = {"Left stick X", "Left stick Y", "Right stick X"};
        for (int i = 0; i < rest.length; i++) {
            if (Math.abs(rest[i]) > SUSPECT_REST) {
                return String.format("%s rests at %.2f, not 0: check the gamepad. Driving is measured from there.",
                        names[i], rest[i]);
            }
        }
        return null;
    }

    /** The field heading "up" on the left stick drives toward. */
    public void setForward(double heading) {
        forwardHeading = heading;
    }

    public void toggleAligning() {
        aligning = !aligning;
        turnedByHandAt = null;
    }

    public boolean isAligning() {
        return aligning;
    }

    /** {@code aimTurn}: a turn that wins over the sticks and isn't slowed (LB's shot), or NaN. */
    public void drive(Follower follower, Gamepad gamepad, double aimTurn) {
        double scale = gamepad.left_stick_button ? SLOW_SCALE : 1.0;
        double forward = -fromRest(gamepad.left_stick_y, rest[1]);
        double strafe = -fromRest(gamepad.left_stick_x, rest[0]);
        double turn = -fromRest(gamepad.right_stick_x, rest[2]);
        double stickSize = Math.min(1, Math.hypot(forward, strafe));
        double heading = follower.pose().heading();
        double now = System.nanoTime() / 1e9;

        boolean turned = turn != 0;
        double turnScale = scale;
        if (!Double.isNaN(aimTurn)) {
            turn = aimTurn;
            turned = true;
            turnScale = 1;
        } else if (aligning) {
            double aligned = alignedTurn(forward, strafe, turn, heading);
            turned |= aligned != 0;
            turn = aligned;
        }
        if (!turned) {
            turn = heldTurn(follower, stickSize, heading, now);
        }
        if (turned || stickSize == 0) {
            heldHeading = null;
            lastTurnedAt = now;
        }
        if (aligning && stickSize >= ALIGN_MIN_STICK) {
            Velocity v = follower.velocity();
            double steer = holdLineSteer(forwardHeading + Math.atan2(strafe, forward), v.vx, v.vy);
            double steeredForward = forward * Math.cos(steer) - strafe * Math.sin(steer);
            strafe = forward * Math.sin(steer) + strafe * Math.cos(steer);
            forward = steeredForward;
        }
        follower.manual(ManualDrive.fieldCentric(forward * scale, strafe * scale, turn * turnScale,
                heading - forwardHeading));
    }

    private double heldTurn(Follower follower, double stickSize, double heading, double now) {
        if (stickSize == 0) {
            return 0;
        }
        if (heldHeading == null) {
            boolean settled = Math.abs(follower.velocity().omega) < HEADING_HOLD_SETTLED
                    || now - lastTurnedAt >= HEADING_HOLD_SETTLE_SECONDS;
            if (!settled) {
                return 0;
            }
            heldHeading = heading;
        }
        return clip(HEADING_HOLD_GAIN * Constants.wrap(heldHeading - heading), HEADING_HOLD_MAX_TURN);
    }

    private double alignedTurn(double forward, double strafe, double driverTurn, double heading) {
        double travel = forwardHeading + Math.atan2(strafe, forward);
        if (driverTurn != 0) {
            turnedByHandAt = travel;
            return driverTurn;
        }
        if (Math.hypot(forward, strafe) < ALIGN_MIN_STICK) {
            turnedByHandAt = null;
            return 0;
        }
        if (turnedByHandAt != null) {
            if (Math.abs(Constants.wrap(travel - turnedByHandAt)) < ALIGN_AGAIN) {
                return 0;
            }
            turnedByHandAt = null;
        }
        // Never more than a quarter turn: backing up doesn't spin the robot round.
        double error = Constants.wrap(travel - heading);
        if (Math.abs(error) > Math.PI / 2) {
            error = Constants.wrap(error + Math.PI);
        }
        return clip(ALIGN_GAIN * error, ALIGN_MAX_TURN);
    }

    private static double holdLineSteer(double travel, double vx, double vy) {
        if (Math.hypot(vx, vy) < HOLD_LINE_MIN_SPEED) {
            return 0;
        }
        double off = Constants.wrap(travel - Math.atan2(vy, vx));
        return Math.abs(off) > Math.PI / 2 ? 0 : clip(HOLD_LINE_GAIN * off, HOLD_LINE_MAX);
    }

    public static double fromRest(double raw, double rest) {
        double moved = raw - rest;
        double travel = moved > 0 ? 1 - rest : 1 + rest;
        if (travel <= 1e-6) {
            return 0;
        }
        double fraction = Math.min(1, Math.abs(moved) / travel);
        return fraction < DEADBAND ? 0 : Math.signum(moved) * (fraction - DEADBAND) / (1 - DEADBAND);
    }

    private static double clip(double value, double max) {
        return Math.max(-max, Math.min(max, value));
    }
}
