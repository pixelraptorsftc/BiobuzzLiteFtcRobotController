package org.firstinspires.ftc.teamcode.common;

import com.pedropathing.algorithm.Foresight;
import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.controllers.Controller;
import com.pedropathing.drivetrain.Drivetrain;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Localizer;
import com.pedropathing.math.Matrix;
import com.pedropathing.math.Pose;
import com.pedropathing.math.Vector2D;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.drivetrains.MecanumConfig;
import com.pedropathing.revhub.localizers.PinpointConfig;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * Competition robot V1's Pedro constants and the BIOBUZZ field. Tuning order: Mecanum, Pinpoint, Foresight, Tests
 * (tuning/pedroPathing/Tuning.java); paste each tuner's block here. Keep maxBrakingPower and the heading's
 * staticFeedforward: the tuner sets neither, and without them paths stall short and headings stop a few degrees off.
 */
public final class Constants {
    private Constants() {}

    public static MecanumConfig drivetrainConfig = new MecanumConfig(c -> {
        c.frontLeftName.set("front-left");
        c.frontRightName.set("front-right");
        c.backLeftName.set("back-left");
        c.backRightName.set("back-right");
        c.frontLeftDirection.set(DcMotorSimple.Direction.REVERSE);
        c.frontRightDirection.set(DcMotorSimple.Direction.FORWARD);
        c.backLeftDirection.set(DcMotorSimple.Direction.REVERSE);
        c.backRightDirection.set(DcMotorSimple.Direction.FORWARD);
    });

    public static PinpointConfig localizerConfig = new PinpointConfig(c -> {
        c.name.set("pinpoint");
        c.podType.set(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        c.xPodOffset.set(3.91);
        c.yPodOffset.set(-7.50);
        c.offsetUnits.set(DistanceUnit.INCH);
        c.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
        c.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.REVERSED);
        c.globalDistanceUnit.set(DistanceUnit.INCH);
    });

    public static final double MAX_BRAKING_POWER = 0.35;
    public static final double HEADING_STATIC_FEEDFORWARD = 0.03;

    // Foresight Tuner on V1, 2026-10-17.
    public static ForesightConfig foresightConfig = new ForesightConfig(c -> {
        Controller primaryTranslationalForward = Controller.proportional(0.27207025724220774);
        Controller secondaryTranslationalForward = Controller.proportional(0.10052269019983145);
        Controller primaryTranslationalLateral = Controller.proportional(0.4208630337998169);
        Controller secondaryTranslationalLateral = Controller.proportional(0.15549764532165478);

        c.forwardTranslational.set(Controller.piecewise(secondaryTranslationalForward).put(2.5, primaryTranslationalForward));
        c.strafeTranslational.set(Controller.piecewise(secondaryTranslationalLateral).put(2.5, primaryTranslationalLateral));

        c.coast.set(Controller.proportionalFeedforward(0.012064132559559874));
        c.brake.set(Controller.proportionalFeedforward(0.010254512675625892));

        c.headingFeedback.set(Controller.proportional(4.3618244024071675)
                .plus(Controller.staticFeedforward(HEADING_STATIC_FEEDFORWARD)));
        c.headingBrakeCoefficients.set(Vector2D.cartesian(0.056970763586588986, 0.0045168993158977615));

        c.linearBrakeCoefficients.set(Matrix.diag(0.09183062322715634, 0.07756349080329632));
        c.quadraticBrakeCoefficients.set(Matrix.diag(0.0014153288859918508, 0.0013665482485888624));

        c.maxAchievableForwardVelocity.set(78.03474652653367);
        c.maxAchievableStrafeVelocity.set(67.24280319022276);

        c.naturalForwardDeceleration.set(33.18802062392624);
        c.naturalStrafeDeceleration.set(62.95876733393938);

        c.maxBrakingPower.set(MAX_BRAKING_POWER);
    });

    public static Drivetrain drivetrain(HardwareMap hardwareMap) {
        return new Mecanum(hardwareMap, drivetrainConfig);
    }

    public static Localizer localizer(HardwareMap hardwareMap) {
        return new PinpointLocalizer(hardwareMap, localizerConfig);
    }

    public static Foresight algorithm() {
        return new Foresight(foresightConfig);
    }

    /** Recalibrates the Pinpoint's IMU: keep the robot still for half a second. */
    public static Follower createFollower(HardwareMap hardwareMap) {
        return new Follower(localizer(hardwareMap), drivetrain(hardwareMap), algorithm());
    }

    // ---- Field (inches, RED's driver station at x = 0). BLUE is RED rotated 180° about the centre. ----

    public static final double FIELD_SIZE = 141.5;
    public static final double CENTER = FIELD_SIZE / 2;

    public static final double RED_HIVE_PIVOT_X = -12.742;
    public static final double BLUE_HIVE_PIVOT_X = 12.758;
    public static final double CELL_OPENING_Y = 15.788;
    public static final double HIVE_FRAME_HALF_DEPTH = 19.476;

    /** RED's park: 11 in off our loading wall, level with the middle of our LOADING ZONE. */
    public static final Pose RED_PARK = new Pose(11.0, CENTER + (46.599 + 23.907) / 2, 0);

    /** A RED-side pose (x, y, heading in degrees) for {@code alliance}. */
    public static Pose pose(RobotState.Alliance alliance, double x, double y, double headingDeg) {
        Pose red = new Pose(x, y, Math.toRadians(headingDeg));
        return alliance == RobotState.Alliance.RED ? red : mirror(red);
    }

    public static Pose mirror(Pose pose) {
        return new Pose(FIELD_SIZE - pose.x(), FIELD_SIZE - pose.y(), pose.heading() + Math.PI);
    }

    /** The far CELL is the one at y > CENTER. */
    public static boolean farEnd(double y) {
        return y > CENTER;
    }

    /** Where {@code alliance}'s CELL at the far or audience end opens when it is up. */
    public static double[] cellOpening(RobotState.Alliance alliance, boolean far) {
        double x = CENTER + (alliance == RobotState.Alliance.RED ? RED_HIVE_PIVOT_X : BLUE_HIVE_PIVOT_X);
        double y = CENTER + (far ? 1 : -1) * CELL_OPENING_Y;
        return new double[] {x, y};
    }

    /** The HIVE a robot at x launches into: ours is in our half. */
    public static RobotState.Alliance hiveAt(double x) {
        return x > CENTER ? RobotState.Alliance.BLUE : RobotState.Alliance.RED;
    }

    /** Whether y is beyond the HIVE frame at the far or audience end, where that CELL's opening can be seen. */
    public static boolean beyondEnd(boolean far, double y) {
        return (y - CENTER) * (far ? 1 : -1) > HIVE_FRAME_HALF_DEPTH;
    }

    public static double wrap(double radians) {
        return Math.atan2(Math.sin(radians), Math.cos(radians));
    }
}
