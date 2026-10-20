package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.utils.ShotTable;

/** Flywheel on shooter1 and shooter2, hood on a servo. Hood degrees map 40°..80° to servo 0.05..0.8 (placeholder). */
public class Shooter {
    public static final double TICKS_PER_REV = 28;
    public static final double HOOD_MIN_DEGREES = 40;
    public static final double HOOD_MAX_DEGREES = 80;
    public static final double HOOD_AT_MIN = 0.05;
    public static final double HOOD_AT_MAX = 0.8;
    public static final double RPM_TOLERANCE = 50;
    public static final double HOOD_TRAVEL_SECONDS = 0.25;
    /** A hood move smaller than this doesn't restart the wait: odometry jitter would keep a shot waiting. */
    public static final double HOOD_SETTLED_DEGREES = 1;

    private final DcMotorEx[] motors;
    private final Servo hood;
    private double targetRpm;
    private double hoodDegrees = Double.NaN;
    private double hoodWaitedFor = Double.NaN;
    private long hoodSentAtNs = Long.MIN_VALUE / 2;
    private double distanceInches = Double.NaN;

    public Shooter(HardwareMap hardwareMap) {
        DcMotorEx first = hardwareMap.get(DcMotorEx.class, "shooter1");
        DcMotorEx second = hardwareMap.get(DcMotorEx.class, "shooter2");
        // Both REVERSE: FORWARD spun V1's flywheel backwards, and the two fought.
        first.setDirection(DcMotorSimple.Direction.REVERSE);
        second.setDirection(DcMotorSimple.Direction.REVERSE);
        motors = new DcMotorEx[] {first, second};
        for (DcMotorEx motor : motors) {
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        }
        hood = hardwareMap.tryGet(Servo.class, "hood");
    }

    /** Flywheel and hood from the shot table for a shot from this far from the CELL's opening. */
    public void aim(double distanceInches) {
        double[] shot = ShotTable.at(distanceInches);
        set(shot[0], shot[1]);
        this.distanceInches = distanceInches;
    }

    public void set(double rpm, double degrees) {
        targetRpm = rpm;
        distanceInches = Double.NaN;
        for (DcMotorEx motor : motors) {
            if (rpm <= 0) {
                motor.setPower(0);
            } else {
                motor.setVelocity(rpm / 60 * TICKS_PER_REV);
            }
        }
        setHood(degrees);
    }

    public void setHood(double degrees) {
        if (Double.isNaN(hoodDegrees) || Math.abs(degrees - hoodDegrees) > 1e-6) {
            hoodDegrees = degrees;
            if (hood != null) {
                hood.setPosition(hoodPosition(degrees));
            }
        }
        if (Double.isNaN(hoodWaitedFor) || Math.abs(degrees - hoodWaitedFor) >= HOOD_SETTLED_DEGREES) {
            hoodWaitedFor = degrees;
            hoodSentAtNs = System.nanoTime();
        }
    }

    public static double hoodPosition(double degrees) {
        double fraction = Math.max(0, Math.min(1, (degrees - HOOD_MIN_DEGREES) / (HOOD_MAX_DEGREES - HOOD_MIN_DEGREES)));
        return HOOD_AT_MIN + fraction * (HOOD_AT_MAX - HOOD_AT_MIN);
    }

    public void stop() {
        targetRpm = 0;
        distanceInches = Double.NaN;
        for (DcMotorEx motor : motors) {
            motor.setPower(0);
        }
    }

    /** Flywheel at speed and the hood given time to get there. */
    public boolean isReady() {
        return targetRpm > 0 && Math.abs(getRpm() - targetRpm) <= RPM_TOLERANCE
                && (System.nanoTime() - hoodSentAtNs) / 1e9 >= HOOD_TRAVEL_SECONDS;
    }

    public double getRpm() {
        double ticksPerSecond = 0;
        for (DcMotorEx motor : motors) {
            ticksPerSecond += motor.getVelocity();
        }
        return ticksPerSecond / motors.length / TICKS_PER_REV * 60;
    }

    /** One motor's rpm, 0 or 1: both should read about the same. */
    public double getRpm(int i) {
        return motors[i].getVelocity() / TICKS_PER_REV * 60;
    }

    public double getTargetRpm() {
        return targetRpm;
    }

    public double getHoodDegrees() {
        return hoodDegrees;
    }

    public double getDistanceInches() {
        return distanceInches;
    }
}
