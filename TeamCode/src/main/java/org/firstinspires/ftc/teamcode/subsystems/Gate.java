package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

/** The door between the intake and the flywheel: closed the intake collects, open it feeds the shots. */
public class Gate {
    public static final double OPEN = 0.20;
    public static final double CLOSED = 0.52;
    public static final double TRAVEL_SECONDS = 0.1;

    private final Servo servo;
    private double open = OPEN;
    private double closed = CLOSED;
    private boolean isOpen;
    private long sentAtNs = Long.MIN_VALUE / 2;

    public Gate(HardwareMap hardwareMap) {
        servo = hardwareMap.get(Servo.class, "door");
        servo.setPosition(closed);
    }

    public void setOpen(boolean open) {
        if (open != isOpen) {
            isOpen = open;
            sentAtNs = System.nanoTime();
            servo.setPosition(getPosition());
        }
    }

    public boolean isOpen() {
        return isOpen;
    }

    /** Open and had time to get there. */
    public boolean isReady() {
        return isOpen && (System.nanoTime() - sentAtNs) / 1e9 >= TRAVEL_SECONDS;
    }

    /** Moves the position it is at now, open or closed, to find better ones on the robot. */
    public void adjust(double step) {
        if (isOpen) {
            open = Math.max(0, Math.min(1, open + step));
        } else {
            closed = Math.max(0, Math.min(1, closed + step));
        }
        servo.setPosition(getPosition());
    }

    public double getPosition() {
        return isOpen ? open : closed;
    }
}
