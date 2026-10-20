package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

/** The FLOWER ramp on flower1 and flower2. The servos are mirrored, so they move opposite ways. */
public class Ramp {
    public static final double FIRST_UP = 0.740;
    public static final double FIRST_DOWN = 0.190;
    public static final double SECOND_UP = 0.350;
    public static final double SECOND_DOWN = 0.900;

    private final Servo first;
    private final Servo second;
    private boolean down;

    public Ramp(HardwareMap hardwareMap) {
        first = hardwareMap.get(Servo.class, "flower1");
        second = hardwareMap.tryGet(Servo.class, "flower2");
        apply();
    }

    public void setDown(boolean down) {
        if (down != this.down) {
            this.down = down;
            apply();
        }
    }

    public boolean isDown() {
        return down;
    }

    private void apply() {
        first.setPosition(down ? FIRST_DOWN : FIRST_UP);
        if (second != null) {
            second.setPosition(down ? SECOND_DOWN : SECOND_UP);
        }
    }
}
