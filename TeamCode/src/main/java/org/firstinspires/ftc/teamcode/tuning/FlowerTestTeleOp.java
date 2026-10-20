package org.firstinspires.ftc.teamcode.tuning;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.subsystems.Ramp;

/**
 * Finds the FLOWER ramp's servo positions. Starts from Ramp's defaults; copy the values it shows into Ramp.java.
 *
 * <p>A: closed (up) / open (down). D-pad up/down: flower1. D-pad right/left: flower2. Each moves the position of
 * whichever state the ramp is in now. RB/LB: bigger/smaller step. Y: back to the defaults.
 */
@Disabled
@TeleOp(name = "Flower Test", group = "Tuning")
public class FlowerTestTeleOp extends OpMode {
    private static final double[] STEPS = {0.001, 0.005, 0.01, 0.05};

    private Servo flower1;
    private Servo flower2;
    private boolean open;
    private int step = 2;
    private double firstClosed = Ramp.FIRST_UP;
    private double firstOpen = Ramp.FIRST_DOWN;
    private double secondClosed = Ramp.SECOND_UP;
    private double secondOpen = Ramp.SECOND_DOWN;

    @Override
    public void init() {
        flower1 = hardwareMap.get(Servo.class, "flower1");
        flower2 = hardwareMap.tryGet(Servo.class, "flower2");
        apply();
    }

    @Override
    public void loop() {
        if (gamepad1.aWasPressed()) {
            open = !open;
        }
        if (gamepad1.rightBumperWasPressed()) {
            step = Math.min(STEPS.length - 1, step + 1);
        }
        if (gamepad1.leftBumperWasPressed()) {
            step = Math.max(0, step - 1);
        }
        if (gamepad1.yWasPressed()) {
            firstClosed = Ramp.FIRST_UP;
            firstOpen = Ramp.FIRST_DOWN;
            secondClosed = Ramp.SECOND_UP;
            secondOpen = Ramp.SECOND_DOWN;
        }

        double delta = STEPS[step];
        double first = gamepad1.dpadUpWasPressed() ? delta : gamepad1.dpadDownWasPressed() ? -delta : 0;
        double second = gamepad1.dpadRightWasPressed() ? delta : gamepad1.dpadLeftWasPressed() ? -delta : 0;
        if (open) {
            firstOpen = clip(firstOpen + first);
            secondOpen = clip(secondOpen + second);
        } else {
            firstClosed = clip(firstClosed + first);
            secondClosed = clip(secondClosed + second);
        }
        apply();

        telemetry.addData("Ramp (A)", open ? "OPEN (down)" : "CLOSED (up)");
        telemetry.addData("Step (RB/LB)", "%.3f", delta);
        telemetry.addLine();
        telemetry.addLine("            closed   open");
        telemetry.addLine(String.format("flower1    %.3f    %.3f   (d-pad up/down)", firstClosed, firstOpen));
        telemetry.addLine(String.format("flower2    %.3f    %.3f   (d-pad right/left)%s", secondClosed, secondOpen,
                flower2 == null ? "  NOT CONFIGURED" : ""));
        telemetry.addLine();
        telemetry.addLine(String.format("Defaults:  flower1 %.3f / %.3f, flower2 %.3f / %.3f (Y resets)",
                Ramp.FIRST_UP, Ramp.FIRST_DOWN, Ramp.SECOND_UP, Ramp.SECOND_DOWN));
        telemetry.update();
    }

    private void apply() {
        flower1.setPosition(open ? firstOpen : firstClosed);
        if (flower2 != null) {
            flower2.setPosition(open ? secondOpen : secondClosed);
        }
    }

    private static double clip(double position) {
        return Math.max(0, Math.min(1, position));
    }
}
