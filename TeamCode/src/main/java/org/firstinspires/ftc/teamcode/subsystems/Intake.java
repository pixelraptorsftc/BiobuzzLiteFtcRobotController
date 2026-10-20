package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Intake {
    public static final double INTAKE_POWER = 1.0;
    public static final double EJECT_POWER = -0.8;

    private final DcMotor motor;
    private double power;

    public Intake(HardwareMap hardwareMap) {
        motor = hardwareMap.get(DcMotor.class, "intake");
        motor.setDirection(DcMotorSimple.Direction.FORWARD);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    /** -1 to 1, positive inward. */
    public void setPower(double power) {
        this.power = power;
        motor.setPower(power);
    }

    public double getPower() {
        return power;
    }

    public void stop() {
        setPower(0);
    }
}
