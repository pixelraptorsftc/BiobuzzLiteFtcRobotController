package org.firstinspires.ftc.teamcode.tuning.pedroPathing;

import com.pedropathing.tuning.autotune.Procedure;
import com.pedropathing.tuning.autotune.Tuner;

import org.firstinspires.ftc.teamcode.common.Constants;
import org.firstinspires.ftc.teamcode.tuning.pedroPathing.procedures.ForesightTuner;
import org.firstinspires.ftc.teamcode.tuning.pedroPathing.procedures.MecanumTuner;
import org.firstinspires.ftc.teamcode.tuning.pedroPathing.procedures.PinpointTuner;
import org.firstinspires.ftc.teamcode.tuning.pedroPathing.procedures.Tests;

/**
 * Pedro's auto-tuners, in the tuning web page the Robot Controller serves. Run them in order and paste each block
 * into common/Constants.java. ForesightTuner has our safety patch (NaN checks, hard stops on the braking steps):
 * keep it when updating from Pedro's quickstart.
 */
public final class Tuning {
    private Tuning() {}

    @Tuner(name = "1. Mecanum Drivetrain")
    public static Procedure mecanum() {
        return new MecanumTuner();
    }

    @Tuner(name = "2. Pinpoint Localizer")
    public static Procedure pinpoint() {
        return new PinpointTuner();
    }

    @Tuner(name = "3. Foresight Algorithm")
    public static Procedure foresight() {
        return new ForesightTuner(Constants::localizer, Constants::drivetrain);
    }

    @Tuner(name = "4. Follower Tests")
    public static Procedure tests() {
        return new Tests(Constants::drivetrain, Constants::localizer, Constants::algorithm);
    }
}
