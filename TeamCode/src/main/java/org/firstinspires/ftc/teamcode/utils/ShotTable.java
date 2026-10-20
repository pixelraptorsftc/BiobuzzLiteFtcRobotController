package org.firstinspires.ftc.teamcode.utils;

/**
 * Flywheel rpm and hood degrees by distance from the robot's middle to the CELL's opening. Interpolated between rows,
 * the end row's beyond the ends. 41 and 46 in are measured on V1 (2026-10-17); the rest are placeholders.
 */
public final class ShotTable {
    private ShotTable() {}

    // {distance in, rpm, hood degrees}
    static final double[][] ROWS = {
            {12, 1845, 80},
            {24, 2064, 80},
            {36, 2372, 80},
            {41, 2456, 76},
            {46, 2606, 80},
            {60, 2942, 80},
            {72, 3201, 80},
            {84, 3447, 80},
            {96, 3681, 80},
            {108, 3905, 80},
            {120, 4121, 80},
    };

    /** {rpm, hood degrees}. */
    public static double[] at(double distanceInches) {
        if (distanceInches <= ROWS[0][0]) {
            return new double[] {ROWS[0][1], ROWS[0][2]};
        }
        for (int i = 1; i < ROWS.length; i++) {
            if (distanceInches <= ROWS[i][0]) {
                double f = (distanceInches - ROWS[i - 1][0]) / (ROWS[i][0] - ROWS[i - 1][0]);
                return new double[] {ROWS[i - 1][1] + f * (ROWS[i][1] - ROWS[i - 1][1]),
                        ROWS[i - 1][2] + f * (ROWS[i][2] - ROWS[i - 1][2])};
            }
        }
        double[] last = ROWS[ROWS.length - 1];
        return new double[] {last[1], last[2]};
    }
}
