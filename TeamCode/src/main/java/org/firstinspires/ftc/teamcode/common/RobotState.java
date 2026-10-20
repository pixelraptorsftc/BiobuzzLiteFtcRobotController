package org.firstinspires.ftc.teamcode.common;

import com.pedropathing.math.Pose;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.File;
import java.io.FileReader;
import java.io.FileWriter;
import java.io.IOException;
import java.io.Reader;
import java.io.Writer;
import java.util.Properties;

/**
 * Where AUTO left the robot, for TeleOp. Kept in a static and, in case the app restarts between them, a file
 * (/sdcard/FIRST/biobuzz/pose-handoff.properties). Older than {@link #MAX_AGE_MS} counts as none.
 */
public final class RobotState {
    private RobotState() {}

    public enum Alliance { RED, BLUE }

    public static final long MAX_AGE_MS = 5 * 60 * 1000;
    public static final long WRITE_EVERY_MS = 200;
    private static final File FILE = new File(AppUtil.FIRST_FOLDER, "biobuzz/pose-handoff.properties");

    private static Pose pose;
    private static Alliance alliance;
    private static long savedAtMs;
    private static long writtenAtMs;
    private static String source = "none";

    /** Call every AUTO loop. */
    public static void save(Pose robotPose, Alliance robotAlliance) {
        if (!Double.isFinite(robotPose.x()) || !Double.isFinite(robotPose.y()) || !Double.isFinite(robotPose.heading())) {
            return;
        }
        pose = robotPose;
        alliance = robotAlliance;
        savedAtMs = System.currentTimeMillis();
        if (savedAtMs - writtenAtMs >= WRITE_EVERY_MS) {
            writeFile();
        }
    }

    /** Writes the last saved pose. Call in AUTO's stop(): after Stop the sensors read zero. */
    public static void writeFile() {
        if (pose == null) {
            return;
        }
        writtenAtMs = System.currentTimeMillis();
        Properties p = new Properties();
        p.setProperty("x", Double.toString(pose.x()));
        p.setProperty("y", Double.toString(pose.y()));
        p.setProperty("heading", Double.toString(pose.heading()));
        p.setProperty("alliance", alliance.name());
        p.setProperty("savedAtMs", Long.toString(savedAtMs));
        File folder = FILE.getParentFile();
        if (folder != null && !folder.isDirectory() && !folder.mkdirs()) {
            return;
        }
        try (Writer out = new FileWriter(FILE)) {
            p.store(out, "BIOBUZZ pose handoff");
        } catch (IOException ignored) {
            // TeleOp still has the static.
        }
    }

    /** Forgets the pose: AUTO at init, TeleOp once it has used it. */
    public static void clear() {
        pose = null;
        alliance = null;
        source = "none";
        //noinspection ResultOfMethodCallIgnored
        FILE.delete();
    }

    /** Takes the fresher of the static and the file. Returns false if neither is fresh. */
    public static boolean resolve() {
        long now = System.currentTimeMillis();
        if (pose != null && now - savedAtMs <= MAX_AGE_MS) {
            source = "memory";
            return true;
        }
        Properties p = new Properties();
        try (Reader in = new FileReader(FILE)) {
            p.load(in);
            long at = Long.parseLong(p.getProperty("savedAtMs"));
            if (now - at > MAX_AGE_MS) {
                return false;
            }
            pose = new Pose(Double.parseDouble(p.getProperty("x")), Double.parseDouble(p.getProperty("y")),
                    Double.parseDouble(p.getProperty("heading")));
            alliance = Alliance.valueOf(p.getProperty("alliance"));
            savedAtMs = at;
            source = "file";
            return true;
        } catch (IOException | RuntimeException e) {
            return false;
        }
    }

    public static Pose getRobotPose() {
        return pose;
    }

    public static Alliance getAlliance() {
        return alliance;
    }

    public static String getResolveSource() {
        return source;
    }
}
