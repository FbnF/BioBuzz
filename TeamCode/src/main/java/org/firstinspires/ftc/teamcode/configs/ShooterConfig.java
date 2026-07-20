package org.firstinspires.ftc.teamcode.configs;

import com.bylazar.configurables.annotations.Configurable;

@Configurable
public final class ShooterConfig {
    private ShooterConfig() {} // static-only

    // ---------------- Controls / Limits ----------------
    public static double MIN_RANGE_IN = 10.0;    // ignore ranges under this (inches)
    public static double TPS_MIN_AUTO = 800.0;   // below this, AUTO stays off
    public static double TPS_TOL = 15.0;

    public static double TPS_MAX_MECH = 2800.0;  // absolute safety limit
    public static double TPS_MAX_AUTO = 2400.0;  // clamp used by AUTO

    // ---------------- Mode Switches ----------------
    public static boolean USE_TABLE = true;
    public static boolean USE_FEED_TABLE = true;

    // Default powers for manual/auto modes
    public static double FEED_DEFAULT = -0.9;
    public static double SIDE_POWER = -1;
    public static double SIDE_DEFAULT = 1.0;
    public static double INTAKE_POWER = 0.90;

    // Timing
    public static double START_WAIT_TIME = 2.0;
    public static double WAIT_TIME = 10.5;

    // Tuning knobs for physics mode
    public static double TPS_SCALE = 1.0;
    public static double TPS_OFFSET = 0.0;

    // No-shot zone
    public static double NO_SHOT_UNDER_IN = 0.0; 

    // Smoothing (0 = off, 1 = heavy)
    public static double DIST_SMOOTH_ALPHA = 0.20;

    // Physics model params
    public static double G = 9.81;
    public static double LAUNCH_DEG = 46.0;
    public static double TARGET_H_M = 0.984;
    public static double SHOOTER_H_M = 0.248;
    public static double WHEEL_RADIUS_M = 0.048;
    public static double EFFICIENCY = 0.30;
    public static double TICKS_PER_REV = 28.0;

    // Handoff sensor
    public static double HANDOFF_DISTANCE_MM = 107.0;

    // Auto Align Config
    public static boolean AUTO_ALIGN_ENABLED = true;
    public static double ALIGN_KP = 0.02;
    public static double ALIGN_KD = 0.00045;
    public static double ALIGN_MAX_TURN = 0.7;
    public static double ALIGN_MIN_TURN = 0.08;
    public static double ALIGN_ERR_DEADBAND_DEG = 0.25;
    public static double ALIGN_MAX_STALE_MS = 100;

    // ---------------- Blue Side Tables ----------------
    public static double[] DIST_IN_BLUE = new double[] {
            45.2, 46.12, 54.4, 56.2, 60.2, 69.7, 80.5, 127.4, 133.47, 139.24
    };
    public static double[] TPS_AT_DIST_BLUE = new double[] {
            1110, 1120, 1110, 1299, 1225, 1262, 1370, 1553, 1590, 1640
    };
    public static double[] MIN_ANGLE_BLUE = new double[] {
            0.4688, 0.4688, 0.4688, 0.4688, 0.4688, 0.4688, 0.4688, 0.4688, 0.4688, 0.5688
    };
    public static double[] MAX_ANGLE_BLUE = new double[] {
            0.4688, 0.4688, 0.4688, 0.4688, 0.4688, 0.4688, 0.4688, 0.4688, 0.4688, 0.5688
    };
    public static double[] FEED_POWER_BLUE = new double[] {
            -0.9, -0.9, -0.4, -0.49, -0.6, -0.4, -0.2, -0.17, -0.15, -0.15
    };
    public static double[] SIDE_POWER_BLUE = new double[] {
            1.0, 1.0, 0.7, 1.0, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5
    };

    // ---------------- Red Side Tables ----------------
    public static double[] DIST_IN_RED = new double[] {
            45.82, 48.78, 51.15, 58.22, 62.8, 69.09, 81.5, 130.9, 136, 138.6
    };
    public static double[] TPS_AT_DIST_RED = new double[] {
            1140, 1250, 1260, 1260, 1255, 1292, 1361, 1583, 1610, 1670
    };
    public static double[] MIN_ANGLE_RED = new double[] {
            -5.06, -10, -4.35, -6.759, -4.42, -6.759, -6.759, -6.759, -5.2, -6.4
    };
    public static double[] MAX_ANGLE_RED = new double[] {
            -0.547, -0.547, -0.547, -0.547, -0.547, -0.547, -0.547, -0.547, -0.547, -0.747
    };
    public static double[] FEED_POWER_RED = new double[] {
            -0.9, -0.4, -0.25, -0.49, -0.25, -0.4, -0.2, -0.15, -0.15, -0.15
    };
    public static double[] SIDE_POWER_RED = new double[] {
            1.0, 1.0, 0.7, 1.0, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5
    };

    // Constant Velocities for Auto
    public static double SHOOTER_VEL_LONG = 1582.0;
    public static double SHOOTER_VEL_SHORT = 1165.0;

    // Red-specific offsets
    public static double RED_ALIGN_OFFSET = 1.0;

    // Blue-specific offsets
    public static double BLUE_ALIGN_OFFSET = 1.0;
}
