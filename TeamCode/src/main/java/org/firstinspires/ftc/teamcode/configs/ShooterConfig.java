package org.firstinspires.ftc.teamcode.configs;

import com.bylazar.configurables.annotations.Configurable;

@Configurable
public class ShooterConfig {
    // Shooter and Feeder Tunables
    public static double SHOOTER_VEL_LONG = 1605.0;
    public static double SHOOTER_VEL_SHORT = 1200.0;
    public static double SIDE_POWER = -0.145;
    public static double START_WAIT_TIME = 2.0;
    public static double WAIT_TIME = 10.5;
    
    // Intake and Handoff Tunables
    public static double INTAKE_POWER = 0.75;
    public static double HANDOFF_DISTANCE_MM = 107.0;
}
