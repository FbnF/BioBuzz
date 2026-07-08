package org.firstinspires.ftc.teamcode.NewTeleOp;

import org.firstinspires.ftc.teamcode.configs.ShooterConfig;
import org.firstinspires.ftc.teamcode.utils.Calculations;

/**
 * ShooterService - Ballistics Computer.
 * Handles TPS calculation using linear interpolation from lookup tables.
 */
public class ShooterService {

    private static ShooterService instance = null;
    private boolean isBlue = false;
    private double lastTargetTPS = 0.0;
    
    // The new "Recalculated" Ceiling
    private static final double MAX_ALLOWED_TPS = 1582.0;

    // Debug fields for telemetry
    private double physicsTPS = 0.0;
    private double tableTPS = 0.0;
    private double baseTPS = 0.0;
    private double finalTPS = 0.0;

    public static ShooterService getInstance() {
        if (instance == null) {
            instance = new ShooterService();
        }
        return instance;
    }

    public void setIsBlue(boolean blue) {
        this.isBlue = blue;
    }

    /**
     * Calculates velocity using linear interpolation.
     * Recalculated so that 1582.0 is the maximum possible value.
     */
    public double calculateVelocity(double distanceInches, boolean targetVisible) {
        if (!targetVisible || distanceInches < ShooterConfig.MIN_RANGE_IN) {
            physicsTPS = tableTPS = baseTPS = finalTPS = 0.0;
            return lastTargetTPS * 0.8; 
        }

        // 1) Physics Calculation
        physicsTPS = Calculations.computeTPSFromRangeInches(
                ShooterConfig.G,
                distanceInches,
                ShooterConfig.LAUNCH_DEG,
                ShooterConfig.SHOOTER_H_M,
                ShooterConfig.TARGET_H_M,
                ShooterConfig.WHEEL_RADIUS_M,
                ShooterConfig.EFFICIENCY,
                ShooterConfig.TICKS_PER_REV
        );
        if (!Double.isFinite(physicsTPS)) physicsTPS = 0.0;

        // 2) Table Calculation
        double[] distTable = isBlue ? ShooterConfig.DIST_IN_BLUE : ShooterConfig.DIST_IN_RED;
        double[] tpsTable = isBlue ? ShooterConfig.TPS_AT_DIST_BLUE : ShooterConfig.TPS_AT_DIST_RED;
        tableTPS = interpolate(distanceInches, distTable, tpsTable);

        // 3) Select Base
        baseTPS = ShooterConfig.USE_TABLE ? tableTPS : physicsTPS;

        // 4) Apply Factors and RECALCULATED limit
        double scaled = ShooterConfig.USE_TABLE ? baseTPS : (baseTPS * ShooterConfig.TPS_SCALE + ShooterConfig.TPS_OFFSET);
        
        // We apply the 0.96 factor, but ensure the result NEVER exceeds 1582.
        double desired = scaled * 0.96; 
        
        if (desired > MAX_ALLOWED_TPS) {
            finalTPS = MAX_ALLOWED_TPS;
        } else {
            finalTPS = desired;
        }
        
        // Also respect the Config's Auto Clamp
        finalTPS = Math.min(finalTPS, ShooterConfig.TPS_MAX_AUTO);
        
        lastTargetTPS = finalTPS;
        return finalTPS;
    }

    public double getFeedPower(double distanceInches) {
        double[] distTable = isBlue ? ShooterConfig.DIST_IN_BLUE : ShooterConfig.DIST_IN_RED;
        double[] pwrTable = isBlue ? ShooterConfig.FEED_POWER_BLUE : ShooterConfig.FEED_POWER_RED;
        return interpolate(distanceInches, distTable, pwrTable);
    }

    public double getSidePower(double distanceInches) {
        double[] distTable = isBlue ? ShooterConfig.DIST_IN_BLUE : ShooterConfig.DIST_IN_RED;
        double[] pwrTable = isBlue ? ShooterConfig.SIDE_POWER_BLUE : ShooterConfig.SIDE_POWER_RED;
        return interpolate(distanceInches, distTable, pwrTable);
    }

    private double interpolate(double x, double[] xTable, double[] yTable) {
        if (xTable == null || yTable == null || xTable.length < 2) return 0.0;
        if (x <= xTable[0]) return yTable[0];
        int last = xTable.length - 1;
        if (x >= xTable[last]) return yTable[last];

        int i = 0;
        while (i < last - 1 && x > xTable[i + 1]) i++;

        double x0 = xTable[i];
        double x1 = xTable[i + 1];
        double y0 = yTable[i];
        double y1 = yTable[i + 1];

        return y0 + (x - x0) * (y1 - y0) / (x1 - x0);
    }

    public void reset() { lastTargetTPS = 0.0; }
    
    public double getPhysicsTPS() { return physicsTPS; }
    public double getTableTPS() { return tableTPS; }
    public double getBaseTPS() { return baseTPS; }
    public double getFinalTPS() { return finalTPS; }
}
