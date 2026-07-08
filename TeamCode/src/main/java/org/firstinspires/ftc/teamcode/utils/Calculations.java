package org.firstinspires.ftc.teamcode.utils;

/**
 * Shooter/ballistics and FTC-friendly unit helpers.
 */
public final class Calculations {

    private Calculations() {} 

    public static double requiredExitVelocity(double g, double x, double launchRad,
                                              double shooterH, double targetH) {
        final double cos = Math.cos(launchRad);
        final double tan = Math.tan(launchRad);
        final double deltaH = targetH - shooterH;
        final double denom = 2.0 * cos * cos * (x * tan - deltaH);

        if (x <= 0 || denom <= 0 || g <= 0) return Double.NaN;

        return Math.sqrt((g * x * x) / denom);
    }

    public static double exitVelocityToWheelRPM(double exitVelMS, double wheelRadiusM, double efficiency) {
        if (exitVelMS <= 0 || wheelRadiusM <= 0 || efficiency <= 0) return Double.NaN;
        final double omega = exitVelMS / (wheelRadiusM * efficiency); // rad/s
        return (omega * 60.0) / (2.0 * Math.PI);
    }

    public static double rpmToTicksPerSecond(double rpm, double ticksPerRev) {
        if (rpm < 0 || ticksPerRev <= 0) return Double.NaN;
        return (rpm * ticksPerRev) / 60.0;
    }

    public static double computeTPSFromRangeInches(double g,
                                                   double horizontalInches,
                                                   double launchDeg,
                                                   double shooterHeightM,
                                                   double targetHeightM,
                                                   double wheelRadiusM,
                                                   double efficiency,
                                                   double ticksPerRev) {
        final double xMeters = horizontalInches * 0.0254;
        final double theta = Math.toRadians(launchDeg);

        double vExit = requiredExitVelocity(g, xMeters, theta, shooterHeightM, targetHeightM);
        if (Double.isNaN(vExit)) return Double.NaN;

        double rpm = exitVelocityToWheelRPM(vExit, wheelRadiusM, efficiency);
        if (Double.isNaN(rpm)) return Double.NaN;

        return rpmToTicksPerSecond(rpm, ticksPerRev);
    }
}
