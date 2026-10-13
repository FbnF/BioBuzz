# Implementation Plan - Create REDBLUE28ShootStart Autonomous

Create the new `REDBLUE28ShootStart` autonomous OpMode under package `org.firstinspires.ftc.teamcode.BiobuzzAuto`. This OpMode starts directly at the shooting position (`58.817, 10.3836, 90°`), spins up the flywheel to `1165.0 TPS`, executes the shooting sequence upon reaching target speed, and then drives to the parking position (`13.9617, 97.3239, 290°`).

## User Review Required

> [!IMPORTANT]
> - **Starting Pose:** Robot starts directly at `(58.817, 10.3836, 90°)`.
> - **Shoot From Start:** Flywheel spins up immediately upon start, waits for velocity to reach target speed (`1165.0 TPS`), unblocks `blockServo` (`0.38`), feeds and intakes.
> - **Park Path:** Follows curved trajectory (`driveToPark`) to `(13.9617, 97.3239, 290°)` with control point `(27.6424, 53.3308, 0°)`.
> - **Code Style:** Clean Pedro 2 + Ivy command implementation without comments.

## Proposed Changes

### BiobuzzAuto

#### [NEW] [REDBLUE28ShootStart.java](file:///C:/Users/mksla/OneDrive/Documents/Biobuzz/BioBuzz/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/BiobuzzAuto/REDBLUE28ShootStart.java)
- Implement `REDBLUE28ShootStart` matching `REDBLUE28LeavePark` hardware & Ivy setup.
- Executes shooting sequence directly at start position before driving to park.

## Verification Plan

### Automated Tests
- Build project using `:TeamCode:compileDebugJavaWithJavac` and `assembleDebug`.
