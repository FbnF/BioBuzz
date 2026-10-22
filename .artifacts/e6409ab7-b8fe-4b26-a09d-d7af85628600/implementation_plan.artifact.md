# Implementation Plan - Create REDBLUE28WaitStart Autonomous

Create the new `REDBLUE28WaitStart` autonomous OpMode under package `org.firstinspires.ftc.teamcode.BiobuzzAuto`. This routine waits for 8 seconds at the starting position (`62.5008, 133.2481, 90°`), pre-spins the flywheel to `1165.0 TPS`, executes the shooting sequence (with `frontServo` torque servo at `0.3`), and then follows a curved path from `path1Start` (`62.5008, 133.2481, 270°`) via `path1Control1` (`59.9119, 115.8859, 0°`) to park at `path1` (`13.9617, 115.0926, 290°`).

## User Review Required

> [!IMPORTANT]
> - **Starting Pose & Wait:** Starts at `(62.5008, 133.2481, 90°)` and waits for 8 seconds (`waitMs(8000)`).
> - **Shooting at Start:** Pre-spins flywheel to `1165.0 TPS` and executes shooting logic (`frontServo` at `0.3`, `blockServo` at `0.38`, `feed`, `intake`, `sideServo`).
> - **Curve Pathing to Park:** Follows curve path from `path1Start` (`270°`) through `path1Control1` to park at `path1` (`290°`).
> - **Code Style:** Clean Pedro 2 + Ivy implementation with no comments and updated `frontServo` torque servo dynamics (`0.2` idle, `0.3` intaking/shooting).

## Proposed Changes

### BiobuzzAuto

#### [NEW] [REDBLUE28WaitStart.java](file:///C:/Users/mksla/OneDrive/Documents/Biobuzz/BioBuzz/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/BiobuzzAuto/REDBLUE28WaitStart.java)
- Implement `REDBLUE28WaitStart` with 8-second delay, start shooting sequence, and curve pathing to park.

## Verification Plan

### Automated Tests
- Build project using `:TeamCode:compileDebugJavaWithJavac` and `assembleDebug`.
