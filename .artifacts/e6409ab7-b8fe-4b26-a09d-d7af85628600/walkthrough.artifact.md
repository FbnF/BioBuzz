# Walkthrough - Create REDBLUE28WaitStart Autonomous

Created the new `REDBLUE28WaitStart` autonomous OpMode under package `org.firstinspires.ftc.teamcode.BiobuzzAuto`.

## Changes

### BiobuzzAuto

#### [REDBLUE28WaitStart.java](file:///C:/Users/mksla/OneDrive/Documents/Biobuzz/BioBuzz/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/BiobuzzAuto/REDBLUE28WaitStart.java)
- Configured start pose `(62.5008, 133.2481, 90°)` and curve park path from `path1Start` (`270°`) through `path1Control1` to `parkPose` (`13.9617, 115.0926, 290°`).
- Implements an 8-second wait (`waitMs(8000)`) at start while pre-spinning the flywheel (`1165.0 TPS`) in parallel, keeping `blockServo` at `0.15` (blocked) to save time without firing early.
- Fires preloads using `combinedShootLogic()` (with `frontServo` torque servo at `0.3`, `blockServo` at `0.38`, `feed`, `intake`, and `sideServo`) after the 8-second delay.
- Follows curve path to park.
- Written in clean Pedro 2 + Ivy structure with zero comments.

## Verification Results

### Automated Builds
- Executed `assembleDebug` successfully (`BUILD SUCCESSFUL`). Compiles cleanly without errors.
