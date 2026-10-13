# Walkthrough - Create RED28LeavePark & Disable Previous Autonomous

Disabled all 16 previous season autonomous OpModes in `NewAutonomous` using `@Disabled` (preserving them as reference). Created the new `RED28LeavePark.java` under package `BiobuzzAuto` using Pedro 2 and Ivy commands.

## Changes

### NewAutonomous (Disabled OpModes)
- Added `@com.qualcomm.robotcore.eventloop.opmode.Disabled` to all 16 OpModes under `NewAutonomous/BigTriBLUE`, `NewAutonomous/BigTriRED`, and `NewAutonomous/SmallTriangle`.

### BiobuzzAuto

#### [RED28LeavePark.java](file:///C:/Users/mksla/OneDrive/Documents/Biobuzz/BioBuzz/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/BiobuzzAuto/RED28LeavePark.java)
- Converted Pedro 3 visualizer path to Pedro 2 `BezierLine` and `BezierCurve`.
- Pre-spins flywheel to `1165.0 TPS` in parallel with `driveToShoot`.
- Executes `combinedShootLogic()` at `shootPose` upon arrival.
- Drives along curved path (`driveToPark`) and shuts down motors and servos at the end.

## Verification Results

### Automated Builds
- Executed `assembleDebug` successfully (`BUILD SUCCESSFUL`). Compiles cleanly without errors.
