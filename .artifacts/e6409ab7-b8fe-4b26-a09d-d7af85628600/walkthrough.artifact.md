# Walkthrough - Create LimelightWaitStartRedP2 & LimelightWaitStartBlueP2 Autonomous OpModes

Created **`LimelightWaitStartRedP2`** and **`LimelightWaitStartBlueP2`** under package `org.firstinspires.ftc.teamcode.BiobuzzAuto`.

## Changes

### BiobuzzAuto

#### [LimelightWaitStartRedP2.java](file:///C:/Users/mksla/OneDrive/Documents/Biobuzz/BioBuzz/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/BiobuzzAuto/LimelightWaitStartRedP2.java) & [LimelightWaitStartBlueP2.java](file:///C:/Users/mksla/OneDrive/Documents/Biobuzz/BioBuzz/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/BiobuzzAuto/LimelightWaitStartBlueP2.java)
- Travel-first pathing based on `WaitStart28P2` (`28WaitP2`): follows `path1Chain` to the shooting position while pre-spinning the flywheel and holding `blockServo` at `0.26` (blocked).
- Integrates `VisionService`:
  - **Red (`LimelightWaitStartRedP2`):** Configured for AprilTag IDs `30, 31, 32, 33`.
  - **Blue (`LimelightWaitStartBlueP2`):** Configured for AprilTag IDs `38, 39, 40, 41`.
- Waits until Limelight sees the target (`waitUntil(() -> vision.isCurrentlySeeing())`), then waits 3 seconds (`waitMs(3000)`) for settling.
- Executes `combinedShootLogic()` and follows `path2Chain` to park.
- Clean Pedro 2 + Ivy implementation with zero comments.

## Verification Results

### Automated Builds
- Executed `assembleDebug` successfully (`BUILD SUCCESSFUL`). Compiles cleanly without errors.
