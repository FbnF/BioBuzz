# Implementation Plan - Create LimelightWaitStartRedP2 & LimelightWaitStartBlueP2 Autonomous OpModes

Create two new autonomous OpModes under package `org.firstinspires.ftc.teamcode.BiobuzzAuto`:
1. **`LimelightWaitStartRedP2`** (DS Name: `"LimelightWaitStartRedP2"`, tags `30, 31, 32, 33`)
2. **`LimelightWaitStartBlueP2`** (DS Name: `"LimelightWaitStartBlueP2"`, tags `38, 39, 40, 41`)

These routines are based on `WaitStart28P2`: they travel to the shooting position first (`path1Chain`), pre-spin the flywheel while traveling with `blockServo` blocked (`0.26`), wait until Limelight sees the alliance AprilTag, wait 3 seconds for settling, execute `combinedShootLogic()`, and then follow `path2Chain` to park.

## User Review Required

> [!IMPORTANT]
> - **Driver Station Names:** `"LimelightWaitStartRedP2"` and `"LimelightWaitStartBlueP2"`.
> - **Travel-First Vision Logic:** Travels to `shootPose` first (`path1Chain`), pre-spinning flywheel and holding `blockServo` at `0.26`, then waits until `vision.isCurrentlySeeing()` (with alliance tags configured).
> - **Settling Delay:** Waits 3 seconds (`waitMs(3000)`) after tag detection before shooting.
> - **Code Style:** Clean Pedro 2 + Ivy implementation with zero comments.

## Proposed Changes

### BiobuzzAuto

#### [NEW] [LimelightWaitStartRedP2.java](file:///C:/Users/mksla/OneDrive/Documents/Biobuzz/BioBuzz/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/BiobuzzAuto/LimelightWaitStartRedP2.java)
- Implement `LimelightWaitStartRedP2` with travel-first pathing, pre-spin, Limelight tag detection (`30..33`), 3-second settle, shooting, and park path.

#### [NEW] [LimelightWaitStartBlueP2.java](file:///C:/Users/mksla/OneDrive/Documents/Biobuzz/BioBuzz/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/BiobuzzAuto/LimelightWaitStartBlueP2.java)
- Implement `LimelightWaitStartBlueP2` with travel-first pathing, pre-spin, Limelight tag detection (`38..41`), 3-second settle, shooting, and park path.

## Verification Plan

### Automated Tests
- Build project using `:TeamCode:compileDebugJavaWithJavac` and `assembleDebug`.
