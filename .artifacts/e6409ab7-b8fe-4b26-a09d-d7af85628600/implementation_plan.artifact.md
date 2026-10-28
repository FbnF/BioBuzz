# Implementation Plan - Create 28WaitP2 Autonomous OpMode

Create the new `WaitStart28P2` autonomous OpMode under package `org.firstinspires.ftc.teamcode.BiobuzzAuto` with DS name `"28WaitP2"`. This routine mirrors `28WaitP1` (8-second wait with flywheel pre-spin and blocked `blockServo`, followed by shooting logic), but uses the new line path from start `(8.8966, 77.6421, 0°)` to park at `(12.5687, 123.8756, 0°)`.

## User Review Required

> [!IMPORTANT]
> - **Driver Station Name:** `"28WaitP2"` (Class name: `WaitStart28P2`).
> - **Starting Pose & Wait:** Starts at `(8.8966, 77.6421, 0°)` and waits for 8 seconds (`waitMs(8000)`) while pre-spinning the flywheel.
> - **Shooting Sequence:** Same as `28WaitP1` (`blockServo`, `feed`, `intake`, `sideServo`, `intakeHelper`, `flowerServo`, `frontServo`).
> - **Line Pathing to Park:** Follows a straight line from start to `(12.5687, 123.8756, 0°)`.
> - **Code Style:** Clean Pedro 2 + Ivy implementation with zero comments.

## Proposed Changes

### BiobuzzAuto

#### [NEW] [WaitStart28P2.java](file:///C:/Users/mksla/OneDrive/Documents/Biobuzz/BioBuzz/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/BiobuzzAuto/WaitStart28P2.java)
- Implement `WaitStart28P2` combining the 8-second wait shooting logic with the new start and park line path coordinates.

## Verification Plan

### Automated Tests
- Build project using `:TeamCode:compileDebugJavaWithJavac` and `assembleDebug`.
