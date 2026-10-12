# Implementation Plan - Refactor Shooter Tables & Auto-Align by Tag Group (Group A vs Group B)

Refactor shooter lookup tables and angle-offset windows in `ShooterConfig` to be driven by Tag Grouping (Group A vs. Group B) rather than alliance color (Red vs. Blue). Update `ShooterService`, `VisionService`, and TeleOp files to select tables dynamically based on the detected AprilTag ID group.

## User Review Required

> [!IMPORTANT]
> - **Group A Tags:** Blue (`38, 39, 40, 41`) + Red (`30, 31, 32, 33`)
> - **Group B Tags:** Blue (`42, 43, 44, 45`) + Red (`34, 35, 36, 37`)
> - **Data Points (Distances 70 to 100 in):** Measured TPS (`1195, 1165, 1185, 1190, 1195, 1205, 1205`) with linear interpolation.
> - **Feed & Side Power:** Replaced distance-varying feed/side powers with uniform constants (`FEED_DEFAULT = -0.9`, `SIDE_DEFAULT = 1.0`).
> - **Angle Windows:** Configured facing adjustments per distance point for Group A and Group B.

## Proposed Changes

### configs

#### [MODIFY] [ShooterConfig.java](file:///C:/Users/mksla/OneDrive/Documents/Biobuzz/BioBuzz/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/configs/ShooterConfig.java)
- Replace `DIST_IN_BLUE` / `DIST_IN_RED` tables with `GROUP_A` (`38..41, 30..33`) and `GROUP_B` (`42..45, 34..37`) distance, TPS, min/max angle, feed, and side power tables.
- Standardize feed power (`-0.9`) and side power (`1.0`) across all distances.
- Add helper methods to check if a tag ID belongs to Group A or Group B.

### NewTeleOp/Services

#### [MODIFY] [VisionService.java](file:///C:/Users/mksla/OneDrive/Documents/Biobuzz/BioBuzz/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/NewTeleOp/Services/VisionService.java)
- Track active matched tag ID.
- Update `getTxWindow()` to select Group A vs. Group B angle tables based on whether the detected tag ID is in Group A or Group B.

#### [MODIFY] [ShooterService.java](file:///C:/Users/mksla/OneDrive/Documents/Biobuzz/BioBuzz/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/NewTeleOp/Services/ShooterService.java)
- Update `calculateVelocity(...)` and feed/side power lookups to pick Group A vs. Group B tables based on the active tag ID group.

### NewTeleOp

#### [MODIFY] [TeleOpMainBlue.java](file:///C:/Users/mksla/OneDrive/Documents/Biobuzz/BioBuzz/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/NewTeleOp/TeleOpMainBlue.java)
- Configure Blue alliance tag IDs (`38, 39, 40, 41, 42, 43, 44, 45`).

#### [MODIFY] [TeleOpMainRed.java](file:///C:/Users/mksla/OneDrive/Documents/Biobuzz/BioBuzz/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/NewTeleOp/TeleOpMainRed.java)
- Configure Red alliance tag IDs (`30, 31, 32, 33, 34, 35, 36, 37`).

## Verification Plan

### Automated Tests
- Build project using `:TeamCode:compileDebugJavaWithJavac` and `assembleDebug`.
