# Walkthrough - Tag-Group-Based Shooter Tables & Angle Adjustments

Refactored `ShooterConfig`, `VisionService`, and `ShooterService` to drive shooter TPS and angle-offset lookup tables by Tag Groupings (**Group A** vs **Group B**) rather than alliance color.

## Changes

### configs

#### [ShooterConfig.java](file:///C:/Users/mksla/OneDrive/Documents/Biobuzz/BioBuzz/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/configs/ShooterConfig.java)
- Configured Tag Groups:
  - **Group A:** Blue (`38, 39, 40, 41`) + Red (`30, 31, 32, 33`)
  - **Group B:** Blue (`42, 43, 44, 45`) + Red (`34, 35, 36, 37`)
- Added distance table `[70.0, 75.0, 80.0, 85.0, 90.0, 95.0, 100.0]` and TPS table `[1195, 1165, 1185, 1190, 1195, 1205, 1205]` for both groups.
- Set uniform feed (`-0.9`) and side (`1.0`) powers across all distances.
- Added helper methods `isGroupA(int tagId)` and `isGroupB(int tagId)`.

### NewTeleOp/Services

#### [VisionService.java](file:///C:/Users/mksla/OneDrive/Documents/Biobuzz/BioBuzz/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/NewTeleOp/Services/VisionService.java)
- Tracks `currentTagId`.
- Updates `getTxWindow()` to select Group A vs Group B angle tables dynamically based on `ShooterConfig.isGroupB(currentTagId)`.

#### [ShooterService.java](file:///C:/Users/mksla/OneDrive/Documents/Biobuzz/BioBuzz/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/NewTeleOp/Services/ShooterService.java)
- Updates `calculateVelocity(...)` and feed/side power lookups to pick Group A vs Group B tables dynamically based on `currentTagId`.

## Verification Results

### Automated Builds
- Executed `assembleDebug` successfully (`BUILD SUCCESSFUL`). Compiles cleanly without errors.
