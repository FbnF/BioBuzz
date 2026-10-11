# Implementation Plan - Uncomment Sample OpModes & Retain Old Version for Reference

Uncomment the FTC SDK sample OpMode files in `FtcRobotController` so they are fully active classes. In each file, keep the old version of AprilTag detection processing (using direct `detection.metadata`, `detection.id`, `detection.center`) commented out for reference, while running the new `AprilTagSingleDetection` implementation.

## User Review Required

> [!IMPORTANT]
> This plan uncomments the sample files (`ConceptAprilTagEasy.java`, `ConceptAprilTagLocalization.java`, `ConceptAprilTagSwitchableCameras.java`, `RobotAutoDriveToAprilTagOmni.java`, `RobotAutoDriveToAprilTagTank.java`) and ensures both the commented-out old implementation (for reference) and active `AprilTagSingleDetection` implementation are present in each file.

## Proposed Changes

### FtcRobotController Sample OpModes

#### [MODIFY] [ConceptAprilTagEasy.java](file:///C:/Users/mksla/OneDrive/Documents/Biobuzz/BioBuzz/FtcRobotController/src/main/java/org/firstinspires/ftc/robotcontroller/external/samples/ConceptAprilTagEasy.java)
- Uncomment the entire class.
- Retain the commented-out old `telemetryAprilTag()` method for reference.
- Provide the active `AprilTagSingleDetection` version of `telemetryAprilTag()`.

#### [MODIFY] [ConceptAprilTagLocalization.java](file:///C:/Users/mksla/OneDrive/Documents/Biobuzz/BioBuzz/FtcRobotController/src/main/java/org/firstinspires/ftc/robotcontroller/external/samples/ConceptAprilTagLocalization.java)
- Uncomment the entire class.
- Retain the commented-out old `telemetryAprilTag()` method for reference.
- Provide the active `AprilTagSingleDetection` version of `telemetryAprilTag()`.

#### [MODIFY] [ConceptAprilTagSwitchableCameras.java](file:///C:/Users/mksla/OneDrive/Documents/Biobuzz/BioBuzz/FtcRobotController/src/main/java/org/firstinspires/ftc/robotcontroller/external/samples/ConceptAprilTagSwitchableCameras.java)
- Uncomment the entire class.
- Retain the commented-out old `telemetryAprilTag()` method for reference.
- Provide the active `AprilTagSingleDetection` version of `telemetryAprilTag()`.

#### [MODIFY] [RobotAutoDriveToAprilTagOmni.java](file:///C:/Users/mksla/OneDrive/Documents/Biobuzz/BioBuzz/FtcRobotController/src/main/java/org/firstinspires/ftc/robotcontroller/external/samples/RobotAutoDriveToAprilTagOmni.java)
- Uncomment the entire class.
- Retain the commented-out old tag-matching loop and `desiredTag` declaration for reference.
- Provide the active `AprilTagSingleDetection` version.

#### [MODIFY] [RobotAutoDriveToAprilTagTank.java](file:///C:/Users/mksla/OneDrive/Documents/Biobuzz/BioBuzz/FtcRobotController/src/main/java/org/firstinspires/ftc/robotcontroller/external/samples/RobotAutoDriveToAprilTagTank.java)
- Uncomment the entire class.
- Retain the commented-out old tag-matching loop and `desiredTag` declaration for reference.
- Provide the active `AprilTagSingleDetection` version.

## Verification Plan

### Automated Tests
- Run `:FtcRobotController:compileDebugJavaWithJavac` and `assembleDebug` to verify successful compilation.
