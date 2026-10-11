# Walkthrough - Uncommenting Sample OpModes & Retain Reference Code

Uncommented the standard FTC SDK sample OpMode files while retaining the old detection logic commented out as reference blocks, and keeping the active `AprilTagSingleDetection` implementation running cleanly.

## Changes

### FtcRobotController Sample OpModes

#### [ConceptAprilTagEasy.java](file:///C:/Users/mksla/OneDrive/Documents/Biobuzz/BioBuzz/FtcRobotController/src/main/java/org/firstinspires/ftc/robotcontroller/external/samples/ConceptAprilTagEasy.java)
- Uncommented the entire OpMode class.
- Added active `AprilTagSingleDetection` processing loop.
- Retained the old telemetry logic block as commented reference code (`/* ... */`).

#### [ConceptAprilTagLocalization.java](file:///C:/Users/mksla/OneDrive/Documents/Biobuzz/BioBuzz/FtcRobotController/src/main/java/org/firstinspires/ftc/robotcontroller/external/samples/ConceptAprilTagLocalization.java)
- Uncommented the entire OpMode class.
- Added active `AprilTagSingleDetection` processing loop.
- Retained the old localization telemetry logic block as commented reference code.

#### [ConceptAprilTagSwitchableCameras.java](file:///C:/Users/mksla/OneDrive/Documents/Biobuzz/BioBuzz/FtcRobotController/src/main/java/org/firstinspires/ftc/robotcontroller/external/samples/ConceptAprilTagSwitchableCameras.java)
- Uncommented the entire OpMode class.
- Added active `AprilTagSingleDetection` processing loop.
- Retained the old telemetry logic block as commented reference code.

#### [RobotAutoDriveToAprilTagOmni.java](file:///C:/Users/mksla/OneDrive/Documents/Biobuzz/BioBuzz/FtcRobotController/src/main/java/org/firstinspires/ftc/robotcontroller/external/samples/RobotAutoDriveToAprilTagOmni.java)
- Uncommented the entire OpMode class.
- Added active `AprilTagSingleDetection` tracking loop.
- Retained the old detection snippet as commented reference code.

#### [RobotAutoDriveToAprilTagTank.java](file:///C:/Users/mksla/OneDrive/Documents/Biobuzz/BioBuzz/FtcRobotController/src/main/java/org/firstinspires/ftc/robotcontroller/external/samples/RobotAutoDriveToAprilTagTank.java)
- Uncommented the entire OpMode class.
- Added active `AprilTagSingleDetection` tracking loop.
- Retained the old detection snippet as commented reference code.

## Verification Results

### Automated Builds
- Executed `assembleDebug` successfully (`BUILD SUCCESSFUL`). All updated OpModes compile cleanly without errors.
