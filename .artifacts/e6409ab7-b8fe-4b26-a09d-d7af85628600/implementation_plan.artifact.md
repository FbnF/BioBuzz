# Implementation Plan - Add Front Servo Synced with Intake and Transfer

Add a new continuous rotation servo (`frontServo`) mapped in hardware and fully synced with intaking, outtaking/ejecting, and transfer operations across TeleOp modes (`TeleOpMainBlue`, `TeleOpMainRed`, `SystemsTest`) and autonomous routines (`REDBLUE28LeavePark`, `REDBLUE28Garden`, `REDBLUE28ShootStart`).

## User Review Required

> [!IMPORTANT]
> - **Hardware Configuration:** Added `FRONT_SERVO = "frontServo"` to `HardwareConfig`.
> - **RobotHardware Service:** Mapped `CRServo frontServo`, defaulting to power `0.0` on init.
> - **TeleOp & Autonomous Logic:** Synced `frontServo` power with `sideServo` / intake power (runs forward at `1.0` when intaking, reverse at `-1.0` when outtaking/ejecting, and `0.0` when stopped).

## Proposed Changes

### configs

#### [HardwareConfig.java](file:///C:/Users/mksla/OneDrive/Documents/Biobuzz/BioBuzz/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/configs/HardwareConfig.java)
- Add `public static final String FRONT_SERVO = "frontServo";`

### NewTeleOp/Services

#### [RobotHardware.java](file:///C:/Users/mksla/OneDrive/Documents/Biobuzz/BioBuzz/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/NewTeleOp/Services/RobotHardware.java)
- Add `public CRServo frontServo;`
- Map `frontServo` in `init()` and set power to `0.0`.

### NewTeleOp

#### [TeleOpMainBlue.java](file:///C:/Users/mksla/OneDrive/Documents/Biobuzz/BioBuzz/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/NewTeleOp/TeleOpMainBlue.java)
- Sync `frontServo.setPower(...)` with `sideServo` / intake power during intake, ejecting, and stop states.

#### [TeleOpMainRed.java](file:///C:/Users/mksla/OneDrive/Documents/Biobuzz/BioBuzz/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/NewTeleOp/TeleOpMainRed.java)
- Sync `frontServo.setPower(...)` with `sideServo` / intake power during intake, ejecting, and stop states.

#### [SystemsTest.java](file:///C:/Users/mksla/OneDrive/Documents/Biobuzz/BioBuzz/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/NewTeleOp/SystemsTest.java)
- Sync `frontServo.setPower(...)` with `sideOn` state.

### BiobuzzAuto

#### [REDBLUE28LeavePark.java](file:///C:/Users/mksla/OneDrive/Documents/Biobuzz/BioBuzz/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/BiobuzzAuto/REDBLUE28LeavePark.java)
- Map and sync `frontServo` alongside `sideServo` during shooting/reloading and shutdown.

#### [REDBLUE28Garden.java](file:///C:/Users/mksla/OneDrive/Documents/Biobuzz/BioBuzz/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/BiobuzzAuto/REDBLUE28Garden.java)
- Map and sync `frontServo` alongside `sideServo` during shooting/reloading and shutdown.

#### [REDBLUE28ShootStart.java](file:///C:/Users/mksla/OneDrive/Documents/Biobuzz/BioBuzz/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/BiobuzzAuto/REDBLUE28ShootStart.java)
- Map and sync `frontServo` alongside `sideServo` during shooting/reloading and shutdown.

## Verification Plan

### Automated Tests
- Build project using `:TeamCode:compileDebugJavaWithJavac` and `assembleDebug`.
