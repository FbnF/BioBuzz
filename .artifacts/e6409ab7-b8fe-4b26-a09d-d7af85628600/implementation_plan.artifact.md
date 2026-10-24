# Implementation Plan - Add intakeHelper CRServo (Port 5)

Add a new continuous rotation servo (`intakeHelper`, port 5) mapped in hardware and fully synced with intake and transfer operations across TeleOp modes (`TeleOpMainBlue`, `TeleOpMainRed`), `SystemsTest`, and Autonomous routines (`REDBLUE28Garden`, `REDBLUE28ShootStart`, `REDBLUE28WaitStart`).

## User Review Required

> [!IMPORTANT]
> - **Hardware Configuration:** Added `INTAKE_HELPER = "intakeHelper"` to `HardwareConfig`.
> - **RobotHardware Service:** Mapped `CRServo intakeHelper`, defaulting to power `0.0` on init.
> - **Sync Logic:** Runs at `-1` when intaking, `1` when reverse intaking/outtaking, and `0` when idle.

## Proposed Changes

### configs

#### [HardwareConfig.java](file:///C:/Users/mksla/OneDrive/Documents/Biobuzz/BioBuzz/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/configs/HardwareConfig.java)
- Add `public static final String INTAKE_HELPER = "intakeHelper";`

### NewTeleOp/Services

#### [RobotHardware.java](file:///C:/Users/mksla/OneDrive/Documents/Biobuzz/BioBuzz/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/NewTeleOp/Services/RobotHardware.java)
- Add `public CRServo intakeHelper;`
- Map `intakeHelper` in `init()` and set power to `0.0`.

### NewTeleOp

#### [TeleOpMainBlue.java](file:///C:/Users/mksla/OneDrive/Documents/Biobuzz/BioBuzz/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/NewTeleOp/TeleOpMainBlue.java)
- Sync `intakeHelper.setPower(...)` with intake / eject state (`-1.0` intaking, `1.0` reverse/eject, `0.0` idle).

#### [TeleOpMainRed.java](file:///C:/Users/mksla/OneDrive/Documents/Biobuzz/BioBuzz/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/NewTeleOp/TeleOpMainRed.java)
- Sync `intakeHelper.setPower(...)` with intake / eject state.

#### [SystemsTest.java](file:///C:/Users/mksla/OneDrive/Documents/Biobuzz/BioBuzz/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/NewTeleOp/SystemsTest.java)
- Sync `intakeHelper.setPower(...)` with `sideOn` state.

### BiobuzzAuto

#### [REDBLUE28Garden.java](file:///C:/Users/mksla/OneDrive/Documents/Biobuzz/BioBuzz/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/BiobuzzAuto/REDBLUE28Garden.java)
- Map and sync `intakeHelper` alongside `sideServo` and `intake` during intaking/shooting and shutdown.

#### [REDBLUE28ShootStart.java](file:///C:/Users/mksla/OneDrive/Documents/Biobuzz/BioBuzz/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/BiobuzzAuto/REDBLUE28ShootStart.java)
- Map and sync `intakeHelper` alongside `sideServo` and `intake` during intaking/shooting and shutdown.

#### [REDBLUE28WaitStart.java](file:///C:/Users/mksla/OneDrive/Documents/Biobuzz/BioBuzz/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/BiobuzzAuto/REDBLUE28WaitStart.java)
- Map and sync `intakeHelper` alongside `sideServo` and `intake` during intaking/shooting and shutdown.

## Verification Plan

### Automated Tests
- Build project using `:TeamCode:compileDebugJavaWithJavac` and `assembleDebug`.
