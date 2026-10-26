# Implementation Plan - Drive frontServo and intakeHelper Directly from intakeMotor Power in TeleOp

Sync `frontServo` and `intakeHelper` power in TeleOp directly with `robot.intakeMotor.getPower()`, ensuring that whenever the intake motor runs (forward or reverse), both gecko wheels spin automatically at their designated speeds (`FRONT_SERVO_INTAKE_POWER`/`REVERSE_POWER` and `INTAKE_HELPER_INTAKE_POWER`/`REVERSE_POWER`).

## User Review Required

> [!IMPORTANT]
> - **Intake Power Derivation:** `frontServo` and `intakeHelper` powers are now driven directly from `robot.intakeMotor.getPower()`.
> - **Forward Intake (`intakePower > 0`):** `frontServo` = `1.0`, `intakeHelper` = `-1.0`.
> - **Reverse Intake / Eject (`intakePower < 0`):** `frontServo` = `-1.0`, `intakeHelper` = `1.0`.
> - **Idle (`intakePower == 0`):** `0.0`.

## Proposed Changes

### NewTeleOp

#### [MODIFY] [TeleOpMainBlue.java](file:///C:/Users/mksla/OneDrive/Documents/Biobuzz/BioBuzz/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/NewTeleOp/TeleOpMainBlue.java)
- Derive `frontServo` and `intakeHelper` powers from `robot.intakeMotor.getPower()`.

#### [MODIFY] [TeleOpMainRed.java](file:///C:/Users/mksla/OneDrive/Documents/Biobuzz/BioBuzz/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/NewTeleOp/TeleOpMainRed.java)
- Derive `frontServo` and `intakeHelper` powers from `robot.intakeMotor.getPower()`.

## Verification Plan

### Automated Tests
- Build project using `:TeamCode:compileDebugJavaWithJavac` and `assembleDebug`.
