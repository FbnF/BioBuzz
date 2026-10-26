# Walkthrough - Drive frontServo and intakeHelper Directly from intakeMotor Power

Updated TeleOp modes (`TeleOpMainBlue` and `TeleOpMainRed`) so `frontServo` and `intakeHelper` derive their continuous power directly from `robot.intakeMotor.getPower()`.

## Changes

### TeleOp
- **[TeleOpMainBlue.java](file:///C:/Users/mksla/OneDrive/Documents/Biobuzz/BioBuzz/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/NewTeleOp/TeleOpMainBlue.java) / [TeleOpMainRed.java](file:///C:/Users/mksla/OneDrive/Documents/Biobuzz/BioBuzz/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/NewTeleOp/TeleOpMainRed.java):**
  - Reads `double intakePower = robot.intakeMotor.getPower();` each loop cycle.
  - When `intakePower > 0.0`: drives `frontServo` at `FRONT_SERVO_INTAKE_POWER` (`1.0`) and `intakeHelper` at `INTAKE_HELPER_INTAKE_POWER` (`-1.0`).
  - When `intakePower < 0.0`: drives `frontServo` at `FRONT_SERVO_REVERSE_POWER` (`-1.0`) and `intakeHelper` at `INTAKE_HELPER_REVERSE_POWER` (`1.0`).
  - When idle (`0.0`): both are set to `0.0`.

## Verification Results

### Automated Builds
- Executed `assembleDebug` successfully (`BUILD SUCCESSFUL`). Compiles cleanly without errors.
