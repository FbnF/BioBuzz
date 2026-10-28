# Walkthrough - Add flowerServo & Convert frontServo back to Positional Torque Servo

Added `flowerServo` (`CRServo`) as the opposite of `intakeHelper`, and converted `frontServo` back to a positional torque servo (`Servo`) controlled via Gamepad 2 A button.

## Changes

### configs & Services
- **[HardwareConfig.java](file:///C:/Users/mksla/OneDrive/Documents/Biobuzz/BioBuzz/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/configs/HardwareConfig.java):** Added `FLOWER_SERVO = "flowerServo"`.
- **[ShooterConfig.java](file:///C:/Users/mksla/OneDrive/Documents/Biobuzz/BioBuzz/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/configs/ShooterConfig.java):** Added `FRONT_SERVO_INIT_POS = 0.06` and `FRONT_SERVO_FLOWER_POS = 0.38`.
- **[RobotHardware.java](file:///C:/Users/mksla/OneDrive/Documents/Biobuzz/BioBuzz/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/NewTeleOp/Services/RobotHardware.java):** Converted `frontServo` to `Servo` (init `0.06`) and mapped `flowerServo` as `CRServo` (init `0.0`).

### TeleOp & SystemsTest
- **TeleOp (`TeleOpMainBlue`, `TeleOpMainRed`, `SystemsTest`):**
  - Gamepad 2 A button (`gamepad2.a`) toggles `frontServo` position between up (`0.06`) and down (`0.38`).
  - `flowerServo` runs as the opposite of `intakeHelper` **only when `frontServo` is down (`0.38`) AND intake is active**. If `frontServo` is up (`0.06`), `flowerServo` stays `0.0`.

## Verification Results

### Automated Builds
- Executed `assembleDebug` successfully (`BUILD SUCCESSFUL`). Compiles cleanly without errors.
