# Walkthrough - Add Front Servo Synced with Intake and Transfer

Added `frontServo` (`CRServo`) mapped in hardware and fully synced with intaking, outtaking/ejecting, and transfer operations across TeleOp modes, SystemsTest, and autonomous routines.

## Changes

### configs & Services
- **[HardwareConfig.java](file:///C:/Users/mksla/OneDrive/Documents/Biobuzz/BioBuzz/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/configs/HardwareConfig.java):** Added `FRONT_SERVO = "frontServo"`.
- **[RobotHardware.java](file:///C:/Users/mksla/OneDrive/Documents/Biobuzz/BioBuzz/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/NewTeleOp/Services/RobotHardware.java):** Mapped `frontServo` (`CRServo`) and initialized its power to `0.0`.

### TeleOp & Autonomous
- **TeleOp (`TeleOpMainBlue`, `TeleOpMainRed`, `SystemsTest`):** Synced `frontServo.setPower(...)` with intake/sideServo power state (running forward at `1.0` when intaking, reverse at `-1.0` when outtaking/ejecting, and `0.0` when stopped).
- **Autonomous (`REDBLUE28Garden`, `REDBLUE28ShootStart`):** Synced `frontServo` alongside `sideServo` and `intake` during shooting/reloading and shutdown.

## Verification Results

### Automated Builds
- Executed `assembleDebug` successfully (`BUILD SUCCESSFUL`). Compiles cleanly without errors.
