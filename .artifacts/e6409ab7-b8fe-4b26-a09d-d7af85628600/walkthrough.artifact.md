# Walkthrough - Add intakeHelper CRServo (Port 5)

Added `intakeHelper` (`CRServo`, port 5) mapped in hardware and fully synced with intake and transfer operations across TeleOp modes, SystemsTest, and Autonomous routines.

## Changes

### configs & Services
- **[HardwareConfig.java](file:///C:/Users/mksla/OneDrive/Documents/Biobuzz/BioBuzz/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/configs/HardwareConfig.java):** Added `INTAKE_HELPER = "intakeHelper"`.
- **[RobotHardware.java](file:///C:/Users/mksla/OneDrive/Documents/Biobuzz/BioBuzz/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/NewTeleOp/Services/RobotHardware.java):** Mapped `intakeHelper` (`CRServo`) and initialized its power to `0.0`.

### TeleOp & Autonomous
- **TeleOp (`TeleOpMainBlue`, `TeleOpMainRed`, `SystemsTest`):** Synced `intakeHelper.setPower(...)` with intake state (runs at `-1.0` when intaking, `1.0` when reverse intaking/outtaking, and `0.0` when idle).
- **Autonomous (`REDBLUE28Garden`, `REDBLUE28ShootStart`, `REDBLUE28WaitStart`):** Synced `intakeHelper.setPower(-1.0)` alongside `sideServo` and `intake` during intaking/shooting and reset to `0.0` on shutdown.

## Verification Results

### Automated Builds
- Executed `assembleDebug` successfully (`BUILD SUCCESSFUL`). Compiles cleanly without errors.
