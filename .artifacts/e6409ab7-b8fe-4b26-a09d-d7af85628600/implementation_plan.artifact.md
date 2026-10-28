# Implementation Plan - Add flowerServo & Convert frontServo back to Positional Torque Servo

Add `flowerServo` (`CRServo`) as the opposite of `intakeHelper`, and convert `frontServo` back to a positional torque servo (`Servo`) that controls `flowerServo`'s up/down position (`0.06` init/up, `0.38` flower/down).
- **frontServo:** Positional servo (`Servo`), toggled via Gamepad 2 A button (`gamepad2.a`) between `0.06` (up) and `0.38` (down).
- **flowerServo:** Continuous rotation servo (`CRServo`), runs opposite to `intakeHelper` **only when `frontServo` is down (`0.38`) AND intake is active**. If `frontServo` is up (`0.06`), `flowerServo` stays `0.0`.
- **intakeHelper:** Continuous rotation servo (`CRServo`), runs normally alongside intake (`-1.0` intake, `1.0` reverse).

## User Review Required

> [!IMPORTANT]
> - **Hardware Configuration:** Added `FLOWER_SERVO = "flowerServo"` to `HardwareConfig`.
> - **RobotHardware Service:** Mapped `frontServo` as `Servo` (init `0.06`) and `flowerServo` as `CRServo` (init `0.0`), alongside `intakeHelper` (`CRServo`).
> - **ShooterConfig Constants:**
>   - `FRONT_SERVO_INIT_POS = 0.06`
>   - `FRONT_SERVO_FLOWER_POS = 0.38`
> - **Conditional Sync Logic:** `flowerServo` runs at opposite power of `intakeHelper` only when `frontServo` is down (`0.38`) and intake is active.

## Proposed Changes

### configs

#### [MODIFY] [HardwareConfig.java](file:///C:/Users/mksla/OneDrive/Documents/Biobuzz/BioBuzz/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/configs/HardwareConfig.java)
- Add `public static final String FLOWER_SERVO = "flowerServo";`.

#### [MODIFY] [ShooterConfig.java](file:///C:/Users/mksla/OneDrive/Documents/Biobuzz/BioBuzz/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/configs/ShooterConfig.java)
- Add `FRONT_SERVO_INIT_POS = 0.06` and `FRONT_SERVO_FLOWER_POS = 0.38`.

### NewTeleOp/Services

#### [MODIFY] [RobotHardware.java](file:///C:/Users/mksla/OneDrive/Documents/Biobuzz/BioBuzz/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/NewTeleOp/Services/RobotHardware.java)
- Change `frontServo` to `Servo`, map and initialize to `0.06`.
- Add `flowerServo` as `CRServo`, map and initialize to `0.0`.

### NewTeleOp

#### [MODIFY] [TeleOpMainBlue.java](file:///C:/Users/mksla/OneDrive/Documents/Biobuzz/BioBuzz/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/NewTeleOp/TeleOpMainBlue.java)
- Map toggle for `frontServo` via Gamepad 2 A button (`gamepad2.a`) between `0.06` (up) and `0.38` (down).
- Drive `flowerServo` to `-helperPower` only when `frontServo` position is `0.38` and intake is running, otherwise `0.0`.

#### [MODIFY] [TeleOpMainRed.java](file:///C:/Users/mksla/OneDrive/Documents/Biobuzz/BioBuzz/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/NewTeleOp/TeleOpMainRed.java)
- Same toggle and conditional sync logic for `frontServo` and `flowerServo`.

#### [MODIFY] [SystemsTest.java](file:///C:/Users/mksla/OneDrive/Documents/Biobuzz/BioBuzz/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/NewTeleOp/SystemsTest.java)
- Update test controls for `frontServo` and `flowerServo`.

### BiobuzzAuto

#### [MODIFY] [REDBLUE28Garden.java](file:///C:/Users/mksla/OneDrive/Documents/Biobuzz/BioBuzz/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/BiobuzzAuto/REDBLUE28Garden.java)
- Map `flowerServo`, set `frontServo` to `0.38` (down) during intaking/shooting (so `flowerServo` runs) and `0.06` (up) on init/shutdown.

#### [MODIFY] [REDBLUE28ShootStart.java](file:///C:/Users/mksla/OneDrive/Documents/Biobuzz/BioBuzz/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/BiobuzzAuto/REDBLUE28ShootStart.java)
- Set `frontServo` to `0.38` during shooting and `0.06` on init/shutdown.

#### [MODIFY] [REDBLUE28WaitStart.java](file:///C:/Users/mksla/OneDrive/Documents/Biobuzz/BioBuzz/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/BiobuzzAuto/REDBLUE28WaitStart.java)
- Set `frontServo` to `0.38` during shooting and `0.06` on init/shutdown.

## Verification Plan

### Automated Tests
- Build project using `:TeamCode:compileDebugJavaWithJavac` and `assembleDebug`.
