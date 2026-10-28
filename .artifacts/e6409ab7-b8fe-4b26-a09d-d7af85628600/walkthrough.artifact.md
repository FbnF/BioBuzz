# Walkthrough - Rename Autonomous Opmode Files and Update 28P2

Successfully renamed all autonomous OpModes and updated their Driver Station display names and class files, while cleanly disabling the old files.

## Changes

### BiobuzzAuto

#### New / Updated OpModes
1. **`ZeroLeaveParkP1.java`** (`@Autonomous(name = "0LeaveParkP1")`): Non-shooting leave/park routine.
2. **`LeavePark28P1.java`** (`@Autonomous(name = "28P1")`): Shooting + leave/park routine.
3. **`LPGardenP1.java`** (`@Autonomous(name = "28LPGardenP1")`): Garden path routine.
4. **`ShootStart28P2.java`** (`@Autonomous(name = "28P2")`): Updated with line path followed by curve path (`path1_2`) as requested, without 8-second delay.
5. **`WaitStart28P1.java`** (`@Autonomous(name = "28WaitP1")`): 8-second wait with pre-spinning and shooting.

## Verification Results

### Automated Builds
- Executed `assembleDebug` successfully (`BUILD SUCCESSFUL`). Compiles cleanly without errors.
