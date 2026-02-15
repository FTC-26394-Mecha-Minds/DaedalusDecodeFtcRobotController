package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;

@Config
@TeleOp(name = "TurretCalibration")
public class TurretCalibration extends LinearOpMode {

    DcMotorEx turret;
    DigitalChannel limitSwitch;  // Optional: connect limit switch at 0° position

    static final double MOTOR_TICKS_PER_REV = 537.6;
    static final double PULLEY_REDUCTION = 108.0 / 16.0;
    static final double TICKS_PER_TURRET_REV = MOTOR_TICKS_PER_REV * PULLEY_REDUCTION;

    // Set to true if you have a limit switch installed
    public static boolean USE_LIMIT_SWITCH = false;
    public static double HOMING_POWER = 0.2;  // Power for auto-homing

    @Override
    public void runOpMode() {

        turret = hardwareMap.get(DcMotorEx.class, "turret");

        if (USE_LIMIT_SWITCH) {
            try {
                limitSwitch = hardwareMap.get(DigitalChannel.class, "turret_limit");
                limitSwitch.setMode(DigitalChannel.Mode.INPUT);
            } catch (Exception e) {
                telemetry.addLine("⚠️ Limit switch not found!");
                telemetry.addLine("Set USE_LIMIT_SWITCH = false");
                telemetry.update();
                sleep(3000);
                return;
            }
        }

        turret.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        turret.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        telemetry.addLine("=================================");
        telemetry.addLine("TURRET CALIBRATION");
        telemetry.addLine("=================================");
        telemetry.addLine();

        if (USE_LIMIT_SWITCH) {
            telemetry.addLine("MODE: Auto-homing with limit switch");
            telemetry.addLine("Press START to begin homing");
        } else {
            telemetry.addLine("MODE: Manual positioning");
            telemetry.addLine("1. Use DPAD to position turret");
            telemetry.addLine("2. Align shooter to FORWARD (0°)");
            telemetry.addLine("3. Press A to set as zero");
        }
        telemetry.update();

        waitForStart();

        if (USE_LIMIT_SWITCH) {
            // Auto-homing routine
            autoHome();
        } else {
            // Manual calibration
            manualCalibration();
        }
    }

    void autoHome() {
        telemetry.addLine("=================================");
        telemetry.addLine("AUTO-HOMING IN PROGRESS");
        telemetry.addLine("=================================");
        telemetry.addLine("Turret will rotate until it hits");
        telemetry.addLine("the limit switch, then set zero.");
        telemetry.update();

        // Rotate toward limit switch
        turret.setPower(-HOMING_POWER);

        while (opModeIsActive() && !limitSwitch.getState()) {
            // Wait for limit switch to be pressed
            telemetry.addData("Limit Switch", limitSwitch.getState() ? "NOT PRESSED" : "PRESSED");
            telemetry.update();
            sleep(10);
        }

        turret.setPower(0);

        // Reset encoder at this position
        turret.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        telemetry.addLine();
        telemetry.addLine("✓ HOMING COMPLETE!");
        telemetry.addLine("Turret zero position set.");
        telemetry.addLine("This position will persist until");
        telemetry.addLine("you power cycle the Control Hub.");
        telemetry.update();
        sleep(3000);
    }

    void manualCalibration() {
        boolean calibrated = false;

        while (opModeIsActive() && !calibrated) {
            // Manual control
            if (gamepad1.dpad_up) {
                turret.setPower(0.3);
            } else if (gamepad1.dpad_down) {
                turret.setPower(-0.3);
            } else if (gamepad1.dpad_left) {
                turret.setPower(0.15);  // Fine control
            } else if (gamepad1.dpad_right) {
                turret.setPower(-0.15);
            } else {
                turret.setPower(0);
            }

            // Set zero position
            if (gamepad1.a) {
                turret.setPower(0);
                turret.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
                turret.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
                calibrated = true;
            }

            telemetry.addLine("=================================");
            telemetry.addLine("MANUAL CALIBRATION");
            telemetry.addLine("=================================");
            telemetry.addLine();
            telemetry.addLine("Controls:");
            telemetry.addLine("  DPAD Up/Down: Move turret");
            telemetry.addLine("  DPAD Left/Right: Fine adjust");
            telemetry.addLine("  A: Set current position as 0°");
            telemetry.addLine();
            telemetry.addData("Current Encoder", turret.getCurrentPosition());
            telemetry.addData("Current Angle", "%.1f°", getTurretAngleDeg());
            telemetry.addLine();
            telemetry.addLine("Position turret so shooter faces");
            telemetry.addLine("FORWARD, then press A");
            telemetry.update();
        }

        if (calibrated) {
            telemetry.addLine();
            telemetry.addLine("✓ CALIBRATION COMPLETE!");
            telemetry.addLine("Zero position set at:");
            telemetry.addData("Encoder", turret.getCurrentPosition());
            telemetry.addLine();
            telemetry.addLine("This will persist until you");
            telemetry.addLine("power cycle the Control Hub.");
            telemetry.addLine();
            telemetry.addLine("You can now run AutoTurretAim");
            telemetry.update();
            sleep(3000);
        }
    }

    double getTurretAngleDeg() {
        return (turret.getCurrentPosition() / TICKS_PER_TURRET_REV) * 360.0;
    }
}