package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.MecanumDrive;

@Config
@TeleOp(name = "AutoTurretAim_Debug")
public class AutoTurretAim_Debug extends LinearOpMode {

    // ---------------- HARDWARE ----------------
    DcMotorEx turret;
    MecanumDrive drive;

    // ---------------- PID ----------------
    public static double kP = 0.025;
    public static double kI = 0.0;
    public static double kD = 0.0005;

    double integral = 0;
    double lastError = 0;
    long lastTime;

    // ---------------- TURRET CONSTANTS ----------------
    static final double MOTOR_TICKS_PER_REV = 537.6;
    static final double PULLEY_REDUCTION = 108.0 / 16.0;
    static final double TICKS_PER_TURRET_REV = MOTOR_TICKS_PER_REV * PULLEY_REDUCTION;

    public static double TURRET_MIN_DEG = -135.0;
    public static double TURRET_MAX_DEG = 135.0;

    // ---------------- TARGET ----------------
    public static double TARGET_X = -72.0;
    public static double TARGET_Y = 72.0;

    // ---------------- DEBUG/OVERRIDE ----------------
    public static boolean MANUAL_OVERRIDE = false;  // Toggle in FTC Dashboard
    public static boolean ENABLE_AUTO_AIM = true;   // Toggle auto-aim on/off

    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        turret = hardwareMap.get(DcMotorEx.class, "turret");
        Pose2d startPose = new Pose2d(0, 0, 0);
        drive = new MecanumDrive(hardwareMap, startPose);

        turret.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        turret.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        // Try REVERSE first, if turret moves wrong way, change to FORWARD
        turret.setDirection(DcMotorEx.Direction.REVERSE);

        lastTime = System.nanoTime();

        telemetry.addLine("===========================");
        telemetry.addLine("Position turret FORWARD (0°)");
        telemetry.addLine("Press START when ready");
        telemetry.addLine("===========================");
        telemetry.addLine();
        telemetry.addLine("Controls:");
        telemetry.addLine("  DPAD Up/Down: Manual turret");
        telemetry.addLine("  A: Toggle auto-aim");
        telemetry.addData("Current Encoder", turret.getCurrentPosition());
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            // Toggle auto-aim with A button
            if (gamepad1.a) {
                ENABLE_AUTO_AIM = !ENABLE_AUTO_AIM;
                sleep(200); // Debounce
            }

            // Manual turret override with DPAD
            if (gamepad1.dpad_up || gamepad1.dpad_down) {
                MANUAL_OVERRIDE = true;
                double manualPower = gamepad1.dpad_up ? 0.3 : -0.3;
                turret.setPower(manualPower);
            } else {
                MANUAL_OVERRIDE = false;
            }

            // Update RR localization
            drive.updatePoseEstimate();
            drive.setDrivePowers(new PoseVelocity2d(
                    new Vector2d(
                            -gamepad1.left_stick_y,
                            -gamepad1.left_stick_x
                    ),
                    -gamepad1.right_stick_x
            ));

            Pose2d pose = drive.localizer.getPose();
            double robotX = pose.position.x;
            double robotY = pose.position.y;
            double robotHeadingDeg = Math.toDegrees(pose.heading.toDouble());

            // Calculate field angle to target
            double dx = TARGET_X - robotX;
            double dy = TARGET_Y - robotY;
            double distanceToTarget = Math.sqrt(dx * dx + dy * dy);
            double angleToTargetDeg = Math.toDegrees(Math.atan2(dy, dx));

            // Turret target relative to robot
            double turretTargetDeg = wrapAngle(angleToTargetDeg - robotHeadingDeg);
            double turretAngleDeg = getTurretAngleDeg();
            double error = wrapAngle(turretTargetDeg - turretAngleDeg);

            // Auto-aim PID control (if enabled and not overridden)
            if (ENABLE_AUTO_AIM && !MANUAL_OVERRIDE) {
                // Check limits
                double directTarget = turretAngleDeg + error;

                if (directTarget < TURRET_MIN_DEG || directTarget > TURRET_MAX_DEG) {
                    double altError = error > 0 ? error - 360 : error + 360;
                    double altTarget = turretAngleDeg + altError;

                    if (altTarget >= TURRET_MIN_DEG && altTarget <= TURRET_MAX_DEG) {
                        error = altError;
                    } else {
                        if (turretAngleDeg < TURRET_MIN_DEG) {
                            error = TURRET_MIN_DEG - turretAngleDeg;
                        } else if (turretAngleDeg > TURRET_MAX_DEG) {
                            error = TURRET_MAX_DEG - turretAngleDeg;
                        } else {
                            if ((turretAngleDeg <= TURRET_MIN_DEG + 1 && error < 0) ||
                                    (turretAngleDeg >= TURRET_MAX_DEG - 1 && error > 0)) {
                                error = 0;
                            }
                        }
                    }
                }

                // PID
                long now = System.nanoTime();
                double dt = (now - lastTime) / 1e9;
                lastTime = now;

                if (Math.abs(error) < 10) {
                    integral += error * dt;
                } else {
                    integral = 0;
                }
                integral = Math.max(-50, Math.min(50, integral));

                double derivative = (dt > 0) ? (error - lastError) / dt : 0;
                lastError = error;

                double power = kP * error + kI * integral + kD * derivative;
                power = Math.max(-0.8, Math.min(0.8, power));

                if ((turretAngleDeg <= TURRET_MIN_DEG && power < 0) ||
                        (turretAngleDeg >= TURRET_MAX_DEG && power > 0)) {
                    power = 0;
                }

                turret.setPower(power);
            } else if (!MANUAL_OVERRIDE) {
                turret.setPower(0);
            }

            // ---------------- ENHANCED TELEMETRY ----------------
            telemetry.addLine("=== AUTO-AIM STATUS ===");
            telemetry.addData("Auto-Aim", ENABLE_AUTO_AIM ? "ENABLED" : "DISABLED");
            telemetry.addData("Manual Override", MANUAL_OVERRIDE ? "ACTIVE" : "---");

            telemetry.addLine();
            telemetry.addLine("=== LOCALIZATION ===");
            telemetry.addData("Robot X", "%.1f in", robotX);
            telemetry.addData("Robot Y", "%.1f in", robotY);
            telemetry.addData("Robot Heading", "%.1f°", robotHeadingDeg);
            telemetry.addData("Distance to Target", "%.1f in", distanceToTarget);

            telemetry.addLine();
            telemetry.addLine("=== TURRET ===");
            telemetry.addData("Current Angle", "%.1f°", turretAngleDeg);
            telemetry.addData("Target Angle", "%.1f°", turretTargetDeg);
            telemetry.addData("Error", "%.1f°", error);
            telemetry.addData("Field Angle to Target", "%.1f°", angleToTargetDeg);
            telemetry.addData("Encoder Ticks", turret.getCurrentPosition());

            telemetry.addLine();
            telemetry.addLine("=== DIAGNOSTICS ===");
            telemetry.addData("Within Soft Limits",
                    (turretAngleDeg > TURRET_MIN_DEG && turretAngleDeg < TURRET_MAX_DEG));
            telemetry.addData("Target X, Y", "%.0f, %.0f", TARGET_X, TARGET_Y);

            // Warning if localization seems wrong
            if (Math.abs(robotX) > 200 || Math.abs(robotY) > 200) {
                telemetry.addLine();
                telemetry.addLine("⚠️ WARNING: Position values seem unrealistic!");
                telemetry.addLine("Check your Road Runner tuning");
            }

            telemetry.update();
        }
    }

    double getTurretAngleDeg() {
        return (turret.getCurrentPosition() / TICKS_PER_TURRET_REV) * 360.0;
    }

    double wrapAngle(double angle) {
        angle %= 360.0;
        if (angle > 180) angle -= 360;
        if (angle < -180) angle += 360;
        return angle;
    }
}