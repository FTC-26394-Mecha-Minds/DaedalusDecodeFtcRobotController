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

@Config
@TeleOp(name = "AutoTurretAim_Final")
public class AutoTurretAim_Final extends LinearOpMode {

    DcMotorEx turret;
    MecanumDrive drive;

    // PID
    public static double kP = 0.02;
    public static double kI = 0.0;
    public static double kD = 0.001;

    double integral = 0;
    double lastError = 0;
    long lastTime;

    // Turret constants
    static final double MOTOR_TICKS_PER_REV = 537.6;
    static final double PULLEY_REDUCTION = 108.0 / 16.0;
    static final double TICKS_PER_TURRET_REV = MOTOR_TICKS_PER_REV * PULLEY_REDUCTION;

    // Turret limits
    public static double TURRET_MIN_DEG = -135.0;
    public static double TURRET_MAX_DEG = 135.0;

    // Target position (field coordinates)
    public static double TARGET_X = -72.0;
    public static double TARGET_Y = 72.0;

    // Motor direction
    public static boolean INVERT_TURRET_DIRECTION = true;

    // Manual override
    boolean manualOverride = false;

    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        turret = hardwareMap.get(DcMotorEx.class, "turret");

        Pose2d startPose = new Pose2d(0, 0, 0);
        drive = new MecanumDrive(hardwareMap, startPose);

        turret.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        // CRITICAL: DO NOT RESET ENCODER
        // This preserves the calibration from TurretCalibration OpMode
        turret.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        turret.setDirection(INVERT_TURRET_DIRECTION ?
                DcMotorEx.Direction.REVERSE : DcMotorEx.Direction.FORWARD);

        lastTime = System.nanoTime();

        double currentAngleDeg = getTurretAngleDeg();

        telemetry.addLine("=================================");
        telemetry.addLine("AUTO-AIM READY");
        telemetry.addLine("=================================");
        telemetry.addLine();
        telemetry.addData("Current Turret Angle", "%.1f°", currentAngleDeg);
        telemetry.addLine();

        if (Math.abs(currentAngleDeg) > 180) {
            telemetry.addLine("⚠️ WARNING:");
            telemetry.addLine("Turret angle seems wrong!");
            telemetry.addLine("Run TurretCalibration first.");
        } else {
            telemetry.addLine("✓ Calibration appears valid");
        }

        telemetry.addLine();
        telemetry.addLine("Controls:");
        telemetry.addLine("  DPAD Up/Down: Manual override");
        telemetry.addLine("  (Release for auto-aim)");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            // Manual override
            if (gamepad1.dpad_up || gamepad1.dpad_down) {
                manualOverride = true;
                turret.setPower(gamepad1.dpad_up ? 0.3 : -0.3);
            } else {
                manualOverride = false;
            }

            // Update pose
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
            double robotHeading = pose.heading.toDouble();

            // Calculate target angle
            double dx = TARGET_X - robotX;
            double dy = TARGET_Y - robotY;
            double angleToTargetField = Math.atan2(dy, dx);
            double angleToTargetRobot = angleToTargetField - robotHeading;

            // Normalize
            while (angleToTargetRobot > Math.PI) angleToTargetRobot -= 2 * Math.PI;
            while (angleToTargetRobot < -Math.PI) angleToTargetRobot += 2 * Math.PI;

            double targetAngleDeg = Math.toDegrees(angleToTargetRobot);
            currentAngleDeg = getTurretAngleDeg();

            // Calculate error
            double error = targetAngleDeg - currentAngleDeg;
            while (error > 180) error -= 360;
            while (error < -180) error += 360;

            // Check limits and wrap-around
            double projectedAngle = currentAngleDeg + error;

            if (projectedAngle < TURRET_MIN_DEG || projectedAngle > TURRET_MAX_DEG) {
                double altError = error > 0 ? error - 360 : error + 360;
                double altProjected = currentAngleDeg + altError;

                if (altProjected >= TURRET_MIN_DEG && altProjected <= TURRET_MAX_DEG) {
                    error = altError;
                } else {
                    if (currentAngleDeg < TURRET_MIN_DEG) {
                        error = TURRET_MIN_DEG - currentAngleDeg;
                    } else if (currentAngleDeg > TURRET_MAX_DEG) {
                        error = TURRET_MAX_DEG - currentAngleDeg;
                    } else {
                        if ((currentAngleDeg <= TURRET_MIN_DEG + 1 && error < 0) ||
                                (currentAngleDeg >= TURRET_MAX_DEG - 1 && error > 0)) {
                            error = 0;
                        }
                    }
                }
            }

            // Auto-aim PID (if not overridden)
            if (!manualOverride) {
                long now = System.nanoTime();
                double dt = (now - lastTime) / 1e9;
                lastTime = now;

                if (Math.abs(error) < 10) {
                    integral += error * dt;
                    integral = Math.max(-50, Math.min(50, integral));
                } else {
                    integral = 0;
                }

                double derivative = (dt > 0) ? (error - lastError) / dt : 0;
                lastError = error;

                double power = kP * error + kI * integral + kD * derivative;
                power = Math.max(-1.0, Math.min(1.0, power));

                if ((currentAngleDeg <= TURRET_MIN_DEG && power < 0) ||
                        (currentAngleDeg >= TURRET_MAX_DEG && power > 0)) {
                    power = 0;
                }

                turret.setPower(power);

                // Telemetry
                telemetry.addLine("=== AUTO-AIM ACTIVE ===");
                telemetry.addLine();
                telemetry.addLine("=== POSITION ===");
                telemetry.addData("Robot", "(%.1f, %.1f) @ %.1f°",
                        robotX, robotY, Math.toDegrees(robotHeading));
                telemetry.addData("Target", "(%.0f, %.0f)", TARGET_X, TARGET_Y);
                telemetry.addData("Distance", "%.1f in", Math.sqrt(dx*dx + dy*dy));

                telemetry.addLine();
                telemetry.addLine("=== TURRET ===");
                telemetry.addData("Current", "%.1f°", currentAngleDeg);
                telemetry.addData("Target", "%.1f°", targetAngleDeg);
                telemetry.addData("Error", "%.1f°", error);
                telemetry.addData("Power", "%.3f", power);
                telemetry.addData("Encoder", turret.getCurrentPosition());

                if (Math.abs(error) < 3) {
                    telemetry.addLine();
                    telemetry.addLine("✓✓✓ LOCKED ON TARGET ✓✓✓");
                }
            } else {
                telemetry.addLine("=== MANUAL OVERRIDE ===");
                telemetry.addData("Current Angle", "%.1f°", currentAngleDeg);
            }

            telemetry.update();
        }
    }

    double getTurretAngleDeg() {
        return (turret.getCurrentPosition() / TICKS_PER_TURRET_REV) * 360.0;
    }
}