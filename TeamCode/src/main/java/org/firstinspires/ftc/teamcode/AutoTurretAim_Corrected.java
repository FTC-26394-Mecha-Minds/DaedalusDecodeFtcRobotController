package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.MecanumDrive;

@Config
@TeleOp(name = "AutoTurretAim_Corrected")
public class AutoTurretAim_Corrected extends LinearOpMode {

    DcMotorEx turret;
    MecanumDrive drive;

    // PID - tune these with FTC Dashboard
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

    // Turret limits (adjust to your wire constraints)
    public static double TURRET_MIN_DEG = -135.0;
    public static double TURRET_MAX_DEG = 135.0;

    // Target position in field coordinates (inches)
    // Red classifier at (-72, 72)
    public static double TARGET_X = -72.0;
    public static double TARGET_Y = 72.0;

    // If turret spins the wrong way, toggle this
    public static boolean INVERT_TURRET_DIRECTION = true;

    @Override
    public void runOpMode() {

        turret = hardwareMap.get(DcMotorEx.class, "turret");

        // Starting pose - same as your autonomous
        Pose2d startPose = new Pose2d(0, 0, 0);
        drive = new MecanumDrive(hardwareMap, startPose);

        turret.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        // Reset encoder - CRITICAL: Position turret so shooter faces FORWARD (0°) before init
        turret.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        turret.setDirection(INVERT_TURRET_DIRECTION ?
                DcMotorEx.Direction.REVERSE : DcMotorEx.Direction.FORWARD);

        lastTime = System.nanoTime();

        telemetry.addLine("=================================");
        telemetry.addLine("SETUP:");
        telemetry.addLine("1. Position turret so SHOOTER");
        telemetry.addLine("   faces FORWARD (0°)");
        telemetry.addLine("2. Outtake wheels should be");
        telemetry.addLine("   AWAY from robot center");
        telemetry.addLine("=================================");
        telemetry.addLine();
        telemetry.addLine("At (0,0) heading 0°:");
        telemetry.addLine("Target (-72, 72) is upper-left");
        telemetry.addLine("Turret should turn RIGHT <90°");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            // Update pose using Road Runner
            drive.updatePoseEstimate();
            drive.setDrivePowers(new PoseVelocity2d(
                    new Vector2d(
                            -gamepad1.left_stick_y,
                            -gamepad1.left_stick_x
                    ),
                    -gamepad1.right_stick_x
            ));

            // Get current pose from localizer (this is how RR 1.0 works)
            Pose2d pose = drive.localizer.getPose();

            // Robot position and heading in field frame
            double robotX = pose.position.x;
            double robotY = pose.position.y;
            double robotHeading = pose.heading.toDouble();  // radians, CCW positive

            // Vector from robot to target (in field frame)
            double dx = TARGET_X - robotX;
            double dy = TARGET_Y - robotY;

            // Field angle to target
            // atan2(dy, dx) gives angle from positive X-axis (0° = right, 90° = up)
            double angleToTargetField = Math.atan2(dy, dx);

            // Convert to robot-relative angle
            // In Road Runner: 0° heading = facing positive X (right on field)
            // Turret 0° = shooter facing robot front
            // If robot heading = 0° and target at field angle 135° (upper left):
            //   Robot-relative = 135° - 0° = 135° (which is LEFT of robot)
            // But you said turret should turn RIGHT <90° to (-72, 72) from (0,0,0°)

            // Let me verify the math:
            // At (0,0) heading 0°, target at (-72, 72)
            // Field angle = atan2(72, -72) = atan2(1, -1) = 135° (upper left quadrant)
            // Robot heading = 0° (facing right/positive X)
            // So target is 135° in field, robot faces 0°
            // Robot-relative angle = 135° - 0° = 135° (target is to robot's left)

            // But you said turret should turn RIGHT...
            // This suggests turret 0° might not align with robot heading 0°
            // OR there's a sign convention difference

            // Let's use the math that works with standard conventions:
            double angleToTargetRobot = angleToTargetField - robotHeading;

            // Normalize to -PI to PI
            while (angleToTargetRobot > Math.PI) angleToTargetRobot -= 2 * Math.PI;
            while (angleToTargetRobot < -Math.PI) angleToTargetRobot += 2 * Math.PI;

            // Convert to degrees
            double targetAngleDeg = Math.toDegrees(angleToTargetRobot);

            // Current turret angle
            double currentAngleDeg = getTurretAngleDeg();

            // Calculate error (shortest path)
            double error = targetAngleDeg - currentAngleDeg;

            // Wrap error to -180 to 180
            while (error > 180) error -= 360;
            while (error < -180) error += 360;

            // Check if direct path violates limits
            double projectedAngle = currentAngleDeg + error;

            if (projectedAngle < TURRET_MIN_DEG || projectedAngle > TURRET_MAX_DEG) {
                // Try going the other way
                double altError = error > 0 ? error - 360 : error + 360;
                double altProjected = currentAngleDeg + altError;

                if (altProjected >= TURRET_MIN_DEG && altProjected <= TURRET_MAX_DEG) {
                    error = altError;
                } else {
                    // Can't reach target within limits
                    if (currentAngleDeg < TURRET_MIN_DEG) {
                        error = TURRET_MIN_DEG - currentAngleDeg;
                    } else if (currentAngleDeg > TURRET_MAX_DEG) {
                        error = TURRET_MAX_DEG - currentAngleDeg;
                    } else {
                        // At limit, don't move further
                        if ((currentAngleDeg <= TURRET_MIN_DEG + 1 && error < 0) ||
                                (currentAngleDeg >= TURRET_MAX_DEG - 1 && error > 0)) {
                            error = 0;
                        }
                    }
                }
            }

            // PID Control
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

            // Safety: don't drive into limits
            if ((currentAngleDeg <= TURRET_MIN_DEG && power < 0) ||
                    (currentAngleDeg >= TURRET_MAX_DEG && power > 0)) {
                power = 0;
            }

            turret.setPower(power);

            // Detailed telemetry for debugging
            telemetry.addLine("=== ROBOT POSE (Field Frame) ===");
            telemetry.addData("X", "%.1f in", robotX);
            telemetry.addData("Y", "%.1f in", robotY);
            telemetry.addData("Heading", "%.1f°", Math.toDegrees(robotHeading));

            telemetry.addLine();
            telemetry.addLine("=== TARGET ANALYSIS ===");
            telemetry.addData("Target", "(%.0f, %.0f)", TARGET_X, TARGET_Y);
            telemetry.addData("Delta X, Y", "(%.1f, %.1f)", dx, dy);
            telemetry.addData("Distance", "%.1f in", Math.sqrt(dx*dx + dy*dy));
            telemetry.addData("Field Angle to Target", "%.1f°", Math.toDegrees(angleToTargetField));
            telemetry.addData("Robot-Relative Angle", "%.1f°", targetAngleDeg);

            telemetry.addLine();
            telemetry.addLine("=== TURRET ===");
            telemetry.addData("Current Angle", "%.1f°", currentAngleDeg);
            telemetry.addData("Target Angle", "%.1f°", targetAngleDeg);
            telemetry.addData("Error", "%.1f°", error);
            telemetry.addData("Power", "%.3f", power);
            telemetry.addData("Encoder Ticks", turret.getCurrentPosition());

            // Visual feedback
            if (Math.abs(error) < 3) {
                telemetry.addLine();
                telemetry.addLine("✓✓✓ ON TARGET ✓✓✓");
            } else if (error > 0) {
                telemetry.addData("Direction", "Turning CCW (left) %.1f°", error);
            } else {
                telemetry.addData("Direction", "Turning CW (right) %.1f°", -error);
            }

            telemetry.addLine();
            telemetry.addLine("=== EXPECTED BEHAVIOR ===");
            telemetry.addLine("At (0,0) heading 0°:");
            telemetry.addLine("Target (-72,72) → Field angle 135°");
            telemetry.addLine("Robot-relative: 135° (left)");
            telemetry.addLine("If turret should turn RIGHT,");
            telemetry.addLine("try toggling INVERT_TURRET_DIRECTION");

            telemetry.update();
        }
    }

    double getTurretAngleDeg() {
        return (turret.getCurrentPosition() / TICKS_PER_TURRET_REV) * 360.0;
    }
}