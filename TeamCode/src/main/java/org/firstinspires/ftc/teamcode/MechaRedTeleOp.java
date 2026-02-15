package org.firstinspires.ftc.teamcode;

import android.graphics.Camera;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.autonomous.RedClassifierAuto;

@TeleOp(name = "MechaRedTeleOp")
public class MechaRedTeleOp extends LinearOpMode {
    private DcMotorEx fL, fR, bL, bR, intake, outtakeOne, outtakeTwo, turret;
    private Servo lights, stopper, latch;
    public static double p = 80, ff = 20, curTargetVelocity = 0;


    public enum shootingStates {
        START,
        Latch_OPEN,
        Latch_TRANSFER,
        BUFFER,
        Latch_CLOSE,
        DONE
    }
    shootingStates shootingState = shootingStates.START;


    @Override
    public void runOpMode() throws InterruptedException {
        ElapsedTime timer = new ElapsedTime();
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        fL = hardwareMap.get(DcMotorEx.class, "fL");
        bL = hardwareMap.get(DcMotorEx.class, "bL");
        fR = hardwareMap.get(DcMotorEx.class, "fR");
        bR = hardwareMap.get(DcMotorEx.class, "bR");
        turret = hardwareMap.get(DcMotorEx.class, "turret");
        outtakeOne = hardwareMap.get(DcMotorEx.class, "outtakeOne");
        outtakeTwo = hardwareMap.get(DcMotorEx.class, "outtakeTwo");
        intake = hardwareMap.get(DcMotorEx.class, "intake");

        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        outtakeOne.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        outtakeTwo.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        PIDFCoefficients pidf = new PIDFCoefficients(p, 0, 0, ff);
        outtakeTwo.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf);
        outtakeOne.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf);

        latch = hardwareMap.get(Servo.class, "latch");
        stopper = hardwareMap.get(Servo.class, "stopper");
        lights = hardwareMap.get(Servo.class, "lights");


        fL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        intake.setDirection(DcMotor.Direction.REVERSE);
        fR.setDirection(DcMotor.Direction.REVERSE);
        bR.setDirection(DcMotor.Direction.REVERSE);
        double maxSpeed = 1;
        boolean lastY = false;
        boolean intakeToggle = false;
        Gamepad currentGamepad1 = new Gamepad();
        Gamepad previousGamepad1 = new Gamepad();



        double f, r, s;
        double fLeftPower, bLeftPower, fRightPower, bRightPower;

        waitForStart();
        // Rising/Falling Edge Detector
        Gamepad lastGamepad2 = new Gamepad();

        // Initial Positions
        latch.setPosition(0.45);
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setTargetPosition(0);
        turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        turret.setPower(0.4);
        stopper.setPosition(0.55);
        if (opModeIsActive()) {
            while (opModeIsActive()) {
                previousGamepad1.copy(currentGamepad1);
                currentGamepad1.copy(gamepad1);
                if (currentGamepad1.a && !previousGamepad1.a) {
                    intakeToggle = !intakeToggle;
                }
                telemetry.update();
                lights.setPosition(0.279);
                double val = turret.getCurrentPosition();

                PIDFCoefficients pidfS = new PIDFCoefficients(p, 0, 0, ff);
                outtakeOne.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfS);
                outtakeTwo.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfS);

                if (gamepad1.dpad_left) {
                    stopper.setPosition(0.15);
                    latch.setPosition(0.2);
                    timer.reset();
                    shootingState = shootingStates.Latch_CLOSE;
                }

                if (gamepad2.dpad_up) {
                    curTargetVelocity = 1150;
                }
                if (gamepad2.dpad_right) {
                    curTargetVelocity = 1325;
                }
                if (gamepad2.dpad_down) {
                    curTargetVelocity = 0;
                }

                outtakeOne.setVelocity(curTargetVelocity);
                outtakeTwo.setPower(outtakeOne.getPower());

                double curVelocity = outtakeOne.getVelocity();
                double error = curTargetVelocity - curVelocity;

                telemetry.addData("Turret Pos", val);
                telemetry.addData("Target Vel", curTargetVelocity);
                telemetry.addData("Current Vel", curVelocity);
                telemetry.addData("Error", "%.2f", error);


                boolean yPressed = gamepad1.y;
                boolean yRisingEdge = yPressed && !lastY;
                lastY = yPressed;


                if (yRisingEdge && shootingState == shootingStates.START) {
                    timer.reset();
                    latch.setPosition(0.2);
                    shootingState = shootingStates.Latch_OPEN;
                }

                switch (shootingState) {
                    case Latch_OPEN:
                        lights.setPosition(0.5);
                        latch.setPosition(0.2);
                        timer.reset();
                        shootingState = shootingStates.Latch_TRANSFER;
                        break;
                    case Latch_TRANSFER:
                        lights.setPosition(0.5);
                        if (gamepad1.dpad_left) {
                            latch.setPosition(0.2);
                            stopper.setPosition(0.15);
                            timer.reset();
                            shootingState = shootingStates.Latch_CLOSE;
                        }
                        break;
//                    case BUFFER:
//                        latch.setPosition(0.2);
//                        lights.setPosition(0.5);
//                        if (timer.milliseconds()>300) {
//                            timer.reset();
//                            shootingState = shootingStates.Latch_CLOSE;
//                        }
//                        break;
                    case Latch_CLOSE:
                        lights.setPosition(0.5);
                        if (timer.milliseconds()>150) {
                            stopper.setPosition(0.55);
                            timer.reset();
                            shootingState = shootingStates.DONE;
                        }
                        break;
                    case DONE:
                        lights.setPosition(0.5);
                        latch.setPosition(0.45);
                        shootingState = shootingStates.START;
                }


                if (intakeToggle) {
                    intake.setPower(1);
                } else if (gamepad1.a && gamepad1.left_bumper){
                    intake.setPower(-1);
                } else {
                    intake.setPower(0);
                }
                if (gamepad1.b) {
                    stopper.setPosition(0.15);
                }

//                if (gamepad1.dpad_up) {
//                    outtakeOne.setPower(0.7);
//                    outtakeTwo.setPower(0.7);
//                }
//                if (gamepad1.dpad_down) {
//                    outtakeOne.setPower(0);
//                    outtakeTwo.setPower(0);
//                }
//                if (gamepad1.dpad_right) {
//                    outtakeOne.setPower(0.9);
//                    outtakeTwo.setPower(0.9);
//                }

                int turretTarget = turret.getTargetPosition();

                if (gamepad2.left_bumper) {
                    turretTarget+= 120;
                } else if (gamepad2.right_bumper) {
                    turretTarget-= 120;
                }
                turret.setTargetPosition(turretTarget);
                turret.setPower(1);
//
//                if (gamepad2.left_bumper) {
//                    turret.setPower(1);
//                } else if (gamepad2.right_bumper) {
//                    turret.setPower(-1);
//                } else {
//                    turret.setPower(0);
//                }

                f = gamepad1.left_stick_y;
                r = -gamepad1.right_stick_x;
                s = -gamepad1.left_stick_x;
                fLeftPower = (f + r + s);
                bLeftPower = (f + r - s);
                fRightPower = (f - r - s);
                bRightPower = (f - r + s);
                double maxN = Math.max(Math.abs(fLeftPower), Math.max(Math.abs(bLeftPower),
                        Math.max(Math.abs(fRightPower), Math.abs(bRightPower))));
                if (maxN > 1) {
                    fLeftPower /= maxN;
                    bLeftPower /= maxN;
                    fRightPower /= maxN;
                    bRightPower /= maxN;
                }
                fL.setPower(fLeftPower * maxSpeed);
                bL.setPower(bLeftPower * maxSpeed);
                fR.setPower(fRightPower * maxSpeed);
                bR.setPower(bRightPower * maxSpeed);

            }
        }
    }
}

