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
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name = "MechaRedTeleOp")
public class MechaRedTeleOp extends LinearOpMode {
    private DcMotor fL, fR, bL, bR, intake, outtakeOne, outtakeTwo, turret;
    private Servo lights, stopper;



    @Override
    public void runOpMode() throws InterruptedException {

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        fL = hardwareMap.dcMotor.get("fL");
        bL = hardwareMap.dcMotor.get("bL");
        fR = hardwareMap.dcMotor.get("fR");
        bR = hardwareMap.dcMotor.get("bR");
        intake = hardwareMap.dcMotor.get("intake");
        outtakeOne = hardwareMap.dcMotor.get("outtakeOne");
        outtakeTwo = hardwareMap.dcMotor.get("outtakeTwo");
        turret = hardwareMap.dcMotor.get("turret");
//        encoder = hardwareMap.dcMotor.get("encoder");


        stopper = hardwareMap.get(Servo.class, "stopper");
        lights = hardwareMap.get(Servo.class, "lights");


        fL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        encoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        encoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        intake.setDirection(DcMotor.Direction.REVERSE);
        fR.setDirection(DcMotor.Direction.REVERSE);
        bR.setDirection(DcMotor.Direction.REVERSE);
        double maxSpeed = 1;
        boolean lastY = false;



        double f, r, s;
        double fLeftPower, bLeftPower, fRightPower, bRightPower;

        waitForStart();
        // Rising/Falling Edge Detector
        Gamepad lastGamepad2 = new Gamepad();

        // Initial Positions
        stopper.setPosition(0.5);
        if (opModeIsActive()) {
            while (opModeIsActive()) {
//                int position = encoder.getCurrentPosition();
//                telemetry.addData("Encoder", position);
                telemetry.update();
                lights.setPosition(0.279);

                if (gamepad1.a) {
                    intake.setPower(1);
                } else {
                    intake.setPower(0);
                }
                if (gamepad1.b) {
                    stopper.setPosition(0.2);
                } else if (gamepad1.y) {
                    stopper.setPosition(0.5);
                } else if (gamepad1.x) {
                    stopper.setPosition(0.65);
                }
                if (gamepad2.dpad_up) {
                    outtakeOne.setPower(0.6);
                    outtakeTwo.setPower(0.6);
                }
                if (gamepad2.dpad_down) {
                    outtakeOne.setPower(0);
                    outtakeTwo.setPower(0);
                }
                if (gamepad2.dpad_right) {
                    outtakeOne.setPower(0.8);
                    outtakeTwo.setPower(0.8);
                }
                if (gamepad2.a) {
                    stopper.setPosition(0.5);
                } else if (gamepad2.y) {
                    stopper.setPosition(0);
                }

                if (gamepad2.left_bumper) {
                    turret.setPower(1);
                } else if (gamepad2.right_bumper) {
                    turret.setPower(-1);
                } else {
                    turret.setPower(0);
                }

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

