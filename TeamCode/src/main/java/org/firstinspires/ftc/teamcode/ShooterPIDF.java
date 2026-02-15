package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

@Config
@TeleOp
public class ShooterPIDF extends OpMode {
    public DcMotorEx outtakeOne, outtakeTwo, intake;
    public static double highVelocity = 1500, lowVelocity = 900, curTargetVelocity = 1150; //Far side curTargetVelocity = 1350-1400
    public static double p = 80, f = 20;

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        outtakeOne = hardwareMap.get(DcMotorEx.class, "outtakeOne");
        outtakeTwo = hardwareMap.get(DcMotorEx.class, "outtakeTwo");
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        intake.setDirection(DcMotorSimple.Direction.REVERSE);
        outtakeOne.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        outtakeTwo.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        PIDFCoefficients pidf = new PIDFCoefficients(p, 0, 0, f);
        outtakeTwo.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf);
        outtakeOne.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf);
        telemetry.addLine("Initialization Complete");

    }
    @Override
    public void loop() {
        if (gamepad1.a) {
            intake.setPower(1);
        } else {
            intake.setPower(0);
        }
        PIDFCoefficients pidf = new PIDFCoefficients(p, 0, 0 ,f);
        outtakeOne.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf);
        outtakeTwo.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf);


        outtakeOne.setVelocity(curTargetVelocity);
        outtakeTwo.setPower(outtakeOne.getPower());

        double curVelocity = outtakeOne.getVelocity();
        double error = curTargetVelocity - curVelocity;

        telemetry.addData("Target Vel", curTargetVelocity);
        telemetry.addData("Current Vel", curVelocity);
        telemetry.addData("Error", "%.2f", error);


    }
}
