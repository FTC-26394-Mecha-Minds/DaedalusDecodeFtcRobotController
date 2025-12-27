package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.Subsystems.IO;
import org.firstinspires.ftc.teamcode.Subsystems.Subsystem;

public class Robot extends OpMode {
    Subsystem[] subsystems;

    DcMotorEx fL, fR, bL, bR, intake, outtakeOne, outtakeTwo, turret;
    DcMotorEx[] driveMotors;
    Servo stopper, lights;
    Drivetrain drive;
    IO io;
    ElapsedTime timer;
    double dt;

    @Override
    public void init() {
        initDrive();
        initIO();
        subsystems = new Subsystem[]{drive, io};
        timer = new ElapsedTime();
    }
    @Override
    public void init_loop() {
        super.init_loop();
        dt = timer.seconds();
        timer.reset();
    }
    @Override
    public void start() {
        for (DcMotorEx motor : driveMotors) {
            motor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        }
        timer.reset();
    }
    @Override
    public void loop() {
        dt = timer.seconds();
        timer.reset();
        for (Subsystem system: subsystems) {
            system.update(getDt(), telemetry);
        }
        telemetry.update();
    }
    public void initDrive() {
        fL = hardwareMap.get(DcMotorEx.class, "fL");
        fR = hardwareMap.get(DcMotorEx.class, "fR");
        bL = hardwareMap.get(DcMotorEx.class, "bL");
        bR = hardwareMap.get(DcMotorEx.class, "bR");

        fR.setDirection(DcMotorEx.Direction.REVERSE);
        bR.setDirection(DcMotorEx.Direction.REVERSE);

        driveMotors = new DcMotorEx[]{fL, fR, bL, bR};

        for (DcMotorEx motor : driveMotors) {
            motor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        }

        drive = new Drivetrain(fL, fR, bL, bR);
    }

    public void initIO() {
        outtakeOne = hardwareMap.get(DcMotorEx.class, "outtakeOne");
        outtakeTwo = hardwareMap.get(DcMotorEx.class, "outtakeTwo");
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        turret = hardwareMap.get(DcMotorEx.class, "turret");
        stopper = hardwareMap.get(Servo.class, "stopper");
        lights = hardwareMap.get(Servo.class, "lights");

        //Reverse if necessary here

        io = new IO(intake, outtakeOne, outtakeTwo, turret, stopper, lights);
    }

    public double getDt() {
        return dt;
    }
    public Drivetrain getDrive() {
        return drive;
    }
    public IO getio() {
        return io;
    }

}
