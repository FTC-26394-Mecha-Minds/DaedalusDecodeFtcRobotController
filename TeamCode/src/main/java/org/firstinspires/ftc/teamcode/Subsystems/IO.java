package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class IO implements Subsystem {

    DcMotorEx intake, outtakeOne, outtakeTwo, turret;
    Servo stopper, lights;

    @Override
    public void update(double dt, Telemetry telemetry) {

    }
    @Override
    public void stop() {

    }
    public IO(DcMotorEx intake, DcMotorEx outtakeOne, DcMotorEx outtakeTwo, DcMotorEx turret, Servo stopper, Servo lights) {
        this.intake = intake;
        this.outtakeOne = outtakeOne;
        this.outtakeTwo = outtakeTwo;
        this.turret = turret;
        this.stopper = stopper;
        this.lights = lights;
    }
    //All methods go here.
    public void runIntake() {
        intake.setPower(1);
    }
    public void stopIntake() {
        intake.setPower(0);
    }

    public void light() {
        lights.setPosition(0.279);
    }
}
