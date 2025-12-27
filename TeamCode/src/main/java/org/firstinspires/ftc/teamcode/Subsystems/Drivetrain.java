package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Drivetrain implements Subsystem{
    DcMotorEx fL, fR, bl, bR;
    DcMotorEx[] driveMotors;

    public Drivetrain(DcMotorEx fL, DcMotorEx fR, DcMotorEx bL, DcMotorEx bR) {
        this.fL= fL;
        this.fR = fR;
        this.bl= bL;
        this.bR=bR;
        driveMotors = new DcMotorEx[]{fL, fR, bL, bR};
    }

    @Override
    public void update(double dt, Telemetry telemetry)  {

    }
    @Override
    public void stop() {
        setDriveMotors(0);
    }

    public void setDriveMotors(double power) {
        for (DcMotorEx i:driveMotors) {
            i.setPower(power);
        }
    }

    public void MecanumDrive(double f, double r, double s, double maxSpeed){

        double fLeftPower = (f + r + s);
        double bleftPower = (f + r - s);
        double fRightPower = (f - r - s);
        double bRightPower = (f - r + s);
        double maxN = Math.max(Math.abs(fLeftPower), Math.max(Math.abs(bleftPower),
                Math.max(Math.abs(fRightPower), Math.abs(bRightPower))));
        if (maxN > 1) {
            fLeftPower /= maxN;
            bleftPower /= maxN;
            fRightPower /= maxN;
            bRightPower /= maxN;
        }
        fL.setPower(fLeftPower * maxSpeed);
        bl.setPower(bleftPower * maxSpeed);
        fR.setPower(fRightPower * maxSpeed);
        bR.setPower(bRightPower * maxSpeed);
    }
}
