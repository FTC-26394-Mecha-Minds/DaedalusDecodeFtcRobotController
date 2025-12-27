package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot;

@TeleOp
public class MechaRedTeleOp extends Robot {
    @Override
    public void init() {
        super.init();
    }
    @Override
    public void loop() {
        super.loop();
        super.getio().light();
        super.getDrive().MecanumDrive(gamepad1.left_stick_y, -gamepad1.right_stick_x, -gamepad1.left_stick_x, 0.8);
        if (gamepad1.a) {
            super.getio().runIntake();
        } else {
            super.getio().stopIntake();
        }
    }
}
