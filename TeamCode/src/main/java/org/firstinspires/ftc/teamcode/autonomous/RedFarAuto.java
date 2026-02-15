package org.firstinspires.ftc.teamcode.autonomous;


import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.MecanumDrive;

@Autonomous (name = "RedFarAuto", group = "Autonomous", preselectTeleOp = "MechaRedTeleOp")
public class RedFarAuto extends LinearOpMode {
    public enum shootingStates {
        START,
        Latch_OPEN,
        Latch_TRANSFER,
        Latch_CLOSE,
        BUFFER,
        DONE
    }
    shootingStates shootingState = shootingStates.START;


    public class IO {
        private DcMotorEx intake, outtakeOne, outtakeTwo, turret;
        private Servo lights, stopper, latch;
        ElapsedTime timer = new ElapsedTime();
        public double p = 80, ff = 20, curTargetVelocity = 1450;

        public IO(HardwareMap hardwareMap) {
            stopper = hardwareMap.get(Servo.class, "stopper");
            lights = hardwareMap.get(Servo.class, "lights");
            latch = hardwareMap.get(Servo.class, "latch");
            intake = hardwareMap.get(DcMotorEx.class, "intake");
            intake.setDirection(DcMotorEx.Direction.REVERSE);
            turret = hardwareMap.get(DcMotorEx.class, "turret");
            outtakeOne = hardwareMap.get(DcMotorEx.class, "outtakeOne");
            outtakeTwo = hardwareMap.get(DcMotorEx.class, "outtakeTwo");
            outtakeOne.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            outtakeTwo.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            PIDFCoefficients pidf = new PIDFCoefficients(p, 0, 0, ff);
            outtakeTwo.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf);
            outtakeOne.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf);
        }
        public class transfer implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (shootingState == shootingStates.START) {
                    timer.reset();
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
                        if (timer.milliseconds()>2000) {
                            stopper.setPosition(0.15);
                            latch.setPosition(0.2);
                            timer.reset();
                            shootingState = shootingStates.BUFFER;
                        }
                        break;
                    case BUFFER:
                        lights.setPosition(0.5);
                        if (timer.milliseconds()>750) {
                            latch.setPosition(0.45);
                            timer.reset();
                            shootingState = shootingStates.Latch_CLOSE;
                        }
                        break;
                    case Latch_CLOSE:
                        lights.setPosition(0.5);
                        stopper.setPosition(0.55);
                        timer.reset();
                        shootingState = shootingStates.DONE;
                        break;
                    case DONE:
                        lights.setPosition(0.5);
                        intake.setPower(0);
                        shootingState = shootingStates.START;
                        return false;
                }
                return true;
            }
        }
        public Action transfer() {
            return new transfer();
        }
        public class intakeRun implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                intake.setPower(1);
                return false;
            }
        }
        public Action intakeRun() {
            return new intakeRun();
        }

        public class intakeStop implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                intake.setPower(0);
                return false;
            }
        }
        public Action intakeStop() {
            return new intakeStop();
        }
        public class turretLock implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                turret.setTargetPosition(0);
                turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                turret.setPower(0.4);
                return false;
            }
        }
        public Action turretLock() {
            return new turretLock();
        }
        public class shoot implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                PIDFCoefficients pidf = new PIDFCoefficients(p, 0, 0, ff);
                outtakeTwo.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf);
                outtakeOne.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf);
                outtakeOne.setVelocity(curTargetVelocity);
                outtakeTwo.setPower(outtakeOne.getPower());
                return false;
            }
        }
        public Action shoot() {
            return new shoot();
        }

        public class shootStop implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                outtakeOne.setPower(0);
                outtakeTwo.setPower(0);
                return false;
            }
        }
        public Action shootStop() {
            return new shootStop();
        }

        public class lightsOn implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                lights.setPosition(0.279);
                return false;
            }
        }
        public Action lightsOn() {
            return new lightsOn();
        }
    }

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        Pose2d initialPose = new Pose2d(60, 16, Math.PI);
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
        IO io = new IO(hardwareMap);
        waitForStart();

        TrajectoryActionBuilder shootPreload = drive.actionBuilder(initialPose)
                .afterTime(0,io.shoot())
                .splineToLinearHeading(new Pose2d(50, 14, Math.toRadians(155)),Math.PI)
                .afterTime(1, io.intakeRun())
                .afterTime(1, io.transfer())
                .waitSeconds(3);
        TrajectoryActionBuilder shootOne = drive.actionBuilder(new Pose2d(34, 62, Math.toRadians(270)))
                .afterTime(0, io.shoot())
                .splineToLinearHeading(new Pose2d(50, 14, Math.toRadians(155)),Math.PI)
                .afterTime(0, io.intakeRun())
                .afterTime(0, io.transfer())
                .waitSeconds(3);
        TrajectoryActionBuilder shootTwo = drive.actionBuilder(new Pose2d(58, 64, Math.toRadians(270)))
                .afterTime(0, io.shoot())
                .splineToLinearHeading(new Pose2d(50, 14, Math.toRadians(155)),Math.PI)
                .afterTime(0, io.intakeRun())
                .afterTime(0, io.transfer())
                .waitSeconds(3);
        TrajectoryActionBuilder intakeOne = drive.actionBuilder(new Pose2d(50, 14, Math.toRadians(155)))
                .turn(Math.toRadians(270))
                .waitSeconds(0)
                .splineToLinearHeading(new Pose2d(34, 30, Math.toRadians(270)), Math.PI)
                .waitSeconds(0)
                .lineToYConstantHeading(62);
        TrajectoryActionBuilder intakeTwo = drive.actionBuilder(new Pose2d(50, 14, Math.toRadians(155)))
                .turnTo(Math.toRadians(270))
                .waitSeconds(0)
                .splineToLinearHeading(new Pose2d(58, 64, Math.toRadians(270)), Math.PI)
                .waitSeconds(0.5);
        TrajectoryActionBuilder intakeThree = drive.actionBuilder(new Pose2d(50, 14, Math.toRadians(155)))
                .turnTo(Math.toRadians(270))
                .waitSeconds(0)
                .splineToLinearHeading(new Pose2d(58, 64, Math.toRadians(270)), Math.PI)
                .waitSeconds(0.5);



        if (isStopRequested()) {return;}
        Actions.runBlocking(
                new ParallelAction(
                        io.turretLock(),
                        io.lightsOn(),
                        new SequentialAction(
                                shootPreload.build(),
                                io.shootStop(),
                                new ParallelAction(
                                        io.intakeRun(),
                                        new SequentialAction(
                                                intakeOne.build(),
                                                shootOne.build()
                                        )
                                ),
                                new ParallelAction(
                                        io.intakeRun(),
                                        new SequentialAction(
                                                intakeTwo.build(),
                                                shootTwo.build()
                                        )
                                ),
                                new ParallelAction(
                                        io.intakeRun(),
                                        intakeThree.build()
                                )
                        )
                )
        );


    }
}