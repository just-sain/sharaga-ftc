package org.firstinspires.ftc.teamcode.prod;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.prod.subsystems.ArmPID;
import org.firstinspires.ftc.teamcode.prod.subsystems.Base;
import org.firstinspires.ftc.teamcode.prod.subsystems.Intake;
import org.firstinspires.ftc.teamcode.prod.subsystems.LiftPID;
import org.firstinspires.ftc.teamcode.prod.subsystems.Outtake;

@TeleOp(name = "main tele op", group = "prod")
public class MainTeleOp extends LinearOpMode {

    // subsystems
    private Base base = new Base();
    private LiftPID liftPID = new LiftPID();
//    private ArmPID armPID = new ArmPID();
//    private Intake intake = new Intake();
//    private Outtake outtake = new Outtake();

    @Override
    public void runOpMode() {

        // initializing
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // subsystems
        base.initialize(hardwareMap, DcMotor.ZeroPowerBehavior.BRAKE, DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        liftPID.initialize(hardwareMap);
//        armPID.initialize(hardwareMap);
//        intake.initialize(hardwareMap);
//        outtake.initialize(hardwareMap);

        // waiting for start
        telemetry.addData("status", "ready for start");
        waitForStart();

        while(opModeIsActive()) {

            // --- start keymap of robot ---
            // -- base --
            base.controlBySticks(
                    gamepad1.left_stick_y,
                    gamepad1.right_stick_x,
                    gamepad1.left_stick_x
            );

            // -- arm pid --
//            if (gamepad1.a) {
//                // home pos
//                armPID.setTargetPosition(ArmPID.Position.HOME);
//            } else if (gamepad1.b) {
//                // middle
//                armPID.setTargetPosition(ArmPID.Position.MIDDLE);
//            } else if (gamepad1.y) {
//                // long
//                armPID.setTargetPosition(ArmPID.Position.LONG);
//            }

            // -- arm pid mode switch --
//            if (gamepad1.left_trigger > 0.05) {
//                // man mode - down
//                armPID.setPower(-gamepad1.left_trigger);
//            }
//            if (gamepad1.right_trigger > 0.05) {
//                // man mode - up
//                armPID.setPower(gamepad1.right_trigger);
//            } else {
//                if (armPID.getIsManMode()) {
//                    armPID.setTargetToCurrentPos();
//                } else if (!armPID.getIsManMode()) {
//                    armPID.periodic();
//                }
//            }

            // -- resetting arm encoders, just in case --
//            if (gamepad1.right_stick_button) {
//                armPID.resetEncoders();
//            }

            // --- intake ---
            // -- intake elbow --
//            if (gamepad2.a) {
//                // intake take pos
//                intake.setElbowTake();
//                intake.setWristTake();
//            } else if (gamepad2.b) {
//                intake.setElbowThrow();
//                intake.setWristTake();
//            } else if (gamepad2.y) {
//                intake.setElbowBoxPos();
//                intake.setWristUp();
//                intake.setClawClose();
//            } else if (gamepad2.x) {
//                intake.setElbowBoxPos();
//                intake.setWristUp();
//                intake.setClawClose();
//            }


            // -- intake hand --
//            if (Math.abs(gamepad2.left_stick_x) > 0.05) {
//                intake.setHandCustom(gamepad2.left_stick_x);
//            } else {
//                intake.setHandNeutral();
//            }
//
//            // -- intake claw --
//            if (gamepad2.left_bumper) {
//                // open intake claw
//                intake.setClawOpen();
//            } else if(gamepad2.right_bumper) {
//                // close intake claw
//                intake.setClawClose();
//            }

            // --- lift ---
            // -- lift setting target position --
//            if (gamepad2.dpad_down) {
//                // home pos
//                liftPID.setTargetPosition(LiftPID.Position.HOME);
//
//                // TODO: if elbow in box pos intake.setWristUp;
//                intake.setWristUp();
//            } else if (gamepad2.dpad_up) {
//                // high chamber
//                liftPID.setTargetPosition(LiftPID.Position.CHAMBER);
//
//                // TODO: if elbow in box pos intake.setWristUp;
//                intake.setWristUp();
//            }

            // -- lift pid mode switch --
            if (gamepad2.left_trigger > 0.05) {
                // man mode - down
                liftPID.setPower(-gamepad2.left_trigger);
            }
             if (gamepad2.right_trigger > 0.05) {
                // man mode - up
                liftPID.setPower(gamepad2.right_trigger);
            } else {
                 // periodic
                 if (liftPID.getIsManMode()) {
                     liftPID.setTargetToCurrentPos();
                 } else if (!liftPID.getIsManMode()) {
                     liftPID.periodic();
                 }
            }

            // -- resetting lift encoders, just in case --
            if (gamepad2.right_stick_button) {
                liftPID.resetEncoders();
            }

            // --- outtake ---
            // -- outtake claw --
//            if (gamepad1.right_bumper) {
//                outtake.setClawClose();
//            } else if (gamepad1.left_bumper) {
//                outtake.setClawOpen();
//            }
//
//            // -- outtake box --
//            if (gamepad2.dpad_left) {
//                outtake.setBoxOpen();
//            } else {
//                outtake.setBoxCLose();
//            }

            // --- end of keymap of robot ---

            // --- telemetry ---
            telemetry.addData("status", "running");

            telemetry.addLine("--- LIFT PID ---");
            liftPID.showLogs(telemetry);

//            telemetry.addLine("--- ARM PID ---");
//            armPID.showLogs(telemetry);
//
//            telemetry.addLine("--- INTAKE ---");
//            intake.showLogs(telemetry);
//
//            telemetry.addLine("--- OUTTAKE ---");
//            outtake.showLogs(telemetry);

            telemetry.update();
        }
    }

}
