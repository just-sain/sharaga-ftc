package org.firstinspires.ftc.teamcode.prod;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.prod.auto.SimplifiedOdometryRobot;
import org.firstinspires.ftc.teamcode.prod.subsystems.Intake;
import org.firstinspires.ftc.teamcode.prod.subsystems.LiftPID;
import org.firstinspires.ftc.teamcode.prod.subsystems.Outtake;

@Config
@Autonomous(name="right start auto", group = "prod")
public class RightStartAuto extends LinearOpMode
{
    // get an instance of the "Robot" class.
    private SimplifiedOdometryRobot robot = new SimplifiedOdometryRobot(this);

    public static double testDistance = 700;
    public static long testMilliseconds = 400;

    @Override public void runOpMode()
    {
        // Initialize the robot hardware & Turn on telemetry
        robot.initialize(true, hardwareMap);

        // Wait for driver to press start
        telemetry.addData(">", "Touch Play to run Auto");
        telemetry.update();

        robot.resetEncoders();

        waitForStart();
        robot.resetHeading();  // Reset heading to set a baseline for Auto

        // Run Auto if stop was not pressed.
        if (opModeIsActive())
        {
            // Note, this example takes more than 30 seconds to execute, so turn OFF the auto timer.

            // 1 cm / 2.54 inch, 10 cm = 3.937 inch

//            robot.turnTo(180, 0.4, 0.1);

//            if (true) {
//                robot.strafeEncoder(5000, 0.3, 500);
//            }

            if (true) {
                // setting sample
                robot.liftPID.setTargetPosition(LiftPID.Position.CHAMBER);
                robot.driveEncoder(12500, 0.3, 0); // 111.76 cm - 29.527
                robot.liftPID.setTargetPosition(LiftPID.Position.SET_CHAMBER);

                robot.liftPID.periodic();
                sleep(10);
                robot.liftPID.periodic();
                sleep(10);
                robot.liftPID.periodic();
                sleep(10);
                robot.liftPID.periodic();
                sleep(10);
                robot.liftPID.periodic();
                sleep(10);
                robot.liftPID.periodic();
                sleep(10);
                robot.liftPID.periodic();
                sleep(10);
                robot.liftPID.periodic();
                sleep(10);
                robot.liftPID.periodic();
                sleep(10);
                robot.liftPID.periodic();
                sleep(10);
                robot.liftPID.periodic();
                sleep(10);
                robot.liftPID.periodic();
                sleep(10);
                robot.liftPID.periodic();
                sleep(10);
                robot.liftPID.periodic();
                sleep(10);
                robot.liftPID.periodic();
                sleep(10);
                robot.liftPID.periodic();
                sleep(10);
                robot.liftPID.periodic();
                sleep(10);
                robot.liftPID.periodic();
                sleep(10);
                robot.liftPID.periodic();
                sleep(10);
                robot.liftPID.periodic();
                sleep(10);
                robot.liftPID.periodic();
                sleep(10);
                robot.liftPID.periodic();
                sleep(10);
                robot.liftPID.periodic();
                sleep(10);
                robot.liftPID.periodic();
                sleep(10);
                robot.liftPID.periodic();
                sleep(10);
                robot.liftPID.periodic();
                sleep(10);
                robot.liftPID.periodic();
                sleep(10);
                robot.liftPID.periodic();
                sleep(10);
                robot.liftPID.periodic();

                robot.outtake.setClawOpen();

                // reset
                robot.resetEncoders();
                sleep(300);

                robot.driveEncoder(-2000, -0.3, 0);

                // reset
                robot.resetEncoders();
                sleep(300);

                robot.strafeEncoder(15000, -0.4, 0);

                // reset
                robot.resetEncoders();
                sleep(500);

                robot.turnRobotEncoder(10200, 0.3, 0);

                // reset
                robot.resetEncoders();
                sleep(300);

                robot.driveEncoder(11500, 0.3, 0);

                robot.resetEncoders();
                sleep(300);

                robot.liftPID.setTargetPosition(LiftPID.Position.HOME);

                robot.liftPID.periodic();
                sleep(100);
                robot.liftPID.periodic();
                sleep(70);
                robot.liftPID.periodic();
                sleep(50);
                robot.liftPID.periodic();
                sleep(30);
                robot.liftPID.periodic();
                sleep(20);
                robot.liftPID.periodic();
                sleep(10);
                robot.liftPID.periodic();
                sleep(10);
                robot.liftPID.periodic();
                sleep(10);
                robot.liftPID.periodic();
                sleep(10);
                robot.liftPID.periodic();

                robot.stopRobot();

                sleep(300);

                robot.outtake.setClawClose();

                sleep(500);

                robot.liftPID.setTargetPosition(LiftPID.Position.CHAMBER);

                robot.liftPID.periodic();
                sleep(10);
                robot.liftPID.periodic();
                sleep(10);
                robot.liftPID.periodic();
                sleep(10);
                robot.liftPID.periodic();
                sleep(10);
                robot.liftPID.periodic();
                sleep(10);
                robot.liftPID.periodic();
                sleep(10);
                robot.liftPID.periodic();
                sleep(10);
                robot.liftPID.periodic();
                sleep(10);
                robot.liftPID.periodic();
                sleep(10);
                robot.liftPID.periodic();
                sleep(10);
                robot.liftPID.periodic();
                sleep(10);
                robot.liftPID.periodic();
                sleep(10);
                robot.liftPID.periodic();
                sleep(10);
                robot.liftPID.periodic();
                sleep(10);
                robot.liftPID.periodic();
                sleep(10);
                robot.liftPID.periodic();
                sleep(10);
                robot.liftPID.periodic();
                sleep(10);
                robot.liftPID.periodic();
                sleep(10);
                robot.liftPID.periodic();
                sleep(10);
                robot.liftPID.periodic();
                sleep(10);
                robot.liftPID.periodic();

                // reset
                robot.resetEncoders();
                sleep(300);

                robot.driveEncoder(11000, -0.3, 0);

                robot.resetEncoders();
                sleep(300);

                robot.strafeEncoder(15000, -0.4, 0);

                // reset
                robot.resetEncoders();
                sleep(500);

                robot.turnRobotEncoder(9500, -0.3, 0);

                // reset
                robot.resetEncoders();
                sleep(300);

                robot.driveEncoder(3250, 0.3, 0);


                robot.liftPID.setTargetPosition(LiftPID.Position.SET_CHAMBER);

                robot.liftPID.periodic();
                sleep(10);
                robot.liftPID.periodic();
                sleep(10);
                robot.liftPID.periodic();
                sleep(10);
                robot.liftPID.periodic();
                sleep(10);
                robot.liftPID.periodic();
                sleep(10);
                robot.liftPID.periodic();
                sleep(10);
                robot.liftPID.periodic();
                sleep(10);
                robot.liftPID.periodic();
                sleep(10);
                robot.liftPID.periodic();
                sleep(10);
                robot.liftPID.periodic();
                sleep(10);
                robot.liftPID.periodic();
                sleep(10);
                robot.liftPID.periodic();
                sleep(10);
                robot.liftPID.periodic();
                sleep(10);
                robot.liftPID.periodic();
                sleep(10);
                robot.liftPID.periodic();
                sleep(10);
                robot.liftPID.periodic();
                sleep(10);
                robot.liftPID.periodic();
                sleep(10);
                robot.liftPID.periodic();
                sleep(10);
                robot.liftPID.periodic();
                sleep(10);
                robot.liftPID.periodic();
                sleep(10);
                robot.liftPID.periodic();
                sleep(10);
                robot.liftPID.periodic();
                sleep(10);
                robot.liftPID.periodic();
                sleep(10);
                robot.liftPID.periodic();
                sleep(10);
                robot.liftPID.periodic();
                sleep(10);
                robot.liftPID.periodic();
                sleep(10);
                robot.liftPID.periodic();
                sleep(10);
                robot.liftPID.periodic();
                sleep(10);
                robot.liftPID.periodic();

                robot.resetEncoders();
                robot.outtake.setClawOpen();
                sleep(400);

                robot.driveEncoder(10000, -0.3, 0);

                robot.resetEncoders();

                robot.strafeEncoder(15000, -0.6, 0);


                // setting to the chamber

                telemetry.addData("drive", robot.driveEncoder.getCurrentPosition());
                telemetry.addData("strafe", robot.strafeEncoder.getCurrentPosition());
            }
        }
    }
}
