package org.firstinspires.ftc.teamcode.prod.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class ArmPID {
    // motors
    private DcMotorEx motor;
    // k of pid
    private double integralSum = 0;
    public static double kP = 0.004, kI = 0.0000003, kD = 0.00000005;
    // needed
    public static double target = Position.HOME.getPos();
    private ElapsedTime timer = new ElapsedTime();
    private double lastError = 0;
    // is manual or pid mode
    private boolean isManMode = false;

    // arm positions
    public static enum Position {
        HOME(0),
        MIDDLE(900),
        LONG(1700);
        // max 1800

        Position(int pos) {
            this.position = pos;
        }

        private int position;

        public int getPos() {
            return position;
        }
    }

    // initialize
    public void initialize(HardwareMap hardwareMap) {
        // getting from hardware map
        motor = hardwareMap.get(DcMotorEx.class, "arm");
        // setting direction
        motor.setDirection(DcMotorEx.Direction.FORWARD);
        // setting zero power behavior to brake
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // stop and reset encoder on the left arm motor
        motor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        // running two arm motors without encoder
        motor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        target = Position.HOME.getPos();
    }

    private double CustomPIDControl(double reference, double state) {
        double error = reference - state;
        integralSum += error * timer.seconds();
        double derivative = (error - lastError) / timer.seconds();
        lastError = error;

        timer.reset();

        double output = (error * kP) + (derivative * kD) + (integralSum * kI);
        return output;
    }

    // manual mode
    public void setPower(double power) {
        motor.setPower(power);

        isManMode = true;
    }

    // set target to current position
    public void setTargetToCurrentPos() {
        target = motor.getCurrentPosition();
        isManMode = false;
    }

    // mode handles
    public boolean getIsManMode() {
        return isManMode;
    }

    // set target
    public void setTargetPosition(Position position) {
        target = position.getPos();
    }

    // get current position
    public int getCurrentPosition() {
        return motor.getCurrentPosition();
    }

    // reset encoders
    public void resetEncoders() {
        motor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        // running two arm motors without encoder
        motor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        target = Position.HOME.getPos();
    }

    public void periodic() {
        isManMode = false;

        double power = CustomPIDControl(target, motor.getCurrentPosition());
        motor.setPower(power);
    }

    // show logs
    public void showLogs(Telemetry telemetry) {
        telemetry.addData("arm power", motor.getPower());
        telemetry.addData("arm pos", motor.getCurrentPosition());
        telemetry.addData("arm target", target);
    }
}
