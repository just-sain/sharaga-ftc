package org.firstinspires.ftc.teamcode.test;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

//@Disabled
@Config
@TeleOp(name = "motor test", group = "test")
public class MotorTest extends OpMode {

    public DcMotor motor;

    @Override
    public void init() {
        motor = hardwareMap.get(DcMotor.class, "motor");
    }

    @Override
    public void loop() {
        if (gamepad1.left_trigger < 0.05) {
            motor.setPower(-gamepad1.left_trigger);
        } else {
            motor.setPower(-gamepad1.right_trigger);
        }

        // telemetry
        telemetry.addData("name", motor.getDeviceName());
        telemetry.addData("power", motor.getPower());
        telemetry.addData("position", motor.getCurrentPosition());
        telemetry.addData("controller", motor.getController());
        telemetry.addData("port number", motor.getPortNumber());
        telemetry.addData("info", motor.getConnectionInfo());
        telemetry.addData("motor type", motor.getMotorType());
    }
}
