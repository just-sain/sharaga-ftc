package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.prod.subsystems.Base;

//@Disabled
@TeleOp(name = "base test", group = "test")
public class BaseTest extends OpMode {

    private Base base = new Base();

    @Override
    public void init() {
        base.initialize(hardwareMap, DcMotor.ZeroPowerBehavior.BRAKE, DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        telemetry.addLine("ready, press start");
    }

    @Override
    public void loop() {
        if (gamepad1.a) {
            base.setCustomMotorsPower(0.3, 0, 0, 0);
        } else if (gamepad1.b) {
            base.setCustomMotorsPower(0, 0.3, 0, 0);
        } else if (gamepad1.y) {
            base.setCustomMotorsPower(0, 0, 0.3, 0);
        } else if (gamepad1.x) {
            base.setCustomMotorsPower(0, 0, 0, 0.3);
        } else {
            base.setMotorsPower(0);
        }

        telemetry.addLine("a -> lf; b -> rf;");
        telemetry.addLine("y -> lr; x -> rr;");
        base.showTelemetry(telemetry);
        telemetry.update();
    }
}
