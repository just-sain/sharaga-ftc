package org.firstinspires.ftc.teamcode.prod.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class Outtake {
    // claw
    private Servo lClaw, rClaw;
    // claw open
    public static double L_CLAW_OPEN = 0.525, R_CLAW_OPEN = 0.725;
    // claw close
    public static double L_CLAW_CLOSE = 0.34, R_CLAW_CLOSE = 0.58;

    public void initialize(HardwareMap hardwareMap) {
        lClaw = hardwareMap.get(Servo.class, "l-claw-outtake");
        rClaw = hardwareMap.get(Servo.class, "r-claw-outtake");
        // direction
        lClaw.setDirection(Servo.Direction.FORWARD);
        rClaw.setDirection(Servo.Direction.REVERSE);
        // scale range
        lClaw.scaleRange(0, 1);
        rClaw.scaleRange(0, 1);
        // setting pos
        lClaw.setPosition(L_CLAW_CLOSE);
        rClaw.setPosition(R_CLAW_CLOSE);
    }

    // claw handles
    public void setClawClose() {
        lClaw.setPosition(L_CLAW_CLOSE);
        rClaw.setPosition(R_CLAW_CLOSE);
    }
    public void setClawOpen() {
        lClaw.setPosition(L_CLAW_OPEN);
        rClaw.setPosition(R_CLAW_OPEN);
    }

    public void showLogs(Telemetry telemetry) {
        telemetry.addData("l claw:", lClaw.getPosition());
        telemetry.addData("r claw:", rClaw.getPosition());
    }
}
