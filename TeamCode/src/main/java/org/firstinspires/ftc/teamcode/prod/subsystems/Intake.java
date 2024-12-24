package org.firstinspires.ftc.teamcode.prod.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class Intake {

    // --- wrist ---
    private Servo wrist;
    // wrist pos
    public static double WRIST_BOX_POS = 0;
    public static double WRIST_UP_POS = 0.475;
    public static double WRIST_TAKE_POS = 0.95;

    // --- hand ---
    private Servo hand;
    // hand pos
    private static final double HAND_MIN_POS = 0.2;
    private static final double HAND_NEUTRAL_POS = 0.5411;
    private static final double HAND_MAX_POS = 0.85;

    // --- claw ---
    private Servo claw;
    // claw pos
    private static final double CLAW_OPEN_POS = 0.15;
    private static final double CLAW_CLOSE_POS = 0.4;

    // --- initialize ---
    public void initialize(HardwareMap hardwareMap) {
        // --- wrist ---
        wrist = hardwareMap.get(Servo.class, "wrist-intake");
        wrist.setDirection(Servo.Direction.REVERSE);
        wrist.scaleRange(0, 1);
        wrist.setPosition(WRIST_UP_POS);

        // --- hand ---
        hand = hardwareMap.get(Servo.class, "hand-intake");
        hand.setDirection(Servo.Direction.FORWARD);
        hand.scaleRange(0, 1);
        hand.setPosition(HAND_NEUTRAL_POS);

        // --- claw ---
        claw = hardwareMap.get(Servo.class, "claw-intake");
        claw.scaleRange(0, 1);
        claw.setDirection(Servo.Direction.FORWARD);
        claw.setPosition(CLAW_CLOSE_POS);
    }

    // wrist handles
    public void setWristTake() {
        wrist.setPosition(WRIST_TAKE_POS);
    }
    public void setWristUp() {
        wrist.setPosition(WRIST_UP_POS);
    }
    public void setWristBox() {
        wrist.setPosition(WRIST_BOX_POS);
    }

    // hand handles
    public void setHandNeutral() {
        hand.setPosition(HAND_NEUTRAL_POS);
    }
    public void setHandCustom(double stickInput) {
        double pos = (-stickInput + 1) / 2;

        if (pos <= HAND_MIN_POS) pos = HAND_MIN_POS;
        if (pos >= HAND_MAX_POS) pos = HAND_MAX_POS;

        hand.setPosition(pos);
    }

    public void setHandMaxPos() {
        hand.setPosition(HAND_MAX_POS);
    }

    // claw handles
    public void setClawOpen() {
        claw.setPosition(CLAW_OPEN_POS);
    }
    public void setClawClose() {
        claw.setPosition(CLAW_CLOSE_POS);
    }

    // show logs
    public void showLogs(Telemetry telemetry) {
        telemetry.addData("wrist intake", wrist.getPosition());
        telemetry.addData("hand intake", hand.getPosition());
        telemetry.addData("claw intake", claw.getPosition());
    }
}