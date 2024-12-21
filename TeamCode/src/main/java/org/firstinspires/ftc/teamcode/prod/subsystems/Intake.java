package org.firstinspires.ftc.teamcode.prod.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class Intake {

    // --- elbow ---
    private Servo lElbow, rElbow;
    // elbow pos
    public static double L_ELBOW_INIT_POS = 0, R_ELBOW_INIT_POS = 0.15;

    public static double L_ELBOW_BOX_POS = 0.4, R_ELBOW_BOX_POS = 0.35;
    public static double L_ELBOW_THROW_POS = 0.62, R_ELBOW_THROW_POS = 0.59;
    public static double L_ELBOW_TAKE_POS = 0.775 , R_ELBOW_TAKE_POS = 0.77;

    // --- wrist ---
    private Servo wrist;
    // wrist pos
    public static double WRIST_BOX_POS = 0;
    public static double WRIST_UP_POS = 0.5;
    public static double WRIST_TAKE_POS = 1;

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
        // --- elbow ---
        lElbow = hardwareMap.get(Servo.class, "l-elbow-intake");
        rElbow = hardwareMap.get(Servo.class, "r-elbow-intake");
        // direction
        lElbow.setDirection(Servo.Direction.FORWARD);
        rElbow.setDirection(Servo.Direction.REVERSE);
        // scale range
        lElbow.scaleRange(0, 1);
        rElbow.scaleRange(0, 1);
        // set init pos
        lElbow.setPosition(L_ELBOW_BOX_POS);
        rElbow.setPosition(R_ELBOW_BOX_POS);

        // --- wrist ---
        wrist = hardwareMap.get(Servo.class, "wrist-intake");
        wrist.setDirection(Servo.Direction.FORWARD);
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

    // elbow handles
    public void setElbowInitPos() {
        lElbow.setPosition(L_ELBOW_INIT_POS);
        rElbow.setPosition(R_ELBOW_INIT_POS);
    }
    public void setElbowBoxPos() {
        lElbow.setPosition(L_ELBOW_BOX_POS);
        rElbow.setPosition(R_ELBOW_BOX_POS);
    }
    public void setElbowTake() {
        lElbow.setPosition(L_ELBOW_TAKE_POS);
        rElbow.setPosition(R_ELBOW_TAKE_POS);
    }
    public void setElbowThrow() {
        lElbow.setPosition(L_ELBOW_THROW_POS);
        rElbow.setPosition(R_ELBOW_THROW_POS);
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
        telemetry.addData("left elbow intake", lElbow.getPosition());
        telemetry.addData("right elbow intake", rElbow.getPosition());
        telemetry.addData("wrist intake", wrist.getPosition());
        telemetry.addData("hand intake", hand.getPosition());
        telemetry.addData("claw intake", claw.getPosition());
    }
}