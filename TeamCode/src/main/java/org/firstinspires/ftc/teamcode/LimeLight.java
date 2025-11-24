package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.ArrayList;
import java.util.List;

public class LimeLight {

    public enum DetectedColor { PURPLE, GREEN, NONE }
    private Limelight3A limelight;
    public DetectedColor lastDetectedColor = DetectedColor.NONE;

    public ArrayList<DetectedColor> ballQueue = new ArrayList<>(3);

    private static final double INTAKE_Y_THRESHOLD = 10.0; //Must tune this for accuracy
    private boolean ballInView = false;

    public void init(HardwareMap hardwareMap) {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100);
        limelight.start();
    }

    public void update(Telemetry telemetry) {
        LLResult result = limelight.getLatestResult();
        lastDetectedColor = DetectedColor.NONE;

        if (result == null || !result.isValid()) {
            ballInView = false;
            return;
        }

        List<LLResultTypes.ColorResult> colorTargets = result.getColorResults();
        if (colorTargets == null || colorTargets.isEmpty()) {
            ballInView = false;
            return;
        }

        LLResultTypes.ColorResult best = null;
        for (LLResultTypes.ColorResult t : colorTargets) {
            if (t.getTargetYDegrees() > INTAKE_Y_THRESHOLD) {
                if (best == null || t.getTargetYDegrees() > best.getTargetYDegrees()) {
                    best = t;
                }
            }
        }

        // No longer in view -> reset
        if (best == null) {
            ballInView = false;
            return;
        }

        // Determine color from pipeline
        int pipeline = result.getPipelineIndex();

        if (pipeline == 0) lastDetectedColor = DetectedColor.PURPLE;
        else if (pipeline == 1) lastDetectedColor = DetectedColor.GREEN;

        // New ball entered
        if (!ballInView) {
            addBall(lastDetectedColor);
            ballInView = true;
        }

        telemetry.addData("Ball Count", ballQueue.size());
        for (int i = 0; i < ballQueue.size(); i++) {
            telemetry.addData("Position " + (i + 1), ballQueue.get(i));
        }
    }

    private void addBall(DetectedColor color) {
        if (ballQueue.size() == 3) {
            ballQueue.remove(0);
        }
        ballQueue.add(color);
    }
}
