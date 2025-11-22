package org.firstinspires.ftc.teamcode.HardwareDrivers;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.Pose3D;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.List;
import java.util.Locale;

/**
 * A driver class for managing a Limelight camera.
 * This class handles initialization and processing of target data.
 */
public class LimeLight {
    private Limelight3A limelight;

    /**
     * Initializes the Limelight camera.
     * @param hardwareMap The robot's hardware map.
     */
    public void init(HardwareMap hardwareMap) {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        // Set a reasonable poll rate. 100Hz is very high and can flood the bus; 30Hz is often sufficient.
        limelight.setPollRateHz(30);
        limelight.start(); // This tells Limelight to start looking!
    }

    /**
     * Processes the latest result from the Limelight and updates telemetry.
     * @param telemetry The Telemetry object to display data.
     */
    public void update(Telemetry telemetry) {
        LLResult result = limelight.getLatestResult();

        // Use a guard clause to exit early if the result is invalid.
        if (result == null || !result.isValid()) {
            telemetry.addData("Limelight", "No Targets");
            return; // Exit the method since there's nothing to process.
        }

        // --- At this point, we know 'result' is valid ---

        // Add primary target data (tx, ty, ta)
        telemetry.addData("Target X", result.getTx());
        telemetry.addData("Target Y", result.getTy());
        telemetry.addData("Target Area", result.getTa());

        // Add robot pose data if available
        Pose3D botpose = result.getBotpose();
        // Check if botpose AND its position are not null before using them.
        if (botpose != null && botpose.getPosition() != null) {
            double x = botpose.getPosition().x;
            double y = botpose.getPosition().y;
            // Simplify the telemetry call by combining formatting and adding data.
            telemetry.addData("Robot Pose (X, Y)", "(%.2f, %.2f)", x, y);
        } else {
            // Add feedback for when the pose is null, so you know why it's not displaying.
            telemetry.addData("Robot Pose (X, Y)", "Not Available");
        }

        // Process and display color target data
        List<LLResultTypes.ColorResult> colorTargets = result.getColorResults();
        if (colorTargets != null && !colorTargets.isEmpty()) {
            // Loop through each detected color target
            for (int i = 0; i < colorTargets.size(); i++) {
                LLResultTypes.ColorResult target = colorTargets.get(i);
                double area = target.getTargetArea();
                // Use a formatted string and a unique key for each target
                telemetry.addData(String.format(Locale.US, "Color Target %d", i + 1),
                        "Area: %.2f%%, X: %.2f, Y: %.2f",
                        area, target.getTargetXDegrees(), target.getTargetYDegrees());
            }
        }
    }
}
