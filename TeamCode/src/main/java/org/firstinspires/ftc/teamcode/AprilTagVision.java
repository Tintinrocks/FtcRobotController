package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
import java.util.Optional;

/**
 * A driver class for managing AprilTag detection using a webcam.
 */
public class AprilTagVision {
    private AprilTagProcessor aprilTagProcessor;
    private VisionPortal visionPortal;

    /**
     * Initializes the AprilTag processor and the vision portal.
     * Must be called during the OpMode's init phase.
     * @param hardwareMap The robot's hardware map.
     */
    public void init(HardwareMap hardwareMap) {
        aprilTagProcessor = new AprilTagProcessor.Builder()
                // Add any custom settings here, e.g., lens intrinsics
                .build();

        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(aprilTagProcessor)
                .build();
    }

    /**
     * Finds a specific AprilTag by its ID.
     * @param targetId The ID of the tag to search for.
     * @return An Optional containing the AprilTagDetection if found, otherwise an empty Optional.
     */
    public Optional<AprilTagDetection> getDetectionById(int targetId) {
        List<AprilTagDetection> detections = aprilTagProcessor.getDetections();

        if (detections == null) {
            return Optional.empty(); // Return an empty optional if list is null
        }

        // Use a stream to find the first detection matching the target ID.
        return detections.stream()
                .filter(detection -> detection.id == targetId)
                .findFirst();
    }

    /**
     * Gets the first (usually closest) AprilTag detection.
     * @return An Optional containing the AprilTagDetection if any tag is found, otherwise an empty Optional.
     */
    public Optional<AprilTagDetection> getClosestDetection() {
        List<AprilTagDetection> detections = aprilTagProcessor.getDetections();

        if (detections == null || detections.isEmpty()) {
            return Optional.empty();
        }

        // The SDK sorts detections by proximity, so the first one is the closest.
        return Optional.of(detections.get(0));
    }

    /**
     * Stops the vision portal stream. Call this at the end of an OpMode.
     */
    public void close() {
        if (visionPortal != null) {
            visionPortal.close();
        }
    }
}

