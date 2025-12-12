package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

public class AprilTagDetector {
    private AprilTagProcessor aprilTagProcessor;
    private VisionPortal visionPortal;

    public int getDesiredTagId() {
        return desiredTagId;
    }

    private int desiredTagId = -1;
    public AprilTagDetector(HardwareMap hardwareMap){
        aprilTagProcessor = new AprilTagProcessor.Builder().build();
        VisionPortal.Builder builder = new VisionPortal.Builder();
        builder.setCamera(hardwareMap.get(WebcamName .class, "Webcam 1"));
        builder.addProcessor(aprilTagProcessor);

        // Build the Vision Portal, using the above settings.
        visionPortal = builder.build();
    }

    public int fetchAprilTag(){
        while (true) {
            List<AprilTagDetection> currentDetections = aprilTagProcessor.getDetections();
            if (!currentDetections.isEmpty()) {
                desiredTagId = currentDetections.get(0).id;
                visionPortal.stopStreaming();
                return desiredTagId;
            }
        }
    }

    public class DetectAprilTag implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            List<AprilTagDetection> currentDetections = aprilTagProcessor.getDetections();
            if(currentDetections.isEmpty()) return true;
            else {
                desiredTagId = currentDetections.get(0).id;
                visionPortal.stopStreaming();
                return false;
            }
        }
    }

    public Action detectAprilTag() {
        return new DetectAprilTag();
    }
}
