package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.OrcaRoboticsConstants.CLOSE_PIVOT_POSITION;
import static org.firstinspires.ftc.teamcode.OrcaRoboticsConstants.FAR_PIVOT_POSITION;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

// Non-RR imports


public class Pivot {
    private Servo pivot;
//    public final static double FAR_PIVOT_POSITION = 0.65;
//    public final static double CLOSE_PIVOT_POSITION = 0.45;

    public Pivot(HardwareMap hardwareMap){
        pivot = hardwareMap.get(Servo.class, "pivot");
        pivot.setPosition(FAR_PIVOT_POSITION);
    }

    public class FarPivot implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            pivot.setPosition(FAR_PIVOT_POSITION);
            return false;
        }

    }
    public Action farPivot(){
        return new FarPivot();
    }

    public class ClosePivot implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            pivot.setPosition(CLOSE_PIVOT_POSITION);
            return false;
        }

    }
    public Action closePivot(){
        return new ClosePivot();
    }
}
