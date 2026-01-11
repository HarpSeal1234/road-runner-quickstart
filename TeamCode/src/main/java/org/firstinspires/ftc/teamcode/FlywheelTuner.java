package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

@TeleOp
@Disabled
public class FlywheelTuner extends OpMode {
    public DcMotorEx outtake1;
    public double highVel = 1500;
    public double lowVel = 1400;
    double targetVel = highVel;
    double F = 0;
    double P = 0;
    double[] stepSizes = {10.0,1.0,0.1,0.01,0.001,0.0001};
    int stepIndex = 1;
    @Override
    public void init(){
        outtake1 = hardwareMap.get(DcMotorEx.class, "outtake1");
        outtake1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        outtake1.setDirection(DcMotorEx.Direction.REVERSE);
        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(P,0,0,F);
        outtake1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,pidfCoefficients);
        telemetry.addLine("Init Complete");
    }

    @Override
    public void loop(){
        // get gamepad commands
        if (gamepad1.yWasPressed()){
            if (targetVel == highVel){
                targetVel = lowVel;
            } else {
                targetVel = highVel;
            }
        }

        if (gamepad1.bWasPressed()){
            stepIndex = (stepIndex + 1) % stepSizes.length;
        }

        if (gamepad1.dpadLeftWasPressed()){
            F -= stepSizes[stepIndex];
        }

        if (gamepad1.dpadRightWasPressed()){
            F += stepSizes[stepIndex];
        }

        if (gamepad1.dpadUpWasPressed()){
            P += stepSizes[stepIndex];
        }

        if (gamepad1.dpad_down){
            P -= stepSizes[stepIndex];
        }

        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(P,0,0,F);
        outtake1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,pidfCoefficients);

        outtake1.setVelocity(targetVel);
        double currentVelocity = outtake1.getVelocity();
        double error = targetVel - currentVelocity;

        telemetry.addData("Target Velocity",targetVel);
        telemetry.addData("Current Velocity", currentVelocity);
        telemetry.addData("Error", "%.2f",error);
        telemetry.addLine("--------");
        telemetry.addData("P","%.4f (D-Pad U/D)",P);
        telemetry.addData("F","%.4f (D-Pad L/R)",F);
        telemetry.addData("Step Size","%.4f (B Button)",stepSizes[stepIndex]);
        telemetry.update();
    }
}