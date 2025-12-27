package org.firstinspires.ftc.teamcode.TestFiles;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;


//1620rpm - 2700 max velocity
//6000rpm - 2800 max velocity
//@Disabled
@TeleOp(name="Max Velocity Test")
public class MaxVelocityTest extends LinearOpMode {
//Motor Variables
    private DcMotorEx outtake1;
    private double motorOneCurrentVelocity = 0.0;
    private double motorOneMaxVelocity = 0.0;

//PID Variables


    @Override
    public void runOpMode() throws InterruptedException {
        initHardware();
        while (!isStarted()) {
            motorTelemetry();
        }
        waitForStart();
        while(opModeIsActive()) {
            runMotorOne();
            motorTelemetry();
        }
    }

    public void initHardware() {
        initMotorOne();
    }
//initialize motorOne
    public void initMotorOne() {
        outtake1 = hardwareMap.get(DcMotorEx.class, "outtake1");
        outtake1.setDirection(DcMotor.Direction.FORWARD);
        outtake1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        outtake1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void runMotorOne() {
        outtake1.setPower(1.0);
        motorOneCurrentVelocity = outtake1.getVelocity();
        if (motorOneCurrentVelocity > motorOneMaxVelocity) {
            motorOneMaxVelocity = motorOneCurrentVelocity;
        }
    }

    public void motorTelemetry() {
        telemetry.log().clear();
        telemetry.addData("Power", outtake1.getPower());
        telemetry.addData("Max Velocity", motorOneMaxVelocity);
        telemetry.addData("Current Velocity", motorOneCurrentVelocity);
        telemetry.update();
    }
}
