package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

@TeleOp(name="color", group="Linear OpMode")
@Disabled
public class colorCheck extends LinearOpMode {
    private NormalizedColorSensor r_frontColorSensor;
    private NormalizedColorSensor l_frontColorSensor;
    private NormalizedColorSensor r_backColorSensor;
    private NormalizedColorSensor l_backColorSensor;
    public void runOpMode() {
        r_frontColorSensor = hardwareMap.get(NormalizedColorSensor.class, "r_frontColorSensor");
        r_backColorSensor = hardwareMap.get(NormalizedColorSensor.class, "r_backColorSensor");
        l_frontColorSensor = hardwareMap.get(NormalizedColorSensor.class, "l_frontColorSensor");
        l_backColorSensor = hardwareMap.get(NormalizedColorSensor.class, "l_backColorSensor");

        while (opModeIsActive()) {
            NormalizedRGBA r_frontColor = r_frontColorSensor.getNormalizedColors();
            r_frontColor.toColor();
            telemetry.addData("rgb", r_frontColor);
            telemetry.addData("rgb", r_frontColor);
            telemetry.addData("rgb", r_frontColor);
            telemetry.addData("rgb", r_frontColor);
            telemetry.update();

//        checkGreen(r_frontColor.red);
//        checkRed(r_frontColor.green);
//        checkBlue(r_frontColor.blue);
        }
    }
    public void checkRed(double color){
        if(color > 0.9) {
            telemetry.addData("color","red");
        }
    }
    public void checkGreen(double color){
        if(color > 0.9) {
            telemetry.addData("color","green");
        }
    }
    public void checkBlue(double color){
        if(color > 0.9) {
            telemetry.addData("color","blue");
        }
    }
}
