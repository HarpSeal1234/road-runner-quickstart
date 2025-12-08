package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.OrcaRoboticsConstants.kD;
import static org.firstinspires.ftc.teamcode.OrcaRoboticsConstants.kI;
import static org.firstinspires.ftc.teamcode.OrcaRoboticsConstants.kP;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;

public class colorSensor {
    private NormalizedColorSensor r_frontColorSensor;
    private NormalizedColorSensor l_frontColorSensor;
    private NormalizedColorSensor r_backColorSensor;
    private NormalizedColorSensor l_backColorSensor;
    public colorSensor(HardwareMap hardwareMap){
        r_frontColorSensor = hardwareMap.get(NormalizedColorSensor.class, "r_frontColorSensor");
        r_backColorSensor = hardwareMap.get(NormalizedColorSensor.class, "r_backColorSensor");
        l_frontColorSensor = hardwareMap.get(NormalizedColorSensor.class, "l_frontColorSensor");
        l_backColorSensor = hardwareMap.get(NormalizedColorSensor.class, "l_backColorSensor");
    }

}
