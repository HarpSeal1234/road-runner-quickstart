package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.OrcaRoboticsConstants.COLOR_SENSOR_GREEN_RED_RATIO_THRESHOLD;

import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


public class BallManager {
    private enum DecodeBallColor {
        GREEN, PURPLE, NO_BALL
    }

    private NormalizedColorSensor r_frontColorSensor;
    private NormalizedColorSensor l_frontColorSensor;
    private NormalizedColorSensor r_backColorSensor;
    private NormalizedColorSensor l_backColorSensor;

    public BallManager(HardwareMap hardwareMap) {
        r_frontColorSensor = hardwareMap.get(NormalizedColorSensor.class, "r_frontColorSensor");
        r_backColorSensor = hardwareMap.get(NormalizedColorSensor.class, "r_backColorSensor");
        l_frontColorSensor = hardwareMap.get(NormalizedColorSensor.class, "l_frontColorSensor");
        l_backColorSensor = hardwareMap.get(NormalizedColorSensor.class, "l_backColorSensor");
    }

    private boolean ballExists(NormalizedColorSensor colorSensor) {
        double distance = ((DistanceSensor) colorSensor).getDistance(DistanceUnit.MM);
        return !(distance > 2.0);
    }

    public boolean rightFrontBallExists() {
        return ballExists((r_frontColorSensor));
    }

    public boolean rightBackBallExists() {
        return ballExists((r_backColorSensor));
    }

    public boolean leftFrontBallExists() {
        return ballExists((l_frontColorSensor));
    }

    public boolean leftBackBallExists() {
        return ballExists((l_backColorSensor));
    }

    public DecodeBallColor detectColor(NormalizedColorSensor colorSensor) {
        if (ballExists(colorSensor)) {
            float red = colorSensor.getNormalizedColors().red;
            float green = colorSensor.getNormalizedColors().green;
            if (green / red > COLOR_SENSOR_GREEN_RED_RATIO_THRESHOLD) {
                return DecodeBallColor.GREEN;
            } else {
                return DecodeBallColor.PURPLE;
            }
        }
        return DecodeBallColor.NO_BALL;
    }

    public int getNumOfBalls() {
        int numOfBalls = 0;
        if (leftFrontBallExists()) numOfBalls++;
        if (leftBackBallExists()) numOfBalls++;
        if (rightFrontBallExists()) numOfBalls++;
        if (rightBackBallExists()) numOfBalls++;
        return numOfBalls;
    }

    public DecodeBallColor detectRightFrontColor() {
        return detectColor(r_frontColorSensor);
    }

    public DecodeBallColor detectRightBackColor() {
        return detectColor(r_backColorSensor);
    }

    public DecodeBallColor detectLeftFrontColor() {
        return detectColor(l_frontColorSensor);
    }

    public DecodeBallColor detectLeftBackColor() {
        return detectColor(l_backColorSensor);
    }
}