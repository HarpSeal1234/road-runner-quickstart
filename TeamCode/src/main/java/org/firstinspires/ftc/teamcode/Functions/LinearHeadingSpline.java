package org.firstinspires.ftc.teamcode.Functions;// RR-specific imports

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.AprilTagDetector;
import org.firstinspires.ftc.teamcode.Avocado;
import org.firstinspires.ftc.teamcode.Intake;
import org.firstinspires.ftc.teamcode.Launcher;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.Pivot;

@Config
@Disabled
@Autonomous(name = "splineLINEARHeading TAN0", group = "Autonomous")

public class LinearHeadingSpline extends LinearOpMode{
    private static final boolean USE_WEBCAM = true;
    public final static int FAR_OUTTAKE_VELOCITY = 1700;
    public final static int CLOSE_OUTTAKE_VELOCITY = 1400;
    public final static int MEDIUM_OUTTAKE_VELOCITY = 1200;

    public void runOpMode() {
        // instantiate your MecanumDrive at a particular pose.
        Pose2d initialPose = new Pose2d(0, 0, 0);
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
        AprilTagDetector aprilTagDetector = new AprilTagDetector(hardwareMap);
        Intake intake1 = new Intake(hardwareMap);
        Launcher outtake1 = new Launcher(hardwareMap);
        Avocado blockerR = new Avocado(hardwareMap);
        Avocado blockerL = new Avocado(hardwareMap);
        Pivot pivot = new Pivot(hardwareMap);


        int visionOutputPosition = 1;

        TrajectoryActionBuilder path1 = drive.actionBuilder(initialPose)
                .setTangent(0)
                .splineToLinearHeading(new Pose2d(48, 48, Math.PI / 2), Math.PI / 2);
        waitForStart();

        if (isStopRequested()) return;
        Action trajectoryActionChosen;
        trajectoryActionChosen = path1.build();
        Actions.runBlocking(new ParallelAction(trajectoryActionChosen)); // RIGHT
        telemetry.addData("position", drive.localizer.getPose());
        telemetry.update();
        sleep(30000);

    }

    }




