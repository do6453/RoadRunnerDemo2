package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.util.DashboardUtil;

import org.firstinspires.ftc.teamcode.util.SplineConstants;

@Config
@Autonomous(group = "drive")
public class SplineTestInSeries extends LinearOpMode {
    public static PIDCoefficients TRANSLATIONAL_PID = new PIDCoefficients(8, 0, 0);

    public static SplineConstants WAY_POINT1 = new SplineConstants( 30, 30, 45,1000 );
    public static SplineConstants WAY_POINT2 = new SplineConstants( 80, -10, 270,1000 );
    public static SplineConstants WAY_POINT3 = new SplineConstants( 0, -15, 180,1000 );
    public static boolean reverseSpline = true;

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Trajectory traj = drive.trajectoryBuilder(new Pose2d())
                .splineTo(new Vector2d(WAY_POINT1.x, WAY_POINT1.y), WAY_POINT1.heading)
                .splineTo(new Vector2d(WAY_POINT2.x, WAY_POINT2.y), Math.toRadians(WAY_POINT2.heading))
                .splineTo(new Vector2d(WAY_POINT3.x, WAY_POINT3.y), Math.toRadians(WAY_POINT3.heading))
                .build();

        DashboardUtil.previewTrajectories(FtcDashboard.getInstance(), traj);

        waitForStart();

        if (isStopRequested()) return;

        drive.followTrajectory(traj);
     }
}
