package drive.writtenCode.auto;


import com.acmerobotics.dashboard.config.Config;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Constants;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.util.ElapsedTime;

import drive.writtenCode.RobotMap;
import drive.writtenCode.controllers.ClawController;
import drive.writtenCode.controllers.ClawPositionController;
import drive.writtenCode.controllers.ClawRotateController;
import drive.writtenCode.controllers.FourbarController;
import drive.writtenCode.controllers.LinkageController;
import drive.writtenCode.controllers.LinkageSlidesController;
import drive.writtenCode.controllers.ScoreSystemController;
import drive.writtenCode.controllers.SlidesController;
import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;


@Config
@Autonomous(group = "Auto")

public class AutoBsk extends LinearOpMode {


    private void wait(int ms) {
        try{
            Thread.sleep(ms);
        }
        catch (InterruptedException ignored) {
            //nu face nimic
        }
    }


    enum STROBOT
    {
        START,
        PLACE_PRELOAD,
        PICKUP1, GRAB_PICKUP1, PICKUP2, GRAB_PICKUP2, PICKUP3, GRAB_PICKUP3, END_AUTO,
        SCORE_PICKUP1, SCORE_PICKUP2, SCORE_PICKUP3, SCORE_INTER1, SCORE_INTER2, SCORE_INTER3, INTER_PRELOAD,PARK;
    }
    ElapsedTime AutoTimer = new ElapsedTime();
    ElapsedTime TimerLeave = new ElapsedTime();
    ElapsedTime TimerPlacePreload = new ElapsedTime();
    ElapsedTime TimerPickup = new ElapsedTime();
    ElapsedTime TimerScore = new ElapsedTime();
    private double time_place_preload = 1.3;
    private double time_leave_preload = 0.5;
    private Follower follower;

    int linkage_target_position = 0;
    int claw_rotate_target = 0;
    int slides_target_position = 0;

    private Path scorePreload,scorePreloadInter ,pickup1,scorePickup1Inter,scorePickup1,pickup2,scorePickup2Inter,scorePickup2,pickup3,scorePickup3Inter,scorePickup3,park;
    private PathChain scorePickup1Chain;
    public static double tunex=0;
    public static double tuney=0;
    final Pose startPose = new Pose(10, 111, Math.toRadians(0));
    final Pose preloadPose = new Pose(8, 121.5, Math.toRadians(312));
    final Pose interPreload = new Pose(9.5, 120, Math.toRadians(312));
    final Pose interPose = new Pose(16,122,Math.toRadians(315));
    final Pose inter2Pose = new Pose(16,122.7,Math.toRadians(315));
    final Pose inter3Pose = new Pose(27.2,121.7,Math.toRadians(315));
    final Pose scorePose = new Pose(9, 125, Math.toRadians(315));
    final Pose scorePose2 = new Pose(9,125, Math.toRadians(315));
    final Pose scorePose3 = new Pose(18.5,126, Math.toRadians(315));
    final Pose pickup1Pose = new Pose(30, 120.5, Math.toRadians(0));
    final Pose pickup2Pose = new Pose(31.2,128,Math.toRadians(0)); //41 19
    final Pose pickup3Pose = new Pose(35.5,129,Math.toRadians(50));

    final Pose parkPose =new Pose(35,115, Math.toRadians(0));


    @Override
    public void runOpMode() throws InterruptedException {
        RobotMap robot = new RobotMap(hardwareMap);
        DigitalChannel beam = robot.beam;
        ClawController claw = new ClawController(robot);
        ClawPositionController clawPosition = new ClawPositionController(robot);
        ClawRotateController clawRotate = new ClawRotateController(robot);
        FourbarController fourbar = new FourbarController(robot);
        LinkageController linkage = new LinkageController(robot);
        SlidesController slides = new SlidesController(robot);
        ScoreSystemController scoreSystem = new ScoreSystemController(claw,clawRotate,fourbar,clawPosition);
        LinkageSlidesController linkageSlides = new LinkageSlidesController(linkage,slides);

        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPose);
        scoreSystem.currentStatus = ScoreSystemController.ScoreSystemStatus.INIT_AUTO;

        scoreSystem.update(robot.encoderClaw.getVoltage());
        linkageSlides.update();
        claw.update();
        clawPosition.update();
        clawRotate.update(ClawRotateController.init_position,0);
        fourbar.update();
        follower.update();
        linkage.update(LinkageController.init_position,linkage_target_position);
        slides.update(SlidesController.init_position,slides_target_position);

        scorePreload = new Path(new BezierLine(new Point(interPreload), new Point(preloadPose)));
        scorePreload.setConstantHeadingInterpolation(preloadPose.getHeading());

        scorePreloadInter = new Path(new BezierLine(new Point(startPose), new Point(interPreload)));
        scorePreloadInter.setLinearHeadingInterpolation(startPose.getHeading(),interPreload.getHeading());
        park = new Path(new BezierLine(new Point(preloadPose), new Point(parkPose)));
        park.setLinearHeadingInterpolation(preloadPose.getHeading(),parkPose.getHeading());

        pickup1 = new Path(new BezierLine(new Point(preloadPose), new Point(pickup1Pose)));
        pickup1.setLinearHeadingInterpolation(preloadPose.getHeading(), pickup1Pose.getHeading());

        scorePickup1Inter = new Path(new BezierLine(new Point(pickup1Pose), new Point(interPose)));
        scorePickup1Inter.setLinearHeadingInterpolation(pickup1Pose.getHeading(), interPose.getHeading());

        scorePickup1 = new Path(new BezierLine(new Point(interPose), new Point(scorePose)));
        scorePickup1.setLinearHeadingInterpolation(interPose.getHeading(), scorePose.getHeading());

//        scorePickup1Chain = new PathChain(
//                scorePickup1Inter,
//                scorePickup1
//        );
        scorePickup1Chain = follower.pathBuilder()
                .addPath(scorePickup1Inter)
                .setLinearHeadingInterpolation(pickup1Pose.getHeading(), interPose.getHeading())
                .addPath(scorePickup1)
                .setLinearHeadingInterpolation(interPose.getHeading(), scorePose.getHeading())
                .build();

        pickup2 = new Path(new BezierLine(new Point(scorePose), new Point(pickup2Pose)));
        pickup2.setLinearHeadingInterpolation(scorePose.getHeading(), pickup2Pose.getHeading());

        scorePickup2Inter = new Path(new BezierLine(new Point(pickup2Pose), new Point(inter2Pose)));
        scorePickup2Inter.setLinearHeadingInterpolation(pickup2Pose.getHeading(),inter2Pose.getHeading());

        scorePickup2 = new Path(new BezierLine(new Point(inter2Pose), new Point(scorePose2)));
        scorePickup2.setLinearHeadingInterpolation(inter2Pose.getHeading(), scorePose2.getHeading());

        pickup3 = new Path(new BezierLine(new Point(scorePose2), new Point(pickup3Pose)));
        pickup3.setLinearHeadingInterpolation(scorePose2.getHeading(), pickup3Pose.getHeading());

        scorePickup3Inter = new Path(new BezierLine(new Point(pickup3Pose), new Point(inter3Pose)));
        scorePickup3Inter.setLinearHeadingInterpolation(pickup3Pose.getHeading(),inter3Pose.getHeading());

        scorePickup3 = new Path(new BezierLine(new Point(inter3Pose), new Point(scorePose3)));
        scorePickup3.setLinearHeadingInterpolation(inter3Pose.getHeading(), scorePose3.getHeading());


        boolean ok=false;

        STROBOT status = STROBOT.START;
        waitForStart();
        if (isStopRequested()) return;
        while (opModeIsActive() && !isStopRequested())
        {
            int slides_current_position = robot.linkage.getCurrentPosition();
            int linkage_current_position = linkage.encoderLinkage.getCurrentPosition();
            switch (status) {
                case START: {
                    AutoTimer.reset();
                    TimerPlacePreload.reset();
                    follower.followPath(scorePreloadInter,false);
                    status = STROBOT.INTER_PRELOAD;
                    break;
                }
                case INTER_PRELOAD:
                {
                    if(!follower.isBusy())
                    {
                        follower.followPath(scorePreload);
                        status=STROBOT.PLACE_PRELOAD;
                    }
                    break;
                }
                case PLACE_PRELOAD: {
                    if(TimerPlacePreload.seconds()>1.5)
                    {
                        linkageSlides.currentStatus= LinkageSlidesController.LinkageSlidesStatus.BSK_HIGH;
                    }
                    if(TimerPlacePreload.seconds() > 2.3)
                    {
                        scoreSystem.currentStatus = ScoreSystemController.ScoreSystemStatus.SCORE;
                    }
                    if(TimerPlacePreload.seconds() > 2.5)
                    {
                        claw.currentStatus= ClawController.ClawStatus.OPEN;
                    }
                    if(TimerPlacePreload.seconds()>2.7)
                    {
                        scoreSystem.currentStatus= ScoreSystemController.ScoreSystemStatus.COLLECT_FROM_SUB;
                        linkageSlides.currentStatus = LinkageSlidesController.LinkageSlidesStatus.INIT;
                    }
                    if(TimerPlacePreload.seconds()>3.15)
                    {
                        slides.currentStatus= SlidesController.SlidesStatus.INIT;
                    }
                    if(TimerPlacePreload.seconds() > 3.5)
                    {
                        status=STROBOT.PARK;
                        follower.followPath(park);
                    }
                    break;
                }
                case GRAB_PICKUP1:
                {
                    if(TimerPlacePreload.seconds()>3.72)
                    {
                        linkage.currentStatus = LinkageController.LinkageStatus.COLLECT;
                    }
                    if(follower.isBusy())
                    {
                        TimerPickup.reset();
                    }
                    if(!follower.isBusy())
                    {
                        fourbar.currentStatus = FourbarController.FourbarStatus.COLLECT_SUB;
                        clawPosition.currentStatus = ClawPositionController.ClawPositionStatus.COLLECT_SUB;
                        if (TimerPickup.seconds() > 0.25) {
                            claw.currentStatus = ClawController.ClawStatus.CLOSE;
                        }
                        if(TimerPickup.seconds() > 0.50)
                        {
                            linkageSlides.currentStatus= LinkageSlidesController.LinkageSlidesStatus.INIT;
                            linkage.currentStatus= LinkageController.LinkageStatus.INIT;
                            scoreSystem.currentStatus= ScoreSystemController.ScoreSystemStatus.INIT_AUTO;
                            TimerScore.reset();
                            follower.followPath(scorePickup1Inter);
                            status=STROBOT.SCORE_INTER1;
                        }
                    }
                    break;
                }
                case SCORE_INTER1:
                {
                    if(follower.isBusy())
                    {
                        TimerScore.reset();
                    }
                    if(!follower.isBusy())
                    {
                        if(TimerScore.seconds()>0.2)
                        {
                            follower.followPath(scorePickup1);
                            TimerScore.reset();
                            status=STROBOT.SCORE_PICKUP1;
                        }
                        linkageSlides.currentStatus= LinkageSlidesController.LinkageSlidesStatus.BSK_HIGH;
                    }
                    break;
                }
                case SCORE_PICKUP1:
                {
                    if(TimerScore.seconds()>1)
                    {
                        scoreSystem.currentStatus = ScoreSystemController.ScoreSystemStatus.SCORE;
                    }
                    if(!follower.isBusy() && TimerScore.seconds()>1.3)
                    {
//                        scoreSystem.currentStatus= ScoreSystemController.ScoreSystemStatus.FLICK;
                        claw.currentStatus= ClawController.ClawStatus.OPEN;
                        if(TimerScore.seconds()>1.5)
                        {
                            scoreSystem.currentStatus= ScoreSystemController.ScoreSystemStatus.COLLECT_FROM_SUB;
                        }
                        if(TimerScore.seconds()>1.7)
                        {
                            linkageSlides.currentStatus = LinkageSlidesController.LinkageSlidesStatus.INIT;
                        }
                        if(TimerScore.seconds()>2)
                        {
                            TimerLeave.reset();
                            follower.followPath(pickup2);
                            status = STROBOT.GRAB_PICKUP2;
                        }
                    }
                    break;
                }
                case GRAB_PICKUP2:
                {
                    if(TimerLeave.seconds()>0.22)
                    {
                        slides.currentStatus= SlidesController.SlidesStatus.INIT;
                        linkage.currentStatus = LinkageController.LinkageStatus.COLLECT;

                    }
                    if(follower.isBusy())
                    {
                        TimerPickup.reset();
                    }
                    if(!follower.isBusy() && TimerLeave.seconds()>0.25)
                    {
                        fourbar.currentStatus = FourbarController.FourbarStatus.COLLECT_SUB;
                        clawPosition.currentStatus = ClawPositionController.ClawPositionStatus.COLLECT_SUB;
                        if (TimerPickup.seconds() > 0.25) {
                            claw.currentStatus = ClawController.ClawStatus.CLOSE;
                        }
                        if(TimerPickup.seconds() > 0.50)
                        {
                            linkageSlides.currentStatus= LinkageSlidesController.LinkageSlidesStatus.INIT;
                            linkage.currentStatus= LinkageController.LinkageStatus.INIT;
                            scoreSystem.currentStatus= ScoreSystemController.ScoreSystemStatus.INIT_AUTO;
                            TimerScore.reset();
                            follower.followPath(scorePickup2Inter);
                            status=STROBOT.SCORE_INTER2;
                        }
                    }
                    break;
                }
                case SCORE_INTER2:
                {
                    if(follower.isBusy())
                    {
                        TimerScore.reset();
                    }
                    if(!follower.isBusy())
                    {
                        if(TimerScore.seconds()>0.2)
                        {
                            follower.followPath(scorePickup2);
                            TimerScore.reset();
                            status=STROBOT.SCORE_PICKUP2;
                        }
                        linkageSlides.currentStatus= LinkageSlidesController.LinkageSlidesStatus.BSK_HIGH;
                    }
                    break;
                }
                case SCORE_PICKUP2:
                {
                    if(TimerScore.seconds()>1)
                    {
                        scoreSystem.currentStatus = ScoreSystemController.ScoreSystemStatus.SCORE;
                    }
                    if(!follower.isBusy() && TimerScore.seconds()>1.3)
                    {
//                        scoreSystem.currentStatus= ScoreSystemController.ScoreSystemStatus.FLICK;
                        claw.currentStatus= ClawController.ClawStatus.OPEN;
                        if(TimerScore.seconds()>2)
                        {
//                            follower.followPath(pickup3);
                            TimerLeave.reset();
//                            status = STROBOT.GRAB_PICKUP3;
                        }
                    }
                    break;
                }
                case GRAB_PICKUP3:
                {
                    if(TimerLeave.seconds()>0.1)
                    {
                        linkageSlides.currentStatus = LinkageSlidesController.LinkageSlidesStatus.INIT;
                    }
                    if(TimerLeave.seconds()>0.3)
                    {
                        slides.currentStatus= SlidesController.SlidesStatus.INIT;
                        linkage.currentStatus = LinkageController.LinkageStatus.COLLECT;
                        scoreSystem.currentStatus= ScoreSystemController.ScoreSystemStatus.COLLECT_FROM_SUB;
                        clawRotate.currentStatus= ClawRotateController.ClawRotateStatus.MINUS;
                    }
                    if(follower.isBusy())
                    {
                        TimerPickup.reset();
                    }
                    if(!follower.isBusy())
                    {
                        fourbar.currentStatus = FourbarController.FourbarStatus.COLLECT_SUB;
                        clawPosition.currentStatus = ClawPositionController.ClawPositionStatus.COLLECT_SUB;
                        if (TimerPickup.seconds() > 0.25) {
                            claw.currentStatus = ClawController.ClawStatus.CLOSE;
                        }
                        if(TimerPickup.seconds() > 0.50)
                        {
                            linkageSlides.currentStatus= LinkageSlidesController.LinkageSlidesStatus.INIT;
                            linkage.currentStatus= LinkageController.LinkageStatus.INIT;
                            clawRotate.currentStatus= ClawRotateController.ClawRotateStatus.INIT;
                            scoreSystem.currentStatus= ScoreSystemController.ScoreSystemStatus.INIT_AUTO;
                            TimerScore.reset();
                            follower.followPath(scorePickup3Inter);
                            status=STROBOT.SCORE_INTER3;
//                            status=STROBOT.END_AUTO;
                        }
                    }
                    break;
                }
                case SCORE_INTER3:
                {
                    if(follower.isBusy())
                    {
                        TimerScore.reset();
                    }
                    if(!follower.isBusy())
                    {
                        if(TimerScore.seconds()>0.2)
                        {
                            follower.followPath(scorePickup3);
                            TimerScore.reset();
                            status=STROBOT.SCORE_PICKUP3;
                        }
                        linkageSlides.currentStatus= LinkageSlidesController.LinkageSlidesStatus.BSK_HIGH;
                    }
                    break;
                }
                case SCORE_PICKUP3:
                {
                    if(TimerScore.seconds()>1)
                    {
                        scoreSystem.currentStatus = ScoreSystemController.ScoreSystemStatus.SCORE;
                    }
                    if(!follower.isBusy() && TimerScore.seconds()>1.3)
                    {
//                        scoreSystem.currentStatus= ScoreSystemController.ScoreSystemStatus.FLICK;
                        claw.currentStatus= ClawController.ClawStatus.OPEN;
                        if(TimerScore.seconds()>2)
                        {
//                            follower.followPath(pickup3);
                            TimerLeave.reset();
                            status = STROBOT.END_AUTO;
                        }
                    }
                    break;
                }
            }
            linkage.update(linkage_current_position,linkage_target_position);
            slides.update(slides_current_position,slides_target_position);
            claw.update();
            fourbar.update();
            clawPosition.update();
            clawRotate.update(claw_rotate_target, 0);
            scoreSystem.update(robot.encoderClaw.getVoltage());
            follower.update();
            linkageSlides.update();
            telemetry.addData("status", status);
            telemetry.addData("slidescurrent", slides_current_position);
            telemetry.addData("clawposition",clawPosition.currentStatus);
            telemetry.addData("scoresysten",scoreSystem.currentStatus);
            telemetry.addData("Current Traj", follower.getCurrentPath());
            telemetry.addData("beam",beam.getState());
            telemetry.addData("ok", ok);
            telemetry.addData("pose", follower.getPose());
            telemetry.update();
        }
    }

}