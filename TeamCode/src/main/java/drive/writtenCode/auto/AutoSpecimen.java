package drive.writtenCode.auto;

import static drive.writtenCode.auto.AutoSpecimen.STROBOT.DELIVER_PICKUP3;
import static drive.writtenCode.auto.AutoSpecimen.STROBOT.END_AUTO;
import static drive.writtenCode.auto.AutoSpecimen.STROBOT.FAILSAFE_BACK;
import static drive.writtenCode.auto.AutoSpecimen.STROBOT.FAILSAFE_FORTH;
import static drive.writtenCode.auto.AutoSpecimen.STROBOT.GRAB_SPECIMEN1;
import static drive.writtenCode.auto.AutoSpecimen.STROBOT.GRAB_SPECIMEN2;
import static drive.writtenCode.auto.AutoSpecimen.STROBOT.GRAB_SPECIMEN3;
import static drive.writtenCode.auto.AutoSpecimen.STROBOT.GRAB_SPECIMEN4;
import static drive.writtenCode.auto.AutoSpecimen.STROBOT.PICKUP2;
import static drive.writtenCode.auto.AutoSpecimen.STROBOT.PICKUP3;
import static drive.writtenCode.auto.AutoSpecimen.STROBOT.SCORE_SPECIMEN1;
import static drive.writtenCode.auto.AutoSpecimen.STROBOT.SCORE_SPECIMEN2;
import static drive.writtenCode.auto.AutoSpecimen.STROBOT.SCORE_SPECIMEN3;
import static drive.writtenCode.auto.AutoSpecimen.STROBOT.SCORE_SPECIMEN4;
import static drive.writtenCode.auto.AutoSpecimen.STROBOT.SPECIMEN1;
import static drive.writtenCode.auto.AutoSpecimen.STROBOT.SPECIMEN2;
import static drive.writtenCode.auto.AutoSpecimen.STROBOT.SPECIMEN3;
import static drive.writtenCode.auto.AutoSpecimen.STROBOT.SPECIMEN4;

import com.acmerobotics.dashboard.config.Config;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;
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

public class AutoSpecimen extends LinearOpMode {



    public static double PRELOAD_LEFT_X = -38;
    public static double PRELOAD_LEFT_Y = 62;
    private void wait(int ms) {
        try{
            Thread.sleep(ms);
        }
        catch (InterruptedException ignored) {
            //nu face nimic
        }
    }


    public enum STROBOT
    {
        START,
        PLACE_PRELOAD,
        PICKUP1, GRAB_PICKUP1, DELIVER_PICKUP1, PICKUP2, DELIVER_PICKUP2, GRAB_PICKUP2, PICKUP3, GRAB_PICKUP3, DELIVER_PICKUP3, END_AUTO
        ,SPECIMEN1,GRAB_SPECIMEN1, GRAB_SPECIMEN2, SCORE_SPECIMEN2, SPECIMEN2, SPECIMEN3, GRAB_SPECIMEN3, SCORE_SPECIMEN3, SPECIMEN4, GRAB_SPECIMEN4, SCORE_SPECIMEN4, FAILSAFE, FAILSAFE_BACK, FAILSAFE_FORTH, SCORE_SPECIMEN1
    }
    ElapsedTime AutoTimer = new ElapsedTime();
    ElapsedTime TimerLeave = new ElapsedTime();
    ElapsedTime TimerPlacePreload = new ElapsedTime();
    ElapsedTime TimerPickup = new ElapsedTime();
    ElapsedTime TimerDeliver = new ElapsedTime();
    ElapsedTime TimerScore = new ElapsedTime();
    ElapsedTime TimerFailsafe = new ElapsedTime();
    private double time_place_preload = 1.3;
    private double time_leave_preload = 0.7;
    private Follower follower;

    int linkage_target_position = 0;
    int claw_rotate_target = 0;
    int slides_target_position = 0;

    private Path specimen1,specimen4,specimen4Score,specimen3,specimen3Score,specimen2,specimen2Score,specimen1Score,scorePreload,grabPickup1,deliver1,grabPickup2,deliver2,grabPickup3,deliver3,park;
    private Path failsafe1_back, failsafe1_forth, failsafe2_back, failsafe2_forth, failsafe3_back, failsafe3_forth, failsafe4_back, failsafe4_forth;
    public static double tunex=0;
    public static double tuney=0;
    final Pose startPose = new Pose(10, 63, Math.toRadians(0));
    final Pose preloadPose = new Pose(42.5, 73, Math.toRadians(0));
//    final Pose pickup1Pose = new Pose(35.25, 33.5, Math.toRadians(-50));
//    final Point pickup1Point = new Point(16, 43, Point.CARTESIAN);
//    final Point pickup1Point2 = new Point(27, 41, Point.CARTESIAN);
//    final Pose deliver1Pose = new Pose(39,26,Math.toRadians(190));
//    final Pose pickup2Pose = new Pose(39,26,Math.toRadians(310)); //41 19
    final Pose pickup1Pose = new Pose(32.4, 22.7, Math.toRadians(0));
//    final Point pickup1Point = new Point(28, 47, Point.CARTESIAN);
    final Point pickup1Point = new Point(30, 67, Point.CARTESIAN);
    final Point pickup1Point2 = new Point(20, 26, Point.CARTESIAN);
    final Pose deliver1Pose = new Pose(24,12,Math.toRadians(0));
    final Pose pickup2Pose = new Pose(34.8,12.7,Math.toRadians(0)); //41 19
    final Pose deliver2Pose = new Pose(24,12,Math.toRadians(0));
    final Pose pickup3Pose = new Pose(37.5,12.3,Math.toRadians(305));
    final Pose deliver3Pose = new Pose(24,12,Math.toRadians(0));

    final Pose failsafe_pose1 = new Pose(22,34.3,Math.toRadians(0));
    final Pose specimen1Pose = new Pose(15,34.3,Math.toRadians(0));
//    final Point specimen1Point1 = new Point(31, 21, Point.CARTESIAN);
    final Point specimen1Point1 = new Point(34.3, 39, Point.CARTESIAN);
    final Point specimen1Point2 = new Point(28, 34.3, Point.CARTESIAN);


    final Pose specimen1ScorePose = new Pose(40, 68, Math.toRadians(0));

    final Pose failsafe_pose2 = new Pose(22,32.5,Math.toRadians(0));
    final Pose specimen2Pose = new Pose(10.5,32.5,Math.toRadians(0));
    final Point specimen2Point1 = new Point(28, 52, Point.CARTESIAN);
    final Point specimen2Point2 = new Point(51, 29, Point.CARTESIAN);
    final Pose specimen2ScorePose = new Pose(40, 66, Math.toRadians(0));

    final Pose failsafe_pose3 = new Pose(22,32.5,Math.toRadians(0));
    final Pose specimen3Pose = new Pose(10.3,32.5,Math.toRadians(0));
    final Point specimen3Point1 = new Point(28, 52, Point.CARTESIAN);
    final Point specimen3Point2 = new Point(51, 29, Point.CARTESIAN);
    final Pose specimen3ScorePose = new Pose(40, 64, Math.toRadians(0));

    final Pose failsafe_pose4 = new Pose(22,32.5,Math.toRadians(0));
    final Pose specimen4Pose = new Pose(10.4,32.5,Math.toRadians(0));
    final Point specimen4Point1 = new Point(28, 52, Point.CARTESIAN);
    final Point specimen4Point2 = new Point(51, 29, Point.CARTESIAN);
    final Pose specimen4ScorePose = new Pose(40, 63, Math.toRadians(0));

    final Pose parkPose =new Pose(13,32.4, Math.toRadians(0));


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

        scorePreload = new Path(new BezierLine(new Point(startPose), new Point(preloadPose)));
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(),preloadPose.getHeading());

        grabPickup1 = new Path(new BezierCurve(new Point(preloadPose), pickup1Point, pickup1Point2, new Point(pickup1Pose)));
        grabPickup1.setLinearHeadingInterpolation(preloadPose.getHeading(),pickup1Pose.getHeading());
        grabPickup1.setZeroPowerAccelerationMultiplier(2.5);

        deliver1 = new Path(new BezierLine(new Point(pickup1Pose),new Point(deliver1Pose)));
        deliver1.setLinearHeadingInterpolation(pickup1Pose.getHeading(), deliver1Pose.getHeading());

        grabPickup2 = new Path(new BezierLine(new Point(deliver1Pose), new Point(pickup2Pose)));
        grabPickup2.setLinearHeadingInterpolation(deliver1Pose.getHeading(), pickup2Pose.getHeading());

        deliver2 = new Path(new BezierLine(new Point(pickup2Pose), new Point(deliver2Pose)));
        deliver2.setLinearHeadingInterpolation(pickup2Pose.getHeading(), deliver2Pose.getHeading());

        grabPickup3 = new Path(new BezierLine(new Point(deliver2Pose), new Point(pickup3Pose)));
        grabPickup3.setLinearHeadingInterpolation(deliver2Pose.getHeading(), pickup3Pose.getHeading());

        deliver3 = new Path(new BezierLine(new Point(pickup3Pose), new Point(deliver3Pose)));
        deliver3.setLinearHeadingInterpolation(pickup3Pose.getHeading(), deliver3Pose.getHeading());

        specimen1 = new Path(new BezierCurve(
                new Point(deliver3Pose),
                specimen1Point1,
                specimen1Point2,
                new Point(specimen1Pose)
        ));
        specimen1.setLinearHeadingInterpolation(deliver3Pose.getHeading(), specimen1Pose.getHeading());

        specimen1Score=new Path(new BezierLine(new Point(specimen1Pose), new Point(specimen1ScorePose)));
        specimen1Score.setLinearHeadingInterpolation(specimen1Pose.getHeading(), specimen1ScorePose.getHeading());

        specimen2 = new Path(new BezierCurve(
                new Point(specimen1ScorePose),
                specimen2Point1,
                specimen2Point2,
                new Point(specimen2Pose)
        ));
        specimen2.setLinearHeadingInterpolation(specimen1ScorePose.getHeading(), specimen2Pose.getHeading());

        specimen2Score=new Path(new BezierLine(new Point(specimen2Pose), new Point(specimen2ScorePose)));
        specimen2Score.setLinearHeadingInterpolation(specimen2Pose.getHeading(), specimen2ScorePose.getHeading());

        specimen3 = new Path(new BezierCurve(
                new Point(specimen2ScorePose),
                specimen3Point1,
                specimen3Point2,
                new Point(specimen3Pose)
        ));
        specimen3.setLinearHeadingInterpolation(specimen2ScorePose.getHeading(), specimen3Pose.getHeading());

        specimen3Score=new Path(new BezierLine(new Point(specimen3Pose), new Point(specimen3ScorePose)));
        specimen3Score.setLinearHeadingInterpolation(specimen3Pose.getHeading(), specimen3ScorePose.getHeading());

        specimen4 = new Path(new BezierCurve(
                new Point(specimen3ScorePose),
                specimen4Point1,
                specimen4Point2,
                new Point(specimen4Pose)
        ));
        specimen4.setLinearHeadingInterpolation(specimen3ScorePose.getHeading(), specimen4Pose.getHeading());

        specimen4Score=new Path(new BezierLine(new Point(specimen4Pose), new Point(specimen4ScorePose)));
        specimen4Score.setLinearHeadingInterpolation(specimen4Pose.getHeading(), specimen4ScorePose.getHeading());

        failsafe1_back = new Path(new BezierLine(new Point(specimen1Pose), new Point(failsafe_pose1))); failsafe1_back.setConstantHeadingInterpolation(specimen1Pose.getHeading());
        failsafe1_forth = new Path(new BezierLine(new Point(failsafe_pose1), new Point(specimen1Pose))); failsafe1_forth.setConstantHeadingInterpolation(specimen1Pose.getHeading());
        failsafe2_back = new Path(new BezierLine(new Point(specimen2Pose), new Point(failsafe_pose2))); failsafe2_back.setConstantHeadingInterpolation(specimen2Pose.getHeading());
        failsafe2_forth = new Path(new BezierLine(new Point(failsafe_pose2), new Point(specimen2Pose))); failsafe2_forth.setConstantHeadingInterpolation(specimen2Pose.getHeading());
        failsafe3_back = new Path(new BezierLine(new Point(specimen3Pose), new Point(failsafe_pose3))); failsafe3_back.setConstantHeadingInterpolation(specimen3Pose.getHeading());
        failsafe3_forth = new Path(new BezierLine(new Point(failsafe_pose3), new Point(specimen3Pose))); failsafe3_forth.setConstantHeadingInterpolation(specimen3Pose.getHeading());
        failsafe4_back = new Path(new BezierLine(new Point(specimen4Pose), new Point(failsafe_pose4))); failsafe4_back.setConstantHeadingInterpolation(specimen4Pose.getHeading());
        failsafe4_forth = new Path(new BezierLine(new Point(failsafe_pose4), new Point(specimen4Pose))); failsafe4_forth.setConstantHeadingInterpolation(specimen4Pose.getHeading());

        park = new Path(new BezierLine(new Point(specimen4ScorePose),new Point(parkPose)));
        park.setLinearHeadingInterpolation(specimen4ScorePose.getHeading(), parkPose.getHeading());
        boolean ok=false;
        int current_specimen = 1;

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
                    follower.followPath(scorePreload,false);
                    scoreSystem.currentStatus = ScoreSystemController.ScoreSystemStatus.RUNG;
//                    linkageSlides.currentStatus = LinkageSlidesController.LinkageSlidesStatus.HIGH_RUNG;
                    status = STROBOT.PLACE_PRELOAD;
                    break;
                }
                case PLACE_PRELOAD: {
                    if(TimerPlacePreload.seconds()>0)
                    {
                        linkageSlides.currentStatus = LinkageSlidesController.LinkageSlidesStatus.HIGH_RUNG_AUTO;
                    }
                    if(TimerPlacePreload.seconds()>time_place_preload) {
                    linkageSlides.currentStatus = LinkageSlidesController.LinkageSlidesStatus.HIGH_RUNG_SCORE;
                    scoreSystem.currentStatus= ScoreSystemController.ScoreSystemStatus.OPEN_RUNG;
                    TimerLeave.reset();
                    status = STROBOT.PICKUP1;
                    }
                    break;
                }
                case PICKUP1:
                {
                    if(TimerLeave.seconds()>time_leave_preload)
                    {
                        follower.followPath(grabPickup1, false);
                        status=STROBOT.GRAB_PICKUP1;
                    }
                    break;
                }
                case GRAB_PICKUP1:
                {
                    if(TimerLeave.seconds()>time_leave_preload+0.5)
                    {
                        slides.currentStatus= SlidesController.SlidesStatus.INIT;
                        linkage.currentStatus = LinkageController.LinkageStatus.COLLECT;
                        scoreSystem.currentStatus= ScoreSystemController.ScoreSystemStatus.COLLECT_FROM_SUB;
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
                            if(TimerPickup.seconds()>0.35)
                            {
                                linkage.currentStatus= LinkageController.LinkageStatus.INIT;
                                clawRotate.currentStatus = ClawRotateController.ClawRotateStatus.INIT;
                                fourbar.currentStatus = FourbarController.FourbarStatus.INIT;
                                clawPosition.currentStatus = ClawPositionController.ClawPositionStatus.INIT;
                                follower.followPath(deliver1);
                                TimerDeliver.reset();
                                status=STROBOT.DELIVER_PICKUP1;
                            }
                        }
                    }
                    break;
                }
                case DELIVER_PICKUP1:
                {
                    if(TimerDeliver.seconds()>0.75)
                    {
                        claw.currentStatus = ClawController.ClawStatus.OPEN;
                        TimerLeave.reset();
                        status = PICKUP2;
                    }
                    break;
                }
                case PICKUP2:
                {
                    if(TimerLeave.seconds()>0)
                    {
                        follower.followPath(grabPickup2, false);
                        status=STROBOT.GRAB_PICKUP2;
                    }
                    break;
                }
                case GRAB_PICKUP2:
                {
                    if(TimerLeave.seconds()>time_leave_preload-0.55)
                    {
                        slides.currentStatus= SlidesController.SlidesStatus.INIT;
                        linkage.currentStatus = LinkageController.LinkageStatus.COLLECT;
                        fourbar.currentStatus= FourbarController.FourbarStatus.SUB;
                        clawPosition.currentStatus = ClawPositionController.ClawPositionStatus.SUB;
                        claw.currentStatus= ClawController.ClawStatus.OPEN;
                        if(TimerLeave.seconds()<time_leave_preload+0.5)
                        {
                            TimerPickup.reset();
                        }
                        if(TimerLeave.seconds()>time_leave_preload+0.5)
                        {
                            fourbar.currentStatus = FourbarController.FourbarStatus.COLLECT_SUB;
                            clawPosition.currentStatus = ClawPositionController.ClawPositionStatus.COLLECT_SUB;
                            if (TimerPickup.seconds() > 0.2) {
                                claw.currentStatus = ClawController.ClawStatus.CLOSE;
                            }
                            if(TimerPickup.seconds()>0.5)
                            {
                                linkage.currentStatus= LinkageController.LinkageStatus.INIT;
                                clawRotate.currentStatus = ClawRotateController.ClawRotateStatus.INIT;
                                fourbar.currentStatus = FourbarController.FourbarStatus.INIT;
                                clawPosition.currentStatus = ClawPositionController.ClawPositionStatus.INIT;
                                follower.followPath(deliver2);
                                TimerDeliver.reset();
                                status=STROBOT.DELIVER_PICKUP2;
                            }
                        }
                    }
                    break;
                }
                case DELIVER_PICKUP2:
                {
                    if(TimerDeliver.seconds()>0.75)
                    {
                        claw.currentStatus = ClawController.ClawStatus.OPEN;
                        TimerLeave.reset();
                        status=PICKUP3;
                    }
                    break;
                }
                case PICKUP3:
                {
                    if(TimerLeave.seconds()>0)
                    {
                        follower.followPath(grabPickup3, false);
                        status=STROBOT.GRAB_PICKUP3;
                    }
                    break;
                }
                case GRAB_PICKUP3:
                {
                    if(TimerLeave.seconds()>time_leave_preload-0.6)
                    {

                        slides.currentStatus= SlidesController.SlidesStatus.INIT;
                        linkage.currentStatus = LinkageController.LinkageStatus.COLLECT;
                        fourbar.currentStatus= FourbarController.FourbarStatus.SUB;
                        clawPosition.currentStatus = ClawPositionController.ClawPositionStatus.SUB;
                        clawRotate.currentStatus= ClawRotateController.ClawRotateStatus.PLUS;
                        claw.currentStatus= ClawController.ClawStatus.OPEN;
                        if(TimerLeave.seconds()<time_leave_preload+0.8)
                        {
                            TimerPickup.reset();
                        }
                        if(TimerLeave.seconds()>time_leave_preload+0.8)
                        {
                            fourbar.currentStatus = FourbarController.FourbarStatus.COLLECT_SUB;
                            clawPosition.currentStatus = ClawPositionController.ClawPositionStatus.COLLECT_SUB;
                            if (TimerPickup.seconds() > 0.3) {
                                claw.currentStatus = ClawController.ClawStatus.CLOSE;
                            }
                            if(TimerPickup.seconds()>0.6)
                            {
                                linkage.currentStatus= LinkageController.LinkageStatus.INIT;
                                if(TimerPickup.seconds()>0.7) {
                                    TimerDeliver.reset();
                                    clawRotate.currentStatus = ClawRotateController.ClawRotateStatus.INIT;
                                    fourbar.currentStatus = FourbarController.FourbarStatus.INIT;
                                    clawPosition.currentStatus = ClawPositionController.ClawPositionStatus.INIT;
                                    if(!follower.isBusy()) {
                                        follower.followPath(deliver3);
                                    }
                                    status = DELIVER_PICKUP3;
                                }
                            }
                        }
                    }
                    break;
                }
                case DELIVER_PICKUP3:
                {
                    if(TimerDeliver.seconds()>0.75)
                    {
                        claw.currentStatus = ClawController.ClawStatus.OPEN;
                        TimerLeave.reset();
                        status=SPECIMEN1;
                    }
                    break;
                }
                case SPECIMEN1:
                {
                    if(TimerLeave.seconds()>0)
                    {
                        follower.followPath(specimen1);
                        status = GRAB_SPECIMEN1;
                        current_specimen = 1;
                        TimerLeave.reset();
                    }
                    break;
                }
                case GRAB_SPECIMEN1:
                {
                    if(!follower.isBusy() && beam.getState()==false)
                    {
                        if(ok==false) {
                            scoreSystem.currentStatus = ScoreSystemController.ScoreSystemStatus.LOWER_FOURBAR;
                            ok=true;
                        }
                        if(claw.currentStatus == ClawController.ClawStatus.CLOSE_SPECIMEN && TimerLeave.seconds()>0.5)
                        {
                            status = SCORE_SPECIMEN1;
                            ok=false;
                            follower.followPath(specimen1Score);
                            TimerScore.reset();
                        }
                    }
                    else if(!follower.isBusy() && beam.getState()==true && TimerLeave.seconds()>3.5)
                    {
                        status=FAILSAFE_BACK;
                    }
                    break;
                }
                case SCORE_SPECIMEN1:
                {
                    if(TimerScore.seconds()>0.4 && ok==false)
                    {
                        scoreSystem.currentStatus = ScoreSystemController.ScoreSystemStatus.RUNG;
                        linkageSlides.currentStatus = LinkageSlidesController.LinkageSlidesStatus.HIGH_RUNG;
                        ok=true;
                    }
                    if(!follower.isBusy() && linkageSlides.currentStatus== LinkageSlidesController.LinkageSlidesStatus.HIGH_RUNG)
                    {
                        linkageSlides.currentStatus= LinkageSlidesController.LinkageSlidesStatus.HIGH_RUNG_SCORE;
                        scoreSystem.currentStatus= ScoreSystemController.ScoreSystemStatus.OPEN_RUNG;
                        TimerLeave.reset();
                    }
                    if(!follower.isBusy()
                            && linkageSlides.currentStatus== LinkageSlidesController.LinkageSlidesStatus.HIGH_RUNG_SCORE
                            && TimerLeave.seconds()>0.1)
                    {
                        ok=false;
                        status=SPECIMEN2;
                        TimerLeave.reset();
                    }
                    break;
                }
                case SPECIMEN2:
                {
                        follower.followPath(specimen2);
                        ok=false;
                        status = GRAB_SPECIMEN2;
                        current_specimen = 2;
                        TimerLeave.reset();
                    break;
                }
                case GRAB_SPECIMEN2:
                {
                    if(!follower.isBusy() && beam.getState()==false)
                    {
                        if(ok==false) {
                            scoreSystem.currentStatus = ScoreSystemController.ScoreSystemStatus.LOWER_FOURBAR;
                            ok=true;
                        }
                        if(claw.currentStatus == ClawController.ClawStatus.CLOSE_SPECIMEN && TimerLeave.seconds()>1.5)
                        {
                            status = SCORE_SPECIMEN2;
                            ok=false;
                            follower.followPath(specimen2Score);
                            TimerScore.reset();
                        }
                    }
                    else if(!follower.isBusy() && beam.getState()==true && TimerLeave.seconds()>3.5)
                    {
                        status=FAILSAFE_BACK;
                    }
                    if(TimerLeave.seconds()>0.3 && linkageSlides.currentStatus!= LinkageSlidesController.LinkageSlidesStatus.INIT)
                    {
                        linkageSlides.currentStatus= LinkageSlidesController.LinkageSlidesStatus.INIT;
                        scoreSystem.currentStatus= ScoreSystemController.ScoreSystemStatus.INIT;
                    }


                    break;
                }
                case SCORE_SPECIMEN2:
                {
                    if(TimerScore.seconds()>0.4 && ok==false)
                    {
                        scoreSystem.currentStatus = ScoreSystemController.ScoreSystemStatus.RUNG;
                        linkageSlides.currentStatus = LinkageSlidesController.LinkageSlidesStatus.HIGH_RUNG;
                        ok=true;
                    }
                    if(!follower.isBusy() && linkageSlides.currentStatus== LinkageSlidesController.LinkageSlidesStatus.HIGH_RUNG)
                    {
                        linkageSlides.currentStatus= LinkageSlidesController.LinkageSlidesStatus.HIGH_RUNG_SCORE;
                        scoreSystem.currentStatus= ScoreSystemController.ScoreSystemStatus.OPEN_RUNG;
                        TimerLeave.reset();
                    }
                    if(!follower.isBusy()
                            && linkageSlides.currentStatus== LinkageSlidesController.LinkageSlidesStatus.HIGH_RUNG_SCORE
                            && TimerLeave.seconds()>0.1)
                    {
                        ok=false;
                        status=SPECIMEN3;
                        TimerLeave.reset();
                    }
                    break;
                }
                case SPECIMEN3:
                {
                        follower.followPath(specimen3);
                        ok=false;
                        status = GRAB_SPECIMEN3;
                        current_specimen = 3;
                        TimerLeave.reset();
                    break;
                }
                case GRAB_SPECIMEN3:
                {

                    if(TimerLeave.seconds()>0.3 && linkageSlides.currentStatus!= LinkageSlidesController.LinkageSlidesStatus.INIT)
                    {
                        linkageSlides.currentStatus= LinkageSlidesController.LinkageSlidesStatus.INIT;
                        scoreSystem.currentStatus= ScoreSystemController.ScoreSystemStatus.INIT;
                    }
                    if(!follower.isBusy() && beam.getState()==false)
                    {
                        if(ok==false) {
                            scoreSystem.currentStatus = ScoreSystemController.ScoreSystemStatus.LOWER_FOURBAR;
                            ok=true;
                        }
                        if(claw.currentStatus == ClawController.ClawStatus.CLOSE_SPECIMEN && TimerLeave.seconds()>1.5)
                        {
                            status = SCORE_SPECIMEN3;
                            ok=false;
                            follower.followPath(specimen3Score);
                            TimerScore.reset();
                        }
                    }
                    else if(!follower.isBusy() && beam.getState()==true && TimerLeave.seconds()>3.5)
                    {
                        status=FAILSAFE_BACK;
                    }

                    break;
                }
                case SCORE_SPECIMEN3:
                {
                    if(TimerScore.seconds()>0.4 && ok==false)
                    {
                        scoreSystem.currentStatus = ScoreSystemController.ScoreSystemStatus.RUNG;
                        linkageSlides.currentStatus = LinkageSlidesController.LinkageSlidesStatus.HIGH_RUNG;
                        ok=true;
                    }
                    if(!follower.isBusy() && linkageSlides.currentStatus== LinkageSlidesController.LinkageSlidesStatus.HIGH_RUNG)
                    {
                        linkageSlides.currentStatus= LinkageSlidesController.LinkageSlidesStatus.HIGH_RUNG_SCORE;
                        scoreSystem.currentStatus= ScoreSystemController.ScoreSystemStatus.OPEN_RUNG;
                        TimerLeave.reset();
                    }
                    if(!follower.isBusy()
                            && linkageSlides.currentStatus== LinkageSlidesController.LinkageSlidesStatus.HIGH_RUNG_SCORE
                            && TimerLeave.seconds()>0.1)
                    {
                        ok=false;
                        TimerLeave.reset();
                        status=SPECIMEN4;
                    }
                    break;
                }
                case SPECIMEN4:
                {
                        follower.followPath(specimen4);
                        ok=false;
                        status = GRAB_SPECIMEN4;
                        current_specimen = 4;
                        TimerLeave.reset();
                    break;
                }
                case GRAB_SPECIMEN4:
                {
                    if(TimerLeave.seconds()>0.3 && linkageSlides.currentStatus!= LinkageSlidesController.LinkageSlidesStatus.INIT)
                    {
                        linkageSlides.currentStatus= LinkageSlidesController.LinkageSlidesStatus.INIT;
                        scoreSystem.currentStatus= ScoreSystemController.ScoreSystemStatus.INIT;
                    }
                    if(!follower.isBusy() && beam.getState()==false)
                    {
                        if(ok==false) {
                            scoreSystem.currentStatus = ScoreSystemController.ScoreSystemStatus.LOWER_FOURBAR;
                            ok=true;
                        }
                        if(claw.currentStatus == ClawController.ClawStatus.CLOSE_SPECIMEN && TimerLeave.seconds()>1.5)
                        {
                            status = SCORE_SPECIMEN4;
                            ok=false;
                            follower.followPath(specimen4Score);
                            TimerScore.reset();
                        }
                    }
                    if(!follower.isBusy() && beam.getState()==true && TimerLeave.seconds()>2)
                    {
//                        status=FAILSAFE_BACK;
                    }
                    break;
                }
                case SCORE_SPECIMEN4:
                {
                    if(TimerScore.seconds()>0.4 && ok==false)
                    {
                        scoreSystem.currentStatus = ScoreSystemController.ScoreSystemStatus.RUNG;
                        linkageSlides.currentStatus = LinkageSlidesController.LinkageSlidesStatus.HIGH_RUNG;
                        ok=true;
                    }
                    if(!follower.isBusy() && linkageSlides.currentStatus== LinkageSlidesController.LinkageSlidesStatus.HIGH_RUNG)
                    {
                        linkageSlides.currentStatus= LinkageSlidesController.LinkageSlidesStatus.HIGH_RUNG_SCORE;
                        scoreSystem.currentStatus= ScoreSystemController.ScoreSystemStatus.OPEN_RUNG;
                        TimerLeave.reset();
                    }
                    if(!follower.isBusy()  && linkageSlides.currentStatus== LinkageSlidesController.LinkageSlidesStatus.HIGH_RUNG_SCORE && TimerLeave.seconds()>0.25)
                    {
                        ok=false;
                        linkageSlides.currentStatus= LinkageSlidesController.LinkageSlidesStatus.INIT;
                        follower.followPath(park);
                        status=END_AUTO;
                    }
                    break;
                }
                case FAILSAFE_BACK:
                {
                    switch (current_specimen)
                    {
                        case 1:
                        {
                            follower.followPath(failsafe1_back);
                            status = FAILSAFE_FORTH;
                            break;
                        }
                        case 2:
                        {
                            follower.followPath(failsafe2_back);
                            status = FAILSAFE_FORTH;
                            break;
                        }
                        case 3:
                        {
                            follower.followPath(failsafe3_back);
                            status = FAILSAFE_FORTH;
                            break;
                        }
                        case 4:
                        {
                            follower.followPath(failsafe4_back);
                            status = FAILSAFE_FORTH;
                            break;
                        }
                    }
                    break;
                }
                case FAILSAFE_FORTH:
                {
                    if(!follower.isBusy())
                    {
                        switch(current_specimen)
                        {
                            case 1:
                            {
                                follower.followPath(failsafe1_forth);
                                status = GRAB_SPECIMEN1;
                                break;
                            }
                            case 2:
                            {
                                follower.followPath(failsafe2_forth);
                                status = GRAB_SPECIMEN2;
                                break;
                            }
                            case 3:
                            {
                                follower.followPath(failsafe3_forth);
                                status = GRAB_SPECIMEN3;
                                break;
                            }
                            case 4:
                            {
                                follower.followPath(failsafe4_forth);
                                status = GRAB_SPECIMEN4;
                                break;
                            }
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