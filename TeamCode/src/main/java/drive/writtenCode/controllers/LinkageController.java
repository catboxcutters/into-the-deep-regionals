package drive.writtenCode.controllers;

import static drive.writtenCode.controllers.LinkageController.LinkageStatus.CLIMB;
import static drive.writtenCode.controllers.LinkageController.LinkageStatus.KILL;
import static drive.writtenCode.controllers.LinkageController.LinkageStatus.RUNTO;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import drive.writtenCode.RobotMap;

@Config
public class LinkageController {
    public enum LinkageStatus {
        INIT,
        COLLECT,
        RUNTO,
        LOW_RUNG,
        LOW_RUNG_SCORE,
        HIGH_RUNG,
        HIGH_RUNG_SCORE,
        KILL,
        SCORE_HIGH, CLIMB, INVERSE_INIT
    }
    public LinkageStatus currentStatus = LinkageStatus.INIT;
    public LinkageStatus previousStatus = null;
    public static int init_position = 0;
    public static int collect_position = 3100;
    public static int inverse_init = -3100;
    public static int climb = 900;


    public DcMotorEx encoderLinkage = null;
    SimplePIDController linkagePID = null;
    SimplePIDController linkagePID_climb = null;
    SimplePIDController PID = null;
    public static double Kp = 0.002;//0.00325
    public static double Ki = 0;//0.0022
    public static double Kd = 0.004;

    public static double KpC = 0.002;//0.0009
    public static double KiC = 0;//0.1
    public static double KdC = 0.004;

    public static double PowerCap = 1;
    public static double maxSpeed = 1;
    public int current_position = init_position;
    public DcMotorEx linkage = null;
    public LinkageController(RobotMap robot) {
        this.linkage = robot.linkage;
        this.encoderLinkage = robot.encoderLinkage;
        linkagePID = new SimplePIDController(Kp, Ki, Kd);
        linkagePID.targetValue = init_position;
        linkagePID.maxOutput = maxSpeed;
        linkagePID_climb = new SimplePIDController(KpC, KiC,KdC);
        linkagePID_climb.targetValue = init_position;
        linkagePID_climb.maxOutput = 0.5;
        PID=linkagePID;
    }
    public void update(int linkage_position, int runto_target)
    {

        double powerColectare = PID.update(linkage_position);
        powerColectare = Math.max(-PowerCap,Math.min(powerColectare,PowerCap));
        if(currentStatus!=KILL) {
            this.linkage.setPower(-powerColectare);
        }
        else
        {
            this.linkage.setPower(0);
        }
        double linkage_current_position = encoderLinkage.getCurrentPosition();
        if(currentStatus!=previousStatus || currentStatus==RUNTO){
            previousStatus=currentStatus;
            switch(currentStatus) {
                case INIT:
                {
                    PID = linkagePID;
                    PID.targetValue = init_position-25;
                    break;
                }
                case SCORE_HIGH:
                {
                    PID = linkagePID;
                    PID.targetValue = init_position-125;
                    break;
                }
                case COLLECT:
                {
                    PID = linkagePID;
                    PID.targetValue = collect_position;
                    break;
                }
                case RUNTO:
                {
                    PID = linkagePID;
                    PID.targetValue = runto_target;
                    break;
                }
                case CLIMB:
                {
                    PID = linkagePID_climb;
                    PID.targetValue = climb;
                    break;
                }
                case INVERSE_INIT:
                {
                    PID=linkagePID;
                    PID.targetValue = inverse_init;
                    break;
                }
                case KILL:
                {
                    break;
                }
            }
        }
        if (Kp!=PID.p || Kd!=PID.d || Ki!=PID.i || maxSpeed !=PID.maxOutput && PID==linkagePID)
        {
            PID.p = Kp;
            PID.d = Kd;
            PID.i = Ki;
            PID.maxOutput = maxSpeed;
        }
        if (KpC!=PID.p || KdC!=PID.d || KiC!=PID.i || maxSpeed !=PID.maxOutput && PID==linkagePID_climb)
        {
            PID.p = KpC;
            PID.d = KdC;
            PID.i = KiC;
            PID.maxOutput = 0.5;
        }
    }
}
