package drive.writtenCode.controllers;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import drive.writtenCode.RobotMap;

@Config
public class SlidesController {
    public enum SlidesStatus {
        INIT,
        COLLECT,
        BSK_LOW,
        BSK_MID,
        BSK_HIGH,
        RUNTO,
        LOW_RUNG,
        LOW_RUNG_SCORE,
        HIGH_RUNG,
        RUNTO_H, HIGH_RUNG_SCORE,
        KILL, DELIVER, HIGH_RUNG_AUTO, PTO, CLIMB,AUTO
    }
    public SlidesStatus currentStatus = SlidesStatus.INIT;
    public SlidesStatus previousStatus = null;
    public static int init_position = 0;
    public static int collect_position = -16000;
    public static int bsk_low_position = -30000;
    public static int bsk_mid_position = -900;
    public static int bsk_high_position = -60500;
    public static int low_rung = -7000;
    public static int high_rung = -31780;
    public static int low_rung_score = -6000;
    public static int climb = -24000;
    public static int auto = -700;
    public static int high_rung_score = -18500;
    public static int deliver = -32000;
    public int current_position = init_position;
    public DcMotorEx slidesLeft = null;
    public DcMotorEx slidesMid = null;
    public DcMotorEx slidesRight = null;
    public DcMotorEx encoderSlides = null;
    SimplePIDController slidesPID = null;
    SimplePIDController slidesPID_horizontal=null;
    public SimplePIDController slidesPID_rung = null;
    public static SimplePIDController PID = null;

    public static double KpH = 0.00025;//0.00325
    public static double KiH = 0;//0.0022
    public static double KdH = 0.001;
    public static double Kp = 0.00025;//0.00325
    public static double Ki = 0.0001;//0.0022
    public static double Kd = 0; //0.001
    public static double KpR = 0.0001;
    public static double KiR = 0;
    public static double KdR = 0.001;
    public static double PowerCap = 1;
    public static double maxSpeed = 1;


    public SlidesController(RobotMap robot) {
        this.slidesLeft = robot.slidesLeft;
        this.slidesMid = robot.slidesMid;
        this.slidesRight = robot.slidesRight;
        this.encoderSlides = robot.linkage;
        slidesPID = new SimplePIDController(Kp, Ki, Kd);
        slidesPID.targetValue = init_position;
        slidesPID.maxOutput = maxSpeed;
        slidesPID_horizontal = new SimplePIDController(KpH, KiH, KdH);
        slidesPID_horizontal.targetValue = init_position;
        slidesPID_horizontal.maxOutput = maxSpeed;
        slidesPID_rung = new SimplePIDController(KpR, KiR, KdR);
        slidesPID_rung.targetValue = init_position;
        slidesPID_rung.maxOutput = 1;
        PID=slidesPID;
    }
    public void update(int slides_position, int runto_target)
    {
        double powerColectare = PID.update(encoderSlides.getCurrentPosition());
        double PowerCap_active = PID.maxOutput;
        powerColectare = Math.max(-PowerCap_active,Math.min(powerColectare,PowerCap_active));
        if(currentStatus==SlidesStatus.KILL) {
            this.slidesLeft.setPower(0);
            this.slidesRight.setPower(0);
            this.slidesMid.setPower(0);
        }
        else if(currentStatus==SlidesStatus.PTO)
        {
            this.slidesLeft.setPower(-0.5);
            this.slidesRight.setPower(-0.5);
            this.slidesMid.setPower(-0.5);
        }
        else
        {
            this.slidesLeft.setPower(-powerColectare);
            this.slidesRight.setPower(-powerColectare);
            this.slidesMid.setPower(-powerColectare);
        }
        double slides_current_position = encoderSlides.getCurrentPosition();
        if(currentStatus!=previousStatus || currentStatus==SlidesStatus.RUNTO || currentStatus==SlidesStatus.RUNTO_H){
            previousStatus=currentStatus;
            switch(currentStatus) {
                case INIT:
                {
                    PID=slidesPID;
                    PID.targetValue = init_position;
                    break;
                }
                case COLLECT:
                {
                    PID=slidesPID_horizontal;
                    PID.targetValue = collect_position;
                    break;
                }
                case BSK_LOW:
                {
                    PID=slidesPID;
                    PID.targetValue = bsk_low_position;
                    break;
                }
                case BSK_HIGH:
                {
                    PID=slidesPID;
                    PID.targetValue = bsk_high_position;
                    break;
                }
                case RUNTO:
                {
                    PID=slidesPID;
                    PID.targetValue = runto_target;
                    break;
                }
                case RUNTO_H:
                {
                    PID=slidesPID_horizontal;
                    PID.targetValue=runto_target;
                    break;
                }
                case LOW_RUNG:
                {
                    PID=slidesPID;
                    PID.targetValue = low_rung;
                    break;
                }
                case LOW_RUNG_SCORE:
                {
                    PID=slidesPID;
                    PID.targetValue = low_rung_score;
                    break;
                }
                case HIGH_RUNG:
                {
                    PID=slidesPID_rung;
                    PID.targetValue = high_rung;
                    break;
                }
                case HIGH_RUNG_SCORE:
                {
                    PID=slidesPID_rung;
                    PID.targetValue = high_rung_score;
                    break;
                }
                case HIGH_RUNG_AUTO:
                {
                    PID=slidesPID_rung;
                    PID.targetValue = high_rung;
                    break;
                }
                case DELIVER:
                {
                    PID=slidesPID_horizontal;
                    PID.targetValue = deliver;
                    break;
                }
                case CLIMB:
                {
                    PID=slidesPID;
                    PID.targetValue = climb;
                    break;
                }
                case AUTO:
                {
                    PID=slidesPID_horizontal;
                    PID.targetValue=auto;
                    break;
                }
                case PTO:
                {
                    break;
                }
                case KILL:
                {
                    break;
                }
            }
        }
        if ((Kp!=PID.p || Kd!=PID.d || Ki!=PID.i || maxSpeed !=PID.maxOutput) && PID==slidesPID)
        {
            PID.p = Kp;
            PID.d = Kd;
            PID.i = Ki;
            PID.maxOutput = maxSpeed;
        }
        if ((KpH!=PID.p || KdH!=PID.d || KiH!=PID.i || maxSpeed !=PID.maxOutput) && PID==slidesPID_horizontal)
        {
            PID.p = KpH;
            PID.d = KdH;
            PID.i = KiH;
            PID.maxOutput = 1;
        }
        if ((KpR!=PID.p || KdR!=PID.d || KiR!=PID.i || 1 !=PID.maxOutput) && PID==slidesPID_rung)
        {
            PID.p = KpR;
            PID.d = KdR;
            PID.i = KiR;
            PID.maxOutput = 1;
        }
    }
}
