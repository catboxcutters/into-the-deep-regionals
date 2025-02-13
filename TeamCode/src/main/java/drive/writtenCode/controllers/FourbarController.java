package drive.writtenCode.controllers;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.Servo;

import drive.writtenCode.RobotMap;

@Config
public class FourbarController {
    public enum FourbarStatus{
        INIT,
        COLLECT,
        SUB,
        COLLECT_SUB,
        RUNTO,
        FLICK,
        RUNG,
        RUNG_SIDE_RETRACT,
        SCORE,
        SCORE_LOW,
        FEED, RUNG_PARA, SUB_INTER,PERIMETER,BACKWARDS_SCORE,BACKWARDS,SAMPLE;
    }
    public FourbarStatus currentStatus = FourbarStatus.INIT;
    public FourbarStatus previousStatus=null;
    public static double init_position=0.84;
    public static double rung_position = 0.27;
    public static double perimeter = 0.74;
    public static double sub_inter_position = 0.51;
    public static double sub_position = 0.45;
    public static double sub_collect_position = 0.39;
    public static double rung_side_retract_position = 0.49;
    public static double collect_position = 0.88;
    public static double score_position = 0.64;
    public static double score_low = 0.64;
    public static double rung_para_position = 0.09;
    public static double back_score_position = 0.26;
    public static double back_position = 0.40;
    public static double flick = 0.47;
    public static double feed = 0.7;
    public static double sample = 0.91;
    public Servo fourbarLeft = null, fourbarRight = null;
    public FourbarController(RobotMap robot) {
        this.fourbarLeft=robot.fourbarLeft;
        this.fourbarRight = robot.fourbarRight;
    }
    public void update()
    {
        if(currentStatus!=previousStatus || currentStatus == FourbarStatus.RUNTO){
            previousStatus=currentStatus;
            switch(currentStatus)
            {
                case INIT:
                {
                    this.fourbarLeft.setPosition(init_position);
                    this.fourbarRight.setPosition(init_position);
                    break;
                }
                case COLLECT:
                {
                    this.fourbarLeft.setPosition(collect_position);
                    this.fourbarRight.setPosition(collect_position);
                    break;
                }
                case COLLECT_SUB:
                {
                    this.fourbarLeft.setPosition(sub_collect_position);
                    this.fourbarRight.setPosition(sub_collect_position);
                    break;
                }
                case SUB:
                {
                    this.fourbarLeft.setPosition(sub_position);
                    this.fourbarRight.setPosition(sub_position);
                    break;
                }
                case SCORE:
                {
                    this.fourbarLeft.setPosition(score_position);
                    this.fourbarRight.setPosition(score_position);
                    break;
                }
                case SCORE_LOW:
                {
                    this.fourbarLeft.setPosition(score_low);
                    this.fourbarRight.setPosition(score_low);
                    break;
                }
                case FLICK:
                {
                    this.fourbarLeft.setPosition(flick);
                    this.fourbarRight.setPosition(flick);
                    break;
                }
                case FEED:
                {
                    this.fourbarLeft.setPosition(feed);
                    this.fourbarRight.setPosition(feed);
                    break;
                }
                case RUNG:
                {
                    this.fourbarLeft.setPosition(rung_position);
                    this.fourbarRight.setPosition(rung_position);
                    break;
                }
                case SUB_INTER:
                {
                    this.fourbarLeft.setPosition(sub_inter_position);
                    this.fourbarRight.setPosition(sub_inter_position);
                    break;
                }
                case RUNG_PARA:
                {
                    this.fourbarLeft.setPosition(rung_para_position);
                    this.fourbarRight.setPosition(rung_para_position);
                    break;
                }
                case RUNG_SIDE_RETRACT:
                {
                    this.fourbarLeft.setPosition(rung_side_retract_position);
                    this.fourbarRight.setPosition(rung_side_retract_position);
                    break;
                }
                case PERIMETER:
                {
                    this.fourbarLeft.setPosition(perimeter);
                    this.fourbarRight.setPosition(perimeter);
                    break;
                }
                case BACKWARDS_SCORE:
                {
                    this.fourbarLeft.setPosition(back_score_position);
                    this.fourbarRight.setPosition(back_score_position);
                    break;
                }
                case BACKWARDS:
                {
                    this.fourbarLeft.setPosition(back_position);
                    this.fourbarRight.setPosition(back_position);
                    break;
                }
                case SAMPLE:
                {
                    this.fourbarLeft.setPosition(sample);
                    this.fourbarRight.setPosition(sample);
                    break;
                }
            }
        }
    }
}