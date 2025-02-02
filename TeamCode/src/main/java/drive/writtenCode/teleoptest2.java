package drive.writtenCode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import drive.writtenCode.controllers.LinkageController;


@TeleOp(name="TeleOptest2", group="Linear OpMode")
public class teleoptest2 extends LinearOpMode {

    DcMotorEx encoder;
    LinkageController linkageController;
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        while(opModeIsActive())
        {
//            linkage.setPower(0.5);
//            telemetry.addData("linkage power", linkage.getPower());
//            telemetry.addData("powerdraw linkage", linkage.getCurrent(CurrentUnit.AMPS));
            telemetry.update();
        }
    }
}