package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
@TeleOp(name="FullTeleOp", group="Pushbot")
public class FullTeleOp extends OpMode {
    //Intake ramp, outtake ramp, strafing
    DcMotor fl;
    DcMotor fr;
    DcMotor rl;
    DcMotor rr;
    DcMotor intake;
    DcMotor outtake;
    DcMotor too;
    double motorPower;
    double intakePower;
    double outtakePower;
    @Override
    public void init() {
        fl = hardwareMap.dcMotor.get("frontleft");
        fr = hardwareMap.dcMotor.get("frontright");
        rl = hardwareMap.dcMotor.get("backleft");
        rr = hardwareMap.dcMotor.get("backright");
        intake = hardwareMap.dcMotor.get("intake");
        outtake = hardwareMap.dcMotor.get("outake2");
        too = hardwareMap.dcMotor.get("too1");
        fr.setDirection(DcMotorSimple.Direction.REVERSE);
        rr.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    @Override
    public void loop() {
        //gamepad1: moving (rightsticky) and strafing (leftstickx)
        if (gamepad1.right_stick_y!=0){
            motorPower = -gamepad1.right_stick_y;
            fl.setPower(motorPower);
            fr.setPower(motorPower);
            rr.setPower(motorPower);
            rl.setPower(motorPower);
        }else{
            motorPower = gamepad1.left_stick_x;
            strafe(motorPower);
        }
        //gamepad2: intake and outtake
        outtake.setPower(1);
        intakePower = -gamepad2.left_stick_y;
        intake.setPower(intakePower);
        outtakePower = -gamepad2.right_stick_y;
        too.setPower(outtakePower);
    }
    public void strafe(double power){
        fl.setPower(power);
        fr.setPower(power);
        rr.setPower(power);
        rl.setPower(power);
    }
}
