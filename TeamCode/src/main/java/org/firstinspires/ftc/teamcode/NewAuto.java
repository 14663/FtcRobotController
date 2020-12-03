package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcontroller.external.samples.HardwarePushbot;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@Autonomous(name="FinalAuto", group="Pushbot")
public class NewAuto extends LinearOpMode {

    /* Declare OpMode members. */
    HardwarePushbot robot   = new HardwarePushbot();   // Use a Pushbot's hardware
    private ElapsedTime runtime = new ElapsedTime();

    static final double     COUNTS_PER_MOTOR_REV    = 537.6 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 19.2 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     DRIVE_SPEED             = 0.8;
    static final double     TURN_SPEED              = 0.7;
    static final double SPIN_DISTANCE = 3.1415*7.5;
    private DcMotor fleftDrive;
    private DcMotor rleftDrive;
    private DcMotor frightDrive;
    private DcMotor rrightDrive;
    private DcMotor outtake;
    private DcMotor too;
    private DcMotor intake;
    BNO055IMU imu;
    Orientation angles;
    Acceleration gravity;

    @Override
    public void runOpMode() throws InterruptedException {

        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        fleftDrive  = hardwareMap.get(DcMotor.class, "frontleft");
        frightDrive = hardwareMap.get(DcMotor.class, "frontright");
        rrightDrive = hardwareMap.get(DcMotor.class, "backright");
        rleftDrive = hardwareMap.get(DcMotor.class, "backleft");
        outtake = hardwareMap.dcMotor.get("outake2");
        too = hardwareMap.get(DcMotor.class, "too1");
        intake = hardwareMap.get(DcMotor.class, "intake");


        frightDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        rrightDrive.setDirection(DcMotorSimple.Direction.REVERSE);

        outtake.setDirection(DcMotor.Direction.FORWARD);
        too.setDirection(DcMotor.Direction.FORWARD);
        intake.setDirection(DcMotor.Direction.FORWARD);
        // Send telemetry message to signify robot waiting;

        fleftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rleftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rrightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        fleftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rleftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rrightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // Step through each leg of the path,
        // Note: Reverse movement is obtained by setting a negative distance (not speed)
        //move forward 5 feet
        encoderDrive(DRIVE_SPEED, 60, 60, 14.0);
        //turn left a little
        //turnWithGyro(-9.5, 0.5);
        //shoot the rings
        too.setPower(1.0);
        outtake.setPower(1.0);
        wait(3000);
        //stop the motors
        too.setPower(0.0);
        outtake.setPower(0.0);
        //forward 4 feet
        encoderDrive(DRIVE_SPEED, 48, 48, 11.0);
        //turn right 3 feet
        encoderDrive(TURN_SPEED, 36, -36, 9.0);
        //if time: pick up wobble goal
        //go back to beginning

    }

    public void encoderDrive(double speed,
                             double leftInches, double rightInches,
                             double timeoutS) {
        int newfrontLeftTarget;
        int newfrontRightTarget;
        int newbackRightTarget;
        int newbackLeftTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newfrontLeftTarget = fleftDrive.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            newfrontRightTarget = frightDrive.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
            newbackLeftTarget = rleftDrive.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            newbackRightTarget = rrightDrive.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
            fleftDrive.setTargetPosition(newfrontLeftTarget);
            frightDrive.setTargetPosition(newfrontRightTarget);
            rleftDrive.setTargetPosition(newbackLeftTarget);
            rrightDrive.setTargetPosition(newbackRightTarget);

            // Turn On RUN_TO_POSITION
            fleftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            frightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rleftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rrightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            fleftDrive.setPower(Math.abs(speed));
            frightDrive.setPower(Math.abs(speed));
            rleftDrive.setPower(Math.abs(speed));
            rrightDrive.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (fleftDrive.isBusy() && frightDrive.isBusy() && rleftDrive.isBusy() && rrightDrive.isBusy())) {

            }

            // Stop all motion;
            fleftDrive.setPower(0);
            frightDrive.setPower(0);
            rleftDrive.setPower(0);
            rrightDrive.setPower(0);

            // Turn off RUN_TO_POSITION
            fleftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            frightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rleftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rrightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }
    public void turnWithGyro(double degrees, double speedDirection){
        //Init
        angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double yaw = -angles.firstAngle;//make this negative
        telemetry.addData("Speed Direction", speedDirection);
        telemetry.addData("Yaw", yaw);
        telemetry.update();
        //
        telemetry.addData("stuff", speedDirection);
        telemetry.update();
        //
        double first;
        double second;
        //
        if (speedDirection > 0){//set target positions
            if (degrees > 10){
                first = (degrees - 10) + devertify(yaw);
                second = degrees + devertify(yaw);
            }else{
                first = devertify(yaw);
                second = degrees + devertify(yaw);
            }
        }else{
            if (degrees > 10){
                first = devertify(-(degrees - 10) + devertify(yaw));
                second = devertify(-degrees + devertify(yaw));
            }else{
                first = devertify(yaw);
                second = devertify(-degrees + devertify(yaw));
            }
            //
        }
        //
        Double firsta = convertify(first - 5);//175
        Double firstb = convertify(first + 5);//-175
        //
        turnWithEncoder(speedDirection);
        //
        if (Math.abs(firsta - firstb) < 11) {
            while (!(firsta < yaw && yaw < firstb) && opModeIsActive()) {//within range?
                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                gravity = imu.getGravity();
                yaw = -angles.firstAngle;
                telemetry.addData("Position", yaw);
                telemetry.addData("first before", first);
                telemetry.addData("first after", convertify(first));
                telemetry.update();
            }
        }else{
            while (!((firsta < yaw && yaw < 180) || (-180 < yaw && yaw < firstb)) && opModeIsActive()) {//within range?
                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                gravity = imu.getGravity();
                yaw = -angles.firstAngle;
                telemetry.addData("Position", yaw);
                telemetry.addData("first before", first);
                telemetry.addData("first after", convertify(first));
                telemetry.update();
            }
        }
        Double seconda = convertify(second - 5);//175
        Double secondb = convertify(second + 5);//-175
        //
        turnWithEncoder(speedDirection / 3);
        //
        if (Math.abs(seconda - secondb) < 11) {
            while (!(seconda < yaw && yaw < secondb) && opModeIsActive()) {//within range?
                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                gravity = imu.getGravity();
                yaw = -angles.firstAngle;
                telemetry.addData("Position", yaw);
                telemetry.addData("second before", second);
                telemetry.addData("second after", convertify(second));
                telemetry.update();
            }
            while (!((seconda < yaw && yaw < 180) || (-180 < yaw && yaw < secondb)) && opModeIsActive()) {//within range?
                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                gravity = imu.getGravity();
                yaw = -angles.firstAngle;
                telemetry.addData("Position", yaw);
                telemetry.addData("second before", second);
                telemetry.addData("second after", convertify(second));
                telemetry.update();
            }
            fleftDrive.setPower(0);
            frightDrive.setPower(0);
            rleftDrive.setPower(0);
            rrightDrive.setPower(0);
        }
        //
        fleftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rleftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rrightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fleftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rleftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rrightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    public double devertify(double degrees){
        if (degrees < 0){
            degrees = degrees + 360;
        }
        return degrees;
    }
    public double convertify(double degrees){
        if (degrees > 179){
            degrees = -(360 - degrees);
        } else if(degrees < -180){
            degrees = 360 + degrees;
        } else if(degrees > 360){
            degrees = degrees - 360;
        }
        return degrees;
    }
    //
    /*
    Init Gyro
    starts the imu gyro
     */
    public void initGyro(){
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
    }
    //
    /*
    This function is used in the turnWithGyro function to set the
    encoder mode and turn.
     */
    public void turnWithEncoder(double input){
        fleftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rleftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rrightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //
        fleftDrive.setPower(input);
        rleftDrive.setPower(input);
        frightDrive.setPower(-input);
        rrightDrive.setPower(-input);
    }

}

