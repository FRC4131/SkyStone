package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;


@TeleOp(name = "Main TeleOp", group = "Iterative Opmode")

public class MainTeleOp extends OpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftBack = null;
    private DcMotor rightFront = null;
    private DcMotor leftFront = null;
    private DcMotor rightBack = null;
    private DcMotor arm = null;
    private Servo clamp = null;
    private boolean was2A = true;
    private boolean was2X = true;
    private Servo left;
    private Servo right;
    DigitalChannel digitalTouch;
    BNO055IMU imu;
    Orientation angles;
    Acceleration gravity;
    double startAngle;
    double targetAngle = 0;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = false;

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        leftFront = hardwareMap.get(DcMotor.class, "left_front");
        rightFront = hardwareMap.get(DcMotor.class, "right_front");
        leftBack = hardwareMap.get(DcMotor.class, "left_back");
        rightBack = hardwareMap.get(DcMotor.class, "right_back");
        arm = hardwareMap.get(DcMotor.class, "arm");
        clamp = hardwareMap.get(Servo.class, "clamp");
        left = hardwareMap.get(Servo.class, "left");
        right = hardwareMap.get(Servo.class, "right");
        // digitalTouch = hardwareMap.get(DigitalChannel.class, "sensor_digital");
        // digitalTouch.setMode(DigitalChannel.Mode.INPUT);
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        leftBack.setDirection(DcMotor.Direction.REVERSE);
        rightBack.setDirection(DcMotor.Direction.FORWARD);
        arm.setDirection(DcMotor.Direction.FORWARD);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


//        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        arm.setTargetPosition(-100);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        clamp.setPosition(0);

        right.setDirection(Servo.Direction.REVERSE);

        right.scaleRange(0,0.25);
        left.scaleRange(0.7,1);



        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {

    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        runtime.reset();
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        startAngle = angles.firstAngle;
        left.setPosition(0);
        right.setPosition(0);
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {


        // arm
        arm.setPower(0.6);

        if (gamepad2.x && !was2X) {
            if (arm.getTargetPosition() == -2550) { // arm is down
                arm.setTargetPosition(-2200); // set to middle
            } else {
                arm.setTargetPosition(-2550); // set to down
            }
        }

        if(gamepad2.y){
            arm.setTargetPosition(-100); // arm is up (all the way back)
        }


        if (gamepad2.a && !was2A) {
            if (clamp.getPosition() == 0) {
                clamp.setPosition(1);
            } else {
                clamp.setPosition(0);
            }
        }

        if (gamepad2.start) {
            left.setPosition(0);
            right.setPosition(0);
        }
        if (gamepad2.back) {
            left.setPosition(1);
            right.setPosition(1);
        }


        was2A = gamepad2.a;
        was2X = gamepad2.x;


        double x = gamepad1.left_stick_x;
        double y = -gamepad1.left_stick_y;

//        x = x * x * Math.signum(x);
//        y = y * y * Math.signum(y);

        // double rotation = gamepad1.right_stick_x;

        targetAngle += gamepad1.right_stick_x;

        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        gravity = imu.getGravity();

        double c = angles.firstAngle - startAngle;
        double d = Math.toRadians(-c);

        double angleDifference = c - targetAngle;

        while (angleDifference < -180) {
            angleDifference += 360;
        }

        while (angleDifference > 180) {
            angleDifference -= 360;
        }

        double rotation = angleDifference * 0.01;
//        double rotation = gamepad1.right_stick_x;

        double forward = x * Math.sin(d) + y * Math.cos(d);
        double sideways = x * Math.cos(d) - y * Math.sin(d);

        double leftBackPower = Range.clip(forward - sideways + rotation, -1, 1);
        double rightFrontPower = Range.clip(forward - sideways - rotation, -1, 1);
        double leftFrontPower = Range.clip(forward + sideways + rotation, -1, 1);
        double rightBackPower = Range.clip(forward + sideways - rotation, -1, 1);

        leftBack.setPower(leftBackPower);
        rightBack.setPower(rightBackPower);
        leftFront.setPower(leftFrontPower);
        rightFront.setPower(rightFrontPower);

        telemetry.addData("lf", leftFront.getCurrentPosition());
        telemetry.addData("rf", rightFront.getCurrentPosition());
        telemetry.addData("lb", leftBack.getCurrentPosition());
        telemetry.addData("rb", rightBack.getCurrentPosition());
        telemetry.update();
    }


    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

}


