package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

@Autonomous(name = "Blue Vision_")
public class FoundationBlueVision extends EncoderDrive {

    private DcMotor arm = null;

    private Servo clamp = null;


    VuforiaLocalizer vuforia;
    VuforiaTrackables trackables;
    VuforiaTrackable template;
    VuforiaTrackableDefaultListener mark;


    @Override
    public void runOpMode() {

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        telemetry.addData("Status", "Resetting Encoders");    //
        telemetry.update();

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = false;

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);


        leftFront  = hardwareMap.get(DcMotor.class, "left_front");
        rightFront = hardwareMap.get(DcMotor.class, "right_front");
        leftBack = hardwareMap.get(DcMotor.class, "left_back");
        rightBack = hardwareMap.get(DcMotor.class, "right_back");

        arm = hardwareMap.get(DcMotor.class, "arm");

        clamp = hardwareMap.get(Servo.class, "clamp");

        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        right = hardwareMap.get(Servo.class, "right");
        left = hardwareMap.get(Servo.class, "left");
        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        leftBack.setDirection(DcMotor.Direction.REVERSE);
        rightBack.setDirection(DcMotor.Direction.FORWARD);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        digitalTouch = hardwareMap.get(DigitalChannel.class, "sensor_digital");
        digitalTouch.setMode(DigitalChannel.Mode.INPUT);

        right.setDirection(Servo.Direction.REVERSE);

        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        arm.setTargetPosition(-100);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        clamp.setPosition(0);

        right.setDirection(Servo.Direction.REVERSE);

        right.scaleRange(0.125, 0.25);
        left.scaleRange(0.7, 0.85);

        initVuforia();

        // Wait for the game to start (driver presses PLAY)rotateToAngle
        waitForStart();
        servo(0);
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        startAngle = angles.firstAngle;
        encoderDrive(0.3, 19,19,5 );
        arm.setPower(0.5);

        int counter = 0;


        while (!markVisible()) {
            encoderSideways(0.35,7, 1);
            sleep(100);
            counter += 1;
        }

        double target = 90;

        while (Math.abs(target - markPos()) > 5) {
            double power = (target - markPos()) * 0.003;

            leftFront.setPower(power);
            rightFront.setPower(-power);
            leftBack.setPower(-power);
            rightBack.setPower(power);
        }

        arm.setTargetPosition(-2575);
        clamp.setPosition(1);
        sleep(1000);

        encoderDrive(0.15, 10, 10, 4);
        sleep(100);

        clamp.setPosition(0);
        sleep(900);

        arm.setTargetPosition(-100);
        encoderDrive(0.5,-4,-4,3);
        encoderSideways(0.7,counter*-7,5);
        rotateToAngle(0,0.7);
        encoderSideways(0.9,-72,10); // cross line
        rotateToAngle(0, 0.7);
        encoderSideways(0.5,-32,5); // aligned to wall
        encoderSideways(0.5,18,5);
        rotateToAngle(0, 0.7);
        touchSensor(0.3);
        servo(1); //servo down
        arm.setTargetPosition(-2575);
        encoderDrive(0.8,-50,-50,5); // pull foundation
        clamp.setPosition(1);
        arm.setTargetPosition(-100);
        servo(0); // servo up
        encoderSideways(0.8,57,5); // park
        arm.setTargetPosition(-30);

    }


    private void initVuforia() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
//        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey =
                "AeTSKkn/////AAABmXK540vJ4k8muhR6D7aoeZpbnFSenqf9a+poNXj4KY56UyTsbTrSeHqrNBi7hJweC+rEjfGiSPfJ813Az57QwdTyLzth/JgNKh3BfGz7OcgIaqCMLwDZf+BAEjFYuX2j5bKUNN/+kCrWp8AUvbPpcPHmGnnwJ7ABzmsazba+tMgSs3rcA3AvezaOGOMDwIiG71ouwN3mKOvybsDNf+2jzMCn1tywUqn3teDCGzKjV2ZeqJW9Qt2wjrvY2sI3MS176buUh/H04Da5FDj+6Dg3/fZtsIlsrVu2fAvepwWvgiCprGf6i9Q4oLBryNgCLDfpMx9EXBWAE4D5hzyBsoFITU0z6zUO+e8b6rJ0xQQr7dbV";
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;

        vuforia = ClassFactory.getInstance().createVuforia(parameters);
        trackables = this.vuforia.loadTrackablesFromAsset("Skystone");
        template = trackables.get(0);

        mark = (VuforiaTrackableDefaultListener) template.getListener();
        trackables.activate();
    }

    private boolean markVisible() {
        return mark.isVisible();
    }

    private double markPos() {
        OpenGLMatrix pose = mark.getPose();

        VectorF trans = pose.getTranslation();
        Orientation rot = Orientation.getOrientation(pose, AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);

        double tX = trans.get(0);
        double tY = trans.get(1);
        double tZ = trans.get(2);

        double rX = rot.firstAngle;
        double rY = rot.secondAngle;
        double rZ = rot.thirdAngle;

        return tX;
    }
}

