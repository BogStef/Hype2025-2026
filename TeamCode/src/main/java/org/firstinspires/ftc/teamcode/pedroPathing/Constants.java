package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.revhub.drivetrains.MecanumConfig;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.localizers.PinpointConfig;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.pedropathing.controllers.Controller;
import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.algorithm.Foresight;
import com.pedropathing.math.Matrix;
import com.pedropathing.math.Vector2D;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Constants {
    //public static PinpointConfig localizerConfig;
    public static MecanumConfig drivetrainConfig = new MecanumConfig(c -> {
        c.frontLeftName.set("stft");
        c.frontRightName.set("drft");
        c.backLeftName.set("stspa");
        c.backRightName.set("drspa");
        c.frontLeftDirection.set(DcMotorSimple.Direction.REVERSE);
        c.frontRightDirection.set(DcMotorSimple.Direction.FORWARD);
        c.backLeftDirection.set(DcMotorSimple.Direction.FORWARD);
        c.backRightDirection.set(DcMotorSimple.Direction.REVERSE);
    });
    public static PinpointConfig localizerConfig = new PinpointConfig(c -> {
        c.name.set("pinpoint");
        c.podType.set(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        c.xPodOffset.set(-2.85465);
        c.yPodOffset.set(-6.29921);
        c.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
        c.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
        c.globalDistanceUnit.set(DistanceUnit.INCH);
        c.offsetUnits.set(DistanceUnit.INCH);
    });
    public static ForesightConfig foresightConfig = new ForesightConfig(
            c -> {
                Controller primaryTranslationalForward = Controller.proportional(0.3476907000243875);
                Controller secondaryTranslationalForward = Controller.proportional(0.12846242319240156);
                Controller primaryTranslationalLateral = Controller.proportional(0.5662940883597831);
                Controller secondaryTranslationalLateral = Controller.proportional(0.2092305339922152);

                c.forwardTranslational.set(Controller.piecewise(secondaryTranslationalForward).put(2.5, primaryTranslationalForward));
                c.strafeTranslational.set(Controller.piecewise(secondaryTranslationalLateral).put(2.5, primaryTranslationalLateral));

                c.coast.set(Controller.proportionalFeedforward(0.014086354685602659));
                c.brake.set(Controller.proportionalFeedforward(0.011973401482762259));

                c.headingFeedback.set(Controller.proportional(5.362065585839823));
                c.headingBrakeCoefficients.set(Vector2D.cartesian(0.033892218938168205, 0.011809896780057854));

                c.linearBrakeCoefficients.set(Matrix.diag(0.04778731916824289, 0.038118283593194614));
                c.quadraticBrakeCoefficients.set(Matrix.diag(0.002663656191476484, 0.0026553096390399067));

                c.maxAchievableForwardVelocity.set(73.1287928508127);
                c.maxAchievableStrafeVelocity.set(52.44521485210474);
                c.naturalForwardDeceleration.set(35.61405930312165);
                c.naturalStrafeDeceleration.set(83.28663742187202);
            }
    );
    public static Follower create(HardwareMap h) {
        return new Follower(
                new PinpointLocalizer(h, localizerConfig),
                new Mecanum(h, drivetrainConfig),
                new Foresight(foresightConfig)
        );
    }
}