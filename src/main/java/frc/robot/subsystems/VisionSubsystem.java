package frc.robot.subsystems;

import java.util.List;
import java.util.Optional;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.commands.vision.SupplyAprilTagPose;


/* Make sure that 
 * • the correct camera resolution is selected 
 * • the targets are sorted by closest to farthest distance
 * on https://photonvision.local:5800 
*/

public class VisionSubsystem extends SubsystemBase {
    private PhotonCamera apriltagCamera;
    private PhotonPoseEstimator poseEstimator;
    private Pose2d targetPose;

    public VisionSystemSim visionSim;
    public PhotonCameraSim cameraSim;

    private static final List<Integer> BLUE_APRILTAG_IDS = List.of(7, 6, 1, 2, 14, 15, 16);
    private static final List<Integer> RED_APRILTAG_IDS = List.of(4, 5, 9, 10, 11, 12, 13);
    
    /* list of fiducial ids to look for depending on alliance */
    private List<Integer> targetFiducialIds = List.of(4);

    // public Transform3d robotToCamera = new Transform3d(
    //     new Translation3d(
    //       Units.inchesToMeters(14.5),
    //       Units.inchesToMeters(0),
    //       Units.inchesToMeters(13.5)
    //     ),
    //     new Rotation3d(0, 0, 0)
    // );

    public VisionSubsystem() {
        apriltagCamera = new PhotonCamera("Limelight1");
        // poseEstimator = new PhotonPoseEstimator(
        //     Constants.Vision.aprilTagFieldLayout,
        //     PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
        //     apriltagCamera,
        //     Constants.Vision.robotToCamera
        // );

        refreshTargetFiducialIds();
        // System.out.println("Targetting fiducial ids: " + targetFiducialIds);

        initSimulation();
    }

    public List<Integer> refreshTargetFiducialIds() {
        DriverStation.getAlliance().map(alliance -> {
            if (alliance == DriverStation.Alliance.Blue) { targetFiducialIds = BLUE_APRILTAG_IDS; }
            else if (alliance == DriverStation.Alliance.Red) { targetFiducialIds = RED_APRILTAG_IDS; }
            return null;
        });

        return targetFiducialIds;
    }

    /**
     * Gets the closest April Tag that matches any id in the list of targetFiducialIds
     *  if and only if its ambiguity < 0.2
     * 
     * @return Optional<PhotonTrackedTarget>: The closest April Tag that matches a target fiducial id if its ambiguity < 0.2
     */
    public Optional<PhotonTrackedTarget> getAprilTag() {
        refreshTargetFiducialIds();
        
        var result = apriltagCamera.getAllUnreadResults();
        List<PhotonTrackedTarget> targets = result.isEmpty() ? List.of() : result.get(0).getTargets();
        Optional<PhotonTrackedTarget> target = Optional.empty();

        /* get closest target that matches any targetFiducialId */
        for (PhotonTrackedTarget potentialTarget: targets) {
            if (targetFiducialIds.contains(potentialTarget.getFiducialId())) {
                target = Optional.of(potentialTarget);
                break;
            }
        }

        /* return target only if ambiguity < 0.2 */
        return target.map(e -> 
            e.getPoseAmbiguity() < 0.2 ? e : null
        );
    }

    /** 
     * @return the Pose2d from robot center to AprilTag target
     */
    public Pose2d getPoseTo(PhotonTrackedTarget target) {
        Transform3d transform = target.getBestCameraToTarget();
        Translation2d end = transform.getTranslation().toTranslation2d()
            .plus(Constants.Vision.robotToCamera.getTranslation().toTranslation2d());

        double zAngleTheta = transform.getRotation().getZ();
        Rotation2d yaw = Rotation2d.fromRadians(Math.signum(zAngleTheta) * (Math.PI - Math.abs(zAngleTheta))).unaryMinus();

        return new Pose2d(end, yaw);
    }

    public void printAllResults() {
        System.out.println("Vision log:");
        var result = apriltagCamera.getLatestResult();

        if (!result.hasTargets()) {
            System.out.println("> No targets found.");
            return;
        }

        System.out.println("> Single AprilTag: " + getAprilTag());
    }

    /** 
     * Can be accessed at http://localhost:1182 by default.
     */
    public void initSimulation() {
        if (!Robot.isSimulation()) { return; } 

        // AprilTagFieldLayout fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape);

        // visionSim = new VisionSystemSim("Vision");
        // visionSim.addAprilTags(fieldLayout);

        // cameraSim = new PhotonCameraSim(apriltagCamera, Constants.Vision.cameraProperties);
        // cameraSim.enableDrawWireframe(true);

        // visionSim.addCamera(cameraSim, Constants.Vision.robotToCamera);

        // System.out.println("Camera sim: " + cameraSim);

        // SmartDashboard.putData("Field", visionSim.getDebugField());
    }

    public void updateSimulation(SwerveSubsystem swerve) {
        if (!Robot.isSimulation()) { return; } 

        // visionSim.update(swerve.getPose());
    }
}