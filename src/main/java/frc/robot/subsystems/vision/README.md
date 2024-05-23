
https://github.com/Mechanical-Advantage/RobotCode2022/tree/422866cc2bca6434aeef52874537bddad0f9a7f2/src/main/java/frc/robot/subsystems/vision


https://github.com/FRC5188/CrescendoCode/blob/42d35c077cd77c97b7812a691fc1ee1a9a175268/src/main/java/frc/robot/hardware/vision/RealVisionIO.java#L4
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
private static final AprilTagFieldLayout FIELD_LAYOUT = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
private static final PhotonPoseEstimator[] ESTIMATORS = createPoseEstimators();

        private static PhotonPoseEstimator[] createPoseEstimators() {
                final PhotonPoseEstimator[] estimators = new PhotonPoseEstimator[HardwareConstants.NUMBER_OF_CAMERAS];
                for (int i = 0; i < HardwareConstants.NUMBER_OF_CAMERAS; i++) {
                        estimators[i] = new PhotonPoseEstimator(
                                FIELD_LAYOUT,
                                PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
                                new PhotonCamera("photoncamera_" + (i + 1)),
                                HardwareConstants.ComponentTransformations._cameraPosition[i]);

                        estimators[i].setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
                }
                return estimators;
        }
