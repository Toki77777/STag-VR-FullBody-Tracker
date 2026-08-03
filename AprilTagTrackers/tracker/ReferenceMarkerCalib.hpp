#pragma once

#include "config/ManualCalib.hpp"
#include "Helpers.hpp"

#include <opencv2/core.hpp>
#include <opencv2/core/quaternion.hpp>

#include <optional>
#include <vector>

namespace tracker
{

/// One synchronized observation of the reference board and of the HMD it is mounted on.
struct ReferenceMarkerSample
{
    /// Board pose in camera space, as returned by math::EstimatePoseTracker.
    Pose markerInCamera;
    /// HMD pose in playspace, as returned by IVRClient::GetHMDPose.
    Pose hmdInPlayspace;
};

/// The playspace transform, expressed exactly as tracker::PlayspaceCalib applies it:
///     posInPlayspace = rotation * (scale * CoordTransformOVR(posInCamera)) + posOffset
///     rotInPlayspace = rotation * CoordTransformOVR(rotInCamera)
/// Keeping one written down model here is what lets the solver below be checked against
/// the transform the tracking loop actually uses.
struct PlayspaceTransform
{
    cv::Matx33d rotation = cv::Matx33d::eye();
    cv::Vec3d posOffset{};
    double scale = 1.0;

    static PlayspaceTransform FromCalib(const cfg::ManualCalib::Real& calib);
    cfg::ManualCalib::Real ToCalib() const;

    Pose Apply(const Pose& poseInCamera) const;
    /// Inverse of the position part of Apply. Returns nullopt when scale is not usable.
    std::optional<cv::Vec3d> ApplyInversePos(const cv::Vec3d& posInPlayspace) const;
};

/// Change of basis between camera axes and playspace axes, negating x and z.
/// This is CoordTransformOVR for both components of a pose.
Pose ToPlayspaceBasis(Pose pose);

/// (pitch, yaw, roll) in radians such that EulerAnglesToRotationMatrix returns rot.
/// A rotation has two decompositions in this convention; this returns the one whose pitch
/// lies in [90, 270] degrees, which is the branch cfg::ManualCalib defaults to and the only
/// one PlayspaceCalibrator::Update accepts, so auto and manual calibration stay comparable.
cv::Vec3d PlayspaceAnglesFromRotation(const cv::Matx33d& rot);

/// Rotation angle between two orientations, in radians, always in [0, pi].
double RotationAngleBetween(const cv::Quatd& lhs, const cv::Quatd& rhs);

/// Acceptance limits of the offset solve. Defaults suit a user moving their head in front
/// of a webcam; tests tighten or loosen them to exercise individual rejections.
struct ReferenceMarkerCalibParams
{
    /// samples needed before the offset solve is attempted
    int minSamples = 6;
    /// relative rotations smaller than this carry no reliable axis and are discarded
    double minRelativeAngle = 5.0 * DEG_2_RAD;
    /// a pair whose HMD and board rotation angles disagree by more than this is an outlier
    double maxAngleMismatch = 5.0 * DEG_2_RAD;
    /// smallest second singular value, relative to the first, of the rotation fit.
    /// Rejects head motion about a single axis, which leaves the offset underdetermined.
    double minAxisSpread = 0.05;
    /// smallest accepted conditioning of the translation and scale system, as the ratio of
    /// its last to its first singular value. Guards against a rank deficient fit, which
    /// reproduces its own samples perfectly and is wrong everywhere else.
    double minTranslationConditioning = 1e-3;
    /// accepted range for the camera scale factor. Matches the range cfg::ManualCalib clamps
    /// to, so an accepted solution can be stored and read back unchanged. A camera whose
    /// scale lands outside it needs its intrinsics calibrated again, not a playspace fit.
    double minScale = 0.8;
    double maxScale = 1.2;
    /// largest accepted RMS error, in meters, between predicted and reported HMD position
    double maxPositionResidual = 0.05;
    /// largest accepted error, in radians, between predicted and reported HMD orientation
    double maxRotationResidual = 10.0 * DEG_2_RAD;
};

struct ReferenceMarkerCalibResult
{
    /// solved playspace calibration, ready for cfg::ManualCalib and PlayspaceCalib
    cfg::ManualCalib::Real playspace{};
    /// rigid offset such that markerInPlayspace = hmdInPlayspace * hmdToMarker
    Pose hmdToMarker = Pose::Ident();
    /// RMS distance, in meters, between predicted and reported HMD positions
    double positionResidual = 0;
    /// worst angle, in radians, between predicted and reported HMD orientations
    double rotationResidual = 0;
    int sampleCount = 0;
};

/// Solve the playspace transform together with the unknown offset between the HMD and the
/// board mounted on it, from observations taken while the user moved their head.
/// This is a hand-eye calibration: rotations come from a least squares fit over relative
/// motions, then scale and both translations fall out of one linear system.
/// Returns nullopt when the motion does not determine the answer or the fit does not
/// reproduce the observed HMD poses; the caller is expected to keep collecting.
std::optional<ReferenceMarkerCalibResult> SolveReferenceMarkerCalib(
    const std::vector<ReferenceMarkerSample>& samples,
    const ReferenceMarkerCalibParams& params = {});

/// Solve only the playspace transform, from a single observation and a known offset.
/// This is what determines the camera pose per frame once the offset is established.
std::optional<cfg::ManualCalib::Real> SolvePlayspaceFromSample(
    const ReferenceMarkerSample& sample, const Pose& hmdToMarker, double scale);

/// HMD pose implied by a board detection and a calibration. The difference against the pose
/// SteamVR reports is the residual that tells a systematic calibration error apart from
/// detection noise, so this is the primary diagnostic of the whole feature.
Pose PredictHmdPose(const cfg::ManualCalib::Real& playspace, const Pose& hmdToMarker,
                    const Pose& markerInCamera);

/// Board position in camera space implied by the HMD pose and a calibration.
/// Lets the reference board be searched for inside the existing detection mask instead of
/// over the whole frame. Returns nullopt when the calibration cannot be inverted.
std::optional<cv::Vec3d> PredictMarkerPosInCamera(
    const cfg::ManualCalib::Real& playspace, const Pose& hmdToMarker,
    const Pose& hmdInPlayspace);

/// Interpolate two playspace calibrations, t = 0 returns from and t = 1 returns to.
/// Orientation is interpolated as a rotation, so results stay correct across angle wrapping.
cfg::ManualCalib::Real BlendPlayspaceCalib(const cfg::ManualCalib::Real& from,
                                           const cfg::ManualCalib::Real& to, double t);

struct PlayspaceCalibDelta
{
    /// distance between the camera positions, in meters
    double position = 0;
    /// angle between the camera orientations, in radians
    double rotation = 0;
    /// absolute difference of the scale factors
    double scale = 0;
};
PlayspaceCalibDelta ComparePlayspaceCalib(const cfg::ManualCalib::Real& lhs,
                                          const cfg::ManualCalib::Real& rhs);

/// Turns a stream of per frame observations into a playspace calibration.
/// Holds no GUI, driver or config state: Update reports what should happen and the caller
/// applies it, which keeps every branch here reachable from tests.
class ReferenceMarkerCalibrator
{
public:
    enum class State
    {
        /// waiting for the board and the HMD to be seen at the same time
        WaitingForData,
        /// collecting head motion to solve the HMD to board offset
        CollectingSamples,
        /// offset known, camera pose is being solved per frame
        Calibrated
    };

    struct Options
    {
        /// keep re-solving the camera pose while the board stays visible
        bool continuous = true;
        /// weight of each new single frame solution, 1 replaces the calibration outright
        double smoothing = 0.05;
        /// minimum HMD motion between stored offset samples, so a still user does not fill
        /// the buffer with copies of one pose
        double sampleMinTranslation = 0.05;
        double sampleMinRotation = 8.0 * DEG_2_RAD;
        /// buffer size of the offset solve; the oldest sample is dropped past this
        int maxSamples = 40;
        /// a single frame solution further than this from the current calibration is held
        /// back as an outlier, and accepted only once it keeps repeating, which is how a
        /// bumped camera gets picked up without one bad detection moving the playspace
        double outlierPosition = 0.25;
        double outlierRotation = 15.0 * DEG_2_RAD;
        int outliersToAccept = 30;
        ReferenceMarkerCalibParams solver{};
    };

    struct UpdateOutcome
    {
        State state = State::WaitingForData;
        /// playspace calibration to apply this frame
        std::optional<cfg::ManualCalib::Real> calib{};
        /// set on the frame the offset was solved, so the caller can store and log it
        bool offsetSolved = false;
        /// stored offset samples, for progress reporting while collecting
        int sampleCount = 0;
        /// HMD position error, in meters, of the calibration in use
        std::optional<double> positionError{};
        /// HMD orientation error, in radians, of the calibration in use
        std::optional<double> rotationError{};
        /// a solution was withheld as an outlier this frame
        bool outlier = false;
        /// consecutive outliers seen so far
        int outlierCount = 0;
    };

    explicit ReferenceMarkerCalibrator(Options options = {}) : mOptions(options) {}

    /// Restore a previously solved offset, skipping the head motion collection.
    void SetOffset(const Pose& hmdToMarker, double scale);
    /// Drop the offset and every stored sample, returning to collection.
    void ResetOffset();
    /// Drop the solved camera pose but keep the offset, so the next detection re-solves it.
    void ResetPlayspace();

    UpdateOutcome Update(const std::optional<Pose>& hmdInPlayspace,
                         const std::optional<Pose>& markerInCamera);

    State GetState() const { return mState; }
    bool HasOffset() const { return mOffset.has_value(); }
    bool IsCalibrated() const { return mOffset.has_value() && mCalib.has_value(); }
    const std::optional<Pose>& GetOffset() const { return mOffset; }
    const std::optional<cfg::ManualCalib::Real>& GetCalib() const { return mCalib; }
    double GetScale() const { return mScale; }
    int GetSampleCount() const { return static_cast<int>(mSamples.size()); }
    const Options& GetOptions() const { return mOptions; }

private:
    bool ShouldStoreSample(const ReferenceMarkerSample& sample) const;
    void EvaluateResidual(const ReferenceMarkerSample& sample, UpdateOutcome& outcome) const;

    Options mOptions;
    State mState = State::WaitingForData;
    std::vector<ReferenceMarkerSample> mSamples{};
    std::optional<Pose> mOffset{};
    std::optional<cfg::ManualCalib::Real> mCalib{};
    double mScale = 1.0;
    int mOutlierCount = 0;
};

} // namespace tracker
