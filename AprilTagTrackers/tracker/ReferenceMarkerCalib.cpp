#include "ReferenceMarkerCalib.hpp"

#include "PlayspaceCalib.hpp"
#include "utils/Assert.hpp"
#include "utils/Test.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>

namespace tracker
{

namespace
{

constexpr double TWO_PI = 2.0 * PI;

double WrapToPi(double angle)
{
    while (angle > PI) angle -= TWO_PI;
    while (angle <= -PI) angle += TWO_PI;
    return angle;
}

cv::Vec3d ToVec(const cv::Point3d& point) { return {point.x, point.y, point.z}; }

cv::Vec3d RotateByQuat(const cv::Quatd& rot, const cv::Vec3d& vec)
{
    return rot.toRotMat3x3(cv::QUAT_ASSUME_UNIT) * vec;
}

/// Rotation vector of rot, always taking the shorter of the two equivalent turns, so that
/// the axis and the angle of two rotations describing the same motion can be compared.
cv::Vec3d ShortestRotVec(cv::Quatd rot)
{
    rot = rot.normalize();
    if (rot.w < 0) rot = -rot;
    return rot.toRotVec(cv::QUAT_ASSUME_UNIT);
}

/// Rotation minimizing the squared error of rot * from[i] - to[i], by SVD.
/// minAxisSpread rejects inputs spanning a single direction, where the rotation about that
/// direction is not determined and any answer fits the input equally well.
std::optional<cv::Matx33d> SolveRotationFromDirections(
    const std::vector<cv::Vec3d>& from, const std::vector<cv::Vec3d>& to, double minAxisSpread)
{
    ATT_ASSERT(from.size() == to.size());
    if (from.size() < 2) return std::nullopt;

    cv::Matx33d covariance = cv::Matx33d::zeros();
    for (std::size_t index = 0; index < from.size(); ++index)
    {
        for (int row = 0; row < 3; ++row)
        {
            for (int col = 0; col < 3; ++col)
            {
                covariance(row, col) += to[index][row] * from[index][col];
            }
        }
    }

    cv::Mat singular;
    cv::Mat leftVec;
    cv::Mat rightVecT;
    cv::SVD::compute(cv::Mat(covariance), singular, leftVec, rightVecT, cv::SVD::FULL_UV);
    const double firstSingular = singular.at<double>(0);
    const double secondSingular = singular.at<double>(1);
    if (firstSingular <= 0 || secondSingular < minAxisSpread * firstSingular) return std::nullopt;

    cv::Mat properRotation = cv::Mat::eye(3, 3, CV_64F);
    if (cv::determinant(leftVec * rightVecT) < 0) properRotation.at<double>(2, 2) = -1.0;
    const cv::Mat rotation = leftVec * properRotation * rightVecT;
    return cv::Matx33d(rotation.ptr<double>());
}

/// Fit the HMD to board rotation from relative motions.
/// Between two samples the same physical turn is seen twice, as hmdRel in playspace and as
/// boardRel on the board, related by hmdRel = offset * boardRel * offset^-1. Conjugation
/// keeps the angle and rotates the axis, so the offset is the one rotation mapping every
/// board rotation axis onto the matching HMD rotation axis.
std::optional<cv::Matx33d> SolveHmdToMarkerRotation(
    const std::vector<ReferenceMarkerSample>& samples, const ReferenceMarkerCalibParams& params)
{
    std::vector<cv::Vec3d> boardAxes;
    std::vector<cv::Vec3d> hmdAxes;
    for (std::size_t lhs = 0; lhs + 1 < samples.size(); ++lhs)
    {
        const cv::Quatd lhsHmd = samples[lhs].hmdInPlayspace.rotation.inv(cv::QUAT_ASSUME_UNIT);
        const cv::Quatd lhsBoard = ToPlayspaceBasis(samples[lhs].markerInCamera).rotation.inv(cv::QUAT_ASSUME_UNIT);
        for (std::size_t rhs = lhs + 1; rhs < samples.size(); ++rhs)
        {
            const cv::Vec3d hmdAxis = ShortestRotVec(lhsHmd * samples[rhs].hmdInPlayspace.rotation);
            const cv::Vec3d boardAxis = ShortestRotVec(
                lhsBoard * ToPlayspaceBasis(samples[rhs].markerInCamera).rotation);

            const double hmdAngle = cv::norm(hmdAxis);
            if (hmdAngle < params.minRelativeAngle) continue;
            if (std::abs(hmdAngle - cv::norm(boardAxis)) > params.maxAngleMismatch) continue;
            hmdAxes.push_back(hmdAxis);
            boardAxes.push_back(boardAxis);
        }
    }
    return SolveRotationFromDirections(boardAxes, hmdAxes, params.minAxisSpread);
}

/// Camera orientation implied by each sample once the HMD to board rotation is known.
/// The estimates differ only by observation noise, so the sign aligned mean is enough.
cv::Quatd SolvePlayspaceRotation(const std::vector<ReferenceMarkerSample>& samples,
                                 const cv::Quatd& hmdToMarkerRot)
{
    ATT_ASSERT(!samples.empty());
    cv::Quatd sum{0, 0, 0, 0};
    cv::Quatd reference{1, 0, 0, 0};
    for (std::size_t index = 0; index < samples.size(); ++index)
    {
        cv::Quatd estimate = samples[index].hmdInPlayspace.rotation * hmdToMarkerRot *
                             ToPlayspaceBasis(samples[index].markerInCamera).rotation.inv(cv::QUAT_ASSUME_UNIT);
        estimate = estimate.normalize();
        if (index == 0)
        {
            reference = estimate;
        }
        else if (estimate.dot(reference) < 0)
        {
            estimate = -estimate;
        }
        sum = sum + estimate;
    }
    return sum.normalize();
}

struct TranslationSolution
{
    double scale = 1.0;
    cv::Vec3d posOffset{};
    cv::Vec3d hmdToMarkerPos{};
};

/// Solve scale, camera position and the HMD to board translation in one least squares pass.
///     scale * playspaceRot * basisChanged(boardPos) + posOffset = hmdPos + hmdRot * offsetPos
/// is linear in all seven unknowns. They are only separable when the HMD orientation varies
/// across the samples, so the conditioning of the system is checked rather than trusted:
/// a rank deficient fit still reproduces its own samples and would pass the residual check
/// while being wrong for every pose that was not measured.
std::optional<TranslationSolution> SolveTranslationAndScale(
    const std::vector<ReferenceMarkerSample>& samples, const cv::Matx33d& playspaceRot,
    double minConditioning)
{
    constexpr int numUnknowns = 7;
    const int rows = static_cast<int>(samples.size()) * 3;
    if (rows < numUnknowns + 2) return std::nullopt;

    cv::Mat lhs = cv::Mat::zeros(rows, numUnknowns, CV_64F);
    cv::Mat rhs = cv::Mat::zeros(rows, 1, CV_64F);
    for (std::size_t index = 0; index < samples.size(); ++index)
    {
        const cv::Vec3d boardPos =
            playspaceRot * ToVec(ToPlayspaceBasis(samples[index].markerInCamera).position);
        const cv::Matx33d hmdRot =
            samples[index].hmdInPlayspace.rotation.toRotMat3x3(cv::QUAT_ASSUME_UNIT);
        const cv::Vec3d hmdPos = ToVec(samples[index].hmdInPlayspace.position);

        for (int axis = 0; axis < 3; ++axis)
        {
            const int row = (static_cast<int>(index) * 3) + axis;
            lhs.at<double>(row, 0) = boardPos[axis];
            lhs.at<double>(row, 1 + axis) = 1.0;
            for (int col = 0; col < 3; ++col)
            {
                lhs.at<double>(row, 4 + col) = -hmdRot(axis, col);
            }
            rhs.at<double>(row, 0) = hmdPos[axis];
        }
    }

    cv::Mat singular;
    cv::SVD::compute(lhs, singular, cv::SVD::NO_UV);
    const double firstSingular = singular.at<double>(0);
    const double lastSingular = singular.at<double>(numUnknowns - 1);
    if (firstSingular <= 0 || lastSingular < minConditioning * firstSingular) return std::nullopt;

    cv::Mat solution;
    if (!cv::solve(lhs, rhs, solution, cv::DECOMP_SVD)) return std::nullopt;

    TranslationSolution result;
    result.scale = solution.at<double>(0);
    result.posOffset = {solution.at<double>(1), solution.at<double>(2), solution.at<double>(3)};
    result.hmdToMarkerPos = {solution.at<double>(4), solution.at<double>(5), solution.at<double>(6)};
    return result;
}

} // namespace

PlayspaceTransform PlayspaceTransform::FromCalib(const cfg::ManualCalib::Real& calib)
{
    return {EulerAnglesToRotationMatrix(calib.angleOffset), calib.posOffset, calib.scale};
}

cfg::ManualCalib::Real PlayspaceTransform::ToCalib() const
{
    return {posOffset, PlayspaceAnglesFromRotation(rotation), scale};
}

Pose PlayspaceTransform::Apply(const Pose& poseInCamera) const
{
    const Pose inBasis = ToPlayspaceBasis(poseInCamera);
    const cv::Vec3d position = (rotation * (scale * ToVec(inBasis.position))) + posOffset;
    const cv::Quatd orientation =
        (cv::Quatd::createFromRotMat(rotation).normalize() * inBasis.rotation).normalize();
    return Pose{cv::Point3d(position), orientation};
}

std::optional<cv::Vec3d> PlayspaceTransform::ApplyInversePos(const cv::Vec3d& posInPlayspace) const
{
    constexpr double minScale = 1e-9;
    if (std::abs(scale) < minScale) return std::nullopt;
    cv::Vec3d position = (rotation.t() * (posInPlayspace - posOffset)) / scale;
    CoordTransformOVR(position);
    return position;
}

Pose ToPlayspaceBasis(Pose pose)
{
    CoordTransformOVR(pose.position);
    CoordTransformOVR(pose.rotation);
    return pose;
}

cv::Vec3d PlayspaceAnglesFromRotation(const cv::Matx33d& rot)
{
    // EulerAnglesToRotationMatrix builds Ry(yaw) * Rx(pitch) * Rz(roll), which gives
    //     rot(1, 2) = -sin(pitch), rot(0, 2) / rot(2, 2) = tan(yaw),
    //     rot(1, 0) / rot(1, 1) = tan(roll)
    // The asin branch below returns the decomposition with cos(pitch) >= 0; the equivalent
    // one is (pi - pitch, yaw + pi, roll + pi). Return that second branch, because it is the
    // one whose pitch lands in [90, 270] degrees, the range cfg::ManualCalib defaults into
    // and the only range PlayspaceCalibrator::Update leaves unclamped.
    const double pitch = std::asin(std::clamp(-rot(1, 2), -1.0, 1.0));
    const double yaw = std::atan2(rot(0, 2), rot(2, 2));
    const double roll = std::atan2(rot(1, 0), rot(1, 1));
    return {PI - pitch, WrapToPi(yaw + PI), WrapToPi(roll + PI)};
}

double RotationAngleBetween(const cv::Quatd& lhs, const cv::Quatd& rhs)
{
    const double dot = std::abs(lhs.normalize().dot(rhs.normalize()));
    return 2.0 * std::acos(std::clamp(dot, 0.0, 1.0));
}

std::optional<ReferenceMarkerCalibResult> SolveReferenceMarkerCalib(
    const std::vector<ReferenceMarkerSample>& samples, const ReferenceMarkerCalibParams& params)
{
    if (static_cast<int>(samples.size()) < params.minSamples) return std::nullopt;

    const auto rotationFit = SolveHmdToMarkerRotation(samples, params);
    if (!rotationFit) return std::nullopt;

    const cv::Quatd hmdToMarkerRot = cv::Quatd::createFromRotMat(*rotationFit).normalize();
    const cv::Quatd playspaceRotQuat = SolvePlayspaceRotation(samples, hmdToMarkerRot);
    const cv::Matx33d playspaceRot = playspaceRotQuat.toRotMat3x3(cv::QUAT_ASSUME_UNIT);

    const auto translation =
        SolveTranslationAndScale(samples, playspaceRot, params.minTranslationConditioning);
    if (!translation) return std::nullopt;
    if (translation->scale < params.minScale || translation->scale > params.maxScale) return std::nullopt;

    ReferenceMarkerCalibResult result;
    result.playspace = {translation->posOffset, PlayspaceAnglesFromRotation(playspaceRot), translation->scale};
    result.hmdToMarker = Pose{cv::Point3d(translation->hmdToMarkerPos), hmdToMarkerRot};
    result.sampleCount = static_cast<int>(samples.size());

    // Score the fit by how well it reproduces the HMD poses that went into it. This runs
    // through PredictHmdPose, so the angles written above are decoded again on the way,
    // and a mistake in that conversion cannot pass unnoticed.
    double squaredSum = 0;
    for (const auto& sample : samples)
    {
        const Pose predicted = PredictHmdPose(result.playspace, result.hmdToMarker, sample.markerInCamera);
        const double distance = cv::norm(ToVec(predicted.position) - ToVec(sample.hmdInPlayspace.position));
        squaredSum += distance * distance;
        result.rotationResidual = std::max(
            result.rotationResidual,
            RotationAngleBetween(predicted.rotation, sample.hmdInPlayspace.rotation));
    }
    result.positionResidual = std::sqrt(squaredSum / static_cast<double>(samples.size()));

    if (result.positionResidual > params.maxPositionResidual) return std::nullopt;
    if (result.rotationResidual > params.maxRotationResidual) return std::nullopt;
    return result;
}

std::optional<cfg::ManualCalib::Real> SolvePlayspaceFromSample(
    const ReferenceMarkerSample& sample, const Pose& hmdToMarker, double scale)
{
    if (!(scale > 0)) return std::nullopt;

    // where the board has to be, going by the HMD and the known mount
    const cv::Quatd markerRot = (sample.hmdInPlayspace.rotation * hmdToMarker.rotation).normalize();
    const cv::Vec3d markerPos = ToVec(sample.hmdInPlayspace.position) +
                                RotateByQuat(sample.hmdInPlayspace.rotation, ToVec(hmdToMarker.position));

    // and where the camera reports it
    const Pose inBasis = ToPlayspaceBasis(sample.markerInCamera);
    const cv::Quatd playspaceRotQuat =
        (markerRot * inBasis.rotation.normalize().inv(cv::QUAT_ASSUME_UNIT)).normalize();
    const cv::Matx33d playspaceRot = playspaceRotQuat.toRotMat3x3(cv::QUAT_ASSUME_UNIT);
    const cv::Vec3d posOffset = markerPos - (playspaceRot * (scale * ToVec(inBasis.position)));

    return cfg::ManualCalib::Real{posOffset, PlayspaceAnglesFromRotation(playspaceRot), scale};
}

Pose PredictHmdPose(const cfg::ManualCalib::Real& playspace, const Pose& hmdToMarker,
                    const Pose& markerInCamera)
{
    const Pose markerInPlayspace = PlayspaceTransform::FromCalib(playspace).Apply(markerInCamera);
    const cv::Quatd hmdRot =
        (markerInPlayspace.rotation * hmdToMarker.rotation.normalize().inv(cv::QUAT_ASSUME_UNIT)).normalize();
    const cv::Vec3d hmdPos =
        ToVec(markerInPlayspace.position) - RotateByQuat(hmdRot, ToVec(hmdToMarker.position));
    return Pose{cv::Point3d(hmdPos), hmdRot};
}

std::optional<cv::Vec3d> PredictMarkerPosInCamera(
    const cfg::ManualCalib::Real& playspace, const Pose& hmdToMarker, const Pose& hmdInPlayspace)
{
    const cv::Vec3d markerPos = ToVec(hmdInPlayspace.position) +
                                RotateByQuat(hmdInPlayspace.rotation, ToVec(hmdToMarker.position));
    return PlayspaceTransform::FromCalib(playspace).ApplyInversePos(markerPos);
}

cfg::ManualCalib::Real BlendPlayspaceCalib(const cfg::ManualCalib::Real& from,
                                           const cfg::ManualCalib::Real& to, double t)
{
    const double weight = std::clamp(t, 0.0, 1.0);
    const cv::Quatd fromRot =
        cv::Quatd::createFromRotMat(EulerAnglesToRotationMatrix(from.angleOffset)).normalize();
    const cv::Quatd toRot =
        cv::Quatd::createFromRotMat(EulerAnglesToRotationMatrix(to.angleOffset)).normalize();
    const cv::Quatd blended = cv::Quatd::slerp(fromRot, toRot, weight, cv::QUAT_ASSUME_UNIT);

    return {(from.posOffset * (1.0 - weight)) + (to.posOffset * weight),
            PlayspaceAnglesFromRotation(blended.toRotMat3x3(cv::QUAT_ASSUME_UNIT)),
            (from.scale * (1.0 - weight)) + (to.scale * weight)};
}

PlayspaceCalibDelta ComparePlayspaceCalib(const cfg::ManualCalib::Real& lhs,
                                          const cfg::ManualCalib::Real& rhs)
{
    const cv::Quatd lhsRot =
        cv::Quatd::createFromRotMat(EulerAnglesToRotationMatrix(lhs.angleOffset)).normalize();
    const cv::Quatd rhsRot =
        cv::Quatd::createFromRotMat(EulerAnglesToRotationMatrix(rhs.angleOffset)).normalize();
    return {cv::norm(lhs.posOffset - rhs.posOffset),
            RotationAngleBetween(lhsRot, rhsRot),
            std::abs(lhs.scale - rhs.scale)};
}

void ReferenceMarkerCalibrator::SetOffset(const Pose& hmdToMarker, double scale)
{
    mOffset = hmdToMarker;
    mScale = scale;
    mSamples.clear();
    mCalib.reset();
    mOutlierCount = 0;
    mState = State::WaitingForData;
}

void ReferenceMarkerCalibrator::ResetOffset()
{
    mOffset.reset();
    mSamples.clear();
    mCalib.reset();
    mOutlierCount = 0;
    mState = State::WaitingForData;
}

void ReferenceMarkerCalibrator::ResetPlayspace()
{
    mCalib.reset();
    mOutlierCount = 0;
    if (mState == State::Calibrated) mState = State::WaitingForData;
}

bool ReferenceMarkerCalibrator::ShouldStoreSample(const ReferenceMarkerSample& sample) const
{
    if (mSamples.empty()) return true;
    const auto& last = mSamples.back();
    const double moved =
        cv::norm(ToVec(sample.hmdInPlayspace.position) - ToVec(last.hmdInPlayspace.position));
    const double turned =
        RotationAngleBetween(sample.hmdInPlayspace.rotation, last.hmdInPlayspace.rotation);
    return moved >= mOptions.sampleMinTranslation || turned >= mOptions.sampleMinRotation;
}

void ReferenceMarkerCalibrator::EvaluateResidual(const ReferenceMarkerSample& sample,
                                                 UpdateOutcome& outcome) const
{
    if (!mCalib || !mOffset) return;
    const Pose predicted = PredictHmdPose(*mCalib, *mOffset, sample.markerInCamera);
    outcome.positionError =
        cv::norm(ToVec(predicted.position) - ToVec(sample.hmdInPlayspace.position));
    outcome.rotationError = RotationAngleBetween(predicted.rotation, sample.hmdInPlayspace.rotation);
}

ReferenceMarkerCalibrator::UpdateOutcome ReferenceMarkerCalibrator::Update(
    const std::optional<Pose>& hmdInPlayspace, const std::optional<Pose>& markerInCamera)
{
    UpdateOutcome outcome;
    outcome.state = mState;
    outcome.sampleCount = static_cast<int>(mSamples.size());
    outcome.outlierCount = mOutlierCount;

    // Either input missing means this frame says nothing about the camera. The calibration
    // already in use stays untouched: the board leaving view is the normal case, and a
    // playspace that drifted whenever the user looked away would be worse than useless.
    if (!hmdInPlayspace || !markerInCamera) return outcome;

    const ReferenceMarkerSample sample{*markerInCamera, *hmdInPlayspace};

    if (!mOffset)
    {
        mState = State::CollectingSamples;
        outcome.state = mState;
        if (!ShouldStoreSample(sample)) return outcome;

        mSamples.push_back(sample);
        outcome.sampleCount = static_cast<int>(mSamples.size());

        const auto solved = SolveReferenceMarkerCalib(mSamples, mOptions.solver);
        if (!solved)
        {
            // The motion so far does not pin the mount down, most often because the head
            // only turned about one axis. Drop the oldest sample past the buffer size so a
            // poor start cannot keep the rest of the session from calibrating.
            if (static_cast<int>(mSamples.size()) > mOptions.maxSamples)
            {
                mSamples.erase(mSamples.begin());
                outcome.sampleCount = static_cast<int>(mSamples.size());
            }
            return outcome;
        }

        mOffset = solved->hmdToMarker;
        mScale = solved->playspace.scale;
        mCalib = solved->playspace;
        mSamples.clear();
        mOutlierCount = 0;
        mState = State::Calibrated;

        outcome.state = mState;
        outcome.sampleCount = 0;
        outcome.calib = mCalib;
        outcome.offsetSolved = true;
        EvaluateResidual(sample, outcome);
        return outcome;
    }

    const auto solved = SolvePlayspaceFromSample(sample, *mOffset, mScale);
    if (!solved) return outcome;

    if (!mCalib)
    {
        mCalib = solved;
        mState = State::Calibrated;
        outcome.state = mState;
        outcome.calib = mCalib;
        EvaluateResidual(sample, outcome);
        return outcome;
    }

    const auto delta = ComparePlayspaceCalib(*mCalib, *solved);
    if (delta.position > mOptions.outlierPosition || delta.rotation > mOptions.outlierRotation)
    {
        ++mOutlierCount;
        outcome.outlier = true;
        outcome.outlierCount = mOutlierCount;
        if (mOutlierCount < mOptions.outliersToAccept)
        {
            EvaluateResidual(sample, outcome);
            return outcome;
        }
        // The disagreement is not going away, so the camera moved rather than one detection
        // being wrong. Snap to the new solution; easing into it would leave tracking wrong
        // for as long as the blend takes.
        mCalib = solved;
        mOutlierCount = 0;
        outcome.outlierCount = 0;
        outcome.calib = mCalib;
        EvaluateResidual(sample, outcome);
        return outcome;
    }

    mOutlierCount = 0;
    outcome.outlierCount = 0;
    if (mOptions.continuous)
    {
        mCalib = BlendPlayspaceCalib(*mCalib, *solved, mOptions.smoothing);
        outcome.calib = mCalib;
    }
    EvaluateResidual(sample, outcome);
    return outcome;
}

#ifdef ATT_TESTING

namespace
{

/// A camera pose and a board mount to generate observations from, so that every solve below
/// is checked against an answer known in advance.
struct SyntheticWorld
{
    cfg::ManualCalib::Real playspace;
    Pose hmdToMarker;
};

SyntheticWorld MakeWorld()
{
    return {
        cfg::ManualCalib::Real{cv::Vec3d{0.15, 1.05, 2.2}, cv::Vec3d{185.0 * DEG_2_RAD, 12.0 * DEG_2_RAD, 3.0 * DEG_2_RAD}, 1.0},
        Pose{cv::Point3d{0.02, 0.06, -0.11},
             cv::Quatd::createFromRvec(cv::Vec3d{15.0 * DEG_2_RAD, -8.0 * DEG_2_RAD, 4.0 * DEG_2_RAD})}};
}

/// Board pose the camera would report for an HMD at hmdInPlayspace.
/// This inverts the playspace transform directly rather than calling anything the solver
/// uses, so a wrong model in the code under test cannot cancel out against the input.
ReferenceMarkerSample MakeSample(const SyntheticWorld& world, const Pose& hmdInPlayspace)
{
    const cv::Matx33d hmdRot = hmdInPlayspace.rotation.toRotMat3x3(cv::QUAT_ASSUME_UNIT);
    const cv::Matx33d markerRot = hmdRot * world.hmdToMarker.rotation.toRotMat3x3(cv::QUAT_ASSUME_UNIT);
    const cv::Vec3d markerPos = cv::Vec3d(hmdInPlayspace.position) +
                                (hmdRot * cv::Vec3d(world.hmdToMarker.position));

    const cv::Matx33d playspaceRot = EulerAnglesToRotationMatrix(world.playspace.angleOffset);
    cv::Vec3d camPos = (playspaceRot.t() * (markerPos - world.playspace.posOffset)) / world.playspace.scale;
    cv::Quatd camRot = cv::Quatd::createFromRotMat(playspaceRot.t() * markerRot).normalize();
    CoordTransformOVR(camPos);
    CoordTransformOVR(camRot);

    return {Pose{cv::Point3d(camPos), camRot}, hmdInPlayspace};
}

/// Head poses covering rotations about every axis, which is what makes the mount solvable.
std::vector<Pose> MakeHeadMotion(int count)
{
    std::vector<Pose> poses;
    for (int index = 0; index < count; ++index)
    {
        const double step = static_cast<double>(index);
        cv::Vec3d axis{std::sin(step * 0.7) + 0.2, std::cos(step * 0.5), std::sin((step * 0.3) + 1.0)};
        axis /= cv::norm(axis);
        const double angle = (18.0 + (12.0 * std::sin(step * 0.9))) * DEG_2_RAD;
        const cv::Point3d position{0.12 * std::sin(step * 0.4),
                                   1.6 + (0.05 * std::cos(step * 0.6)),
                                   0.1 * std::sin(step * 0.25)};
        poses.push_back(Pose{position, cv::Quatd::createFromRvec(cv::Vec3d(axis * angle)).normalize()});
    }
    return poses;
}

/// Head poses that only ever turn about the up axis, the motion a standing user produces
/// without being asked to do anything in particular.
std::vector<Pose> MakeYawOnlyHeadMotion(int count)
{
    std::vector<Pose> poses;
    for (int index = 0; index < count; ++index)
    {
        const double step = static_cast<double>(index);
        const double angle = (25.0 * std::sin(step * 0.8)) * DEG_2_RAD;
        const cv::Point3d position{0.12 * std::sin(step * 0.4), 1.6, 0.1 * std::sin(step * 0.25)};
        poses.push_back(Pose{position,
                             cv::Quatd::createFromRvec(cv::Vec3d{0, angle, 0}).normalize()});
    }
    return poses;
}

std::vector<ReferenceMarkerSample> MakeSamples(const SyntheticWorld& world,
                                               const std::vector<Pose>& headPoses)
{
    std::vector<ReferenceMarkerSample> samples;
    samples.reserve(headPoses.size());
    for (const auto& headPose : headPoses)
    {
        samples.push_back(MakeSample(world, headPose));
    }
    return samples;
}

void CheckCalibNear(const cfg::ManualCalib::Real& actual, const cfg::ManualCalib::Real& expected,
                    double posTolerance, double angleTolerance)
{
    const double posError = cv::norm(actual.posOffset - expected.posOffset);
    const double angleError = RotationAngleBetween(
        cv::Quatd::createFromRotMat(EulerAnglesToRotationMatrix(actual.angleOffset)).normalize(),
        cv::Quatd::createFromRotMat(EulerAnglesToRotationMatrix(expected.angleOffset)).normalize());
    const double scaleError = std::abs(actual.scale - expected.scale);
    CAPTURE(posError);
    CAPTURE(angleError);
    CAPTURE(scaleError);
    CHECK(posError <= posTolerance);
    CHECK(angleError <= angleTolerance);
    CHECK(scaleError <= posTolerance);
}

TEST_CASE("Reference marker forward model matches the playspace transform used for tracking")
{
    // The solver inverts this model. If it drifts from what PlayspaceCalib does, every
    // synthetic test below would still pass while real tracking came out wrong.
    const cfg::ManualCalib::Real calib{
        cv::Vec3d{0.15, 1.05, 2.2},
        cv::Vec3d{185.0 * DEG_2_RAD, 12.0 * DEG_2_RAD, 3.0 * DEG_2_RAD},
        1.07};
    const Pose poseInCamera{cv::Point3d{0.31, -0.22, 1.94},
                            cv::Quatd::createFromRvec(cv::Vec3d{0.3, -0.7, 0.2}).normalize()};

    tracker::PlayspaceCalib playspace;
    playspace.Set(calib);
    // MainLoopRunner scales the detected position before handing it to the transform
    const Pose scaledInCamera{poseInCamera.position * playspace.GetScale(), poseInCamera.rotation};
    const Pose expected = playspace.TransformToOVR(scaledInCamera);

    const Pose actual = PlayspaceTransform::FromCalib(calib).Apply(poseInCamera);

    CHECK(actual.position.x == doctest::Approx(expected.position.x).epsilon(1e-12));
    CHECK(actual.position.y == doctest::Approx(expected.position.y).epsilon(1e-12));
    CHECK(actual.position.z == doctest::Approx(expected.position.z).epsilon(1e-12));
    CHECK(RotationAngleBetween(actual.rotation, expected.rotation) < 1e-12);
}

TEST_CASE("Playspace angles round trip through the rotation matrix they are read from")
{
    const std::array<cv::Vec3d, 4> angleCases{{
        {180.0 * DEG_2_RAD, 0, 0}, // the cfg::ManualCalib default
        {185.0 * DEG_2_RAD, 12.0 * DEG_2_RAD, 3.0 * DEG_2_RAD},
        {120.0 * DEG_2_RAD, -150.0 * DEG_2_RAD, 30.0 * DEG_2_RAD},
        {250.0 * DEG_2_RAD, 95.0 * DEG_2_RAD, -80.0 * DEG_2_RAD},
    }};

    for (const auto& angles : angleCases)
    {
        const double pitchDeg = angles[0] * RAD_2_DEG;
        CAPTURE(pitchDeg);
        const cv::Matx33d rot = EulerAnglesToRotationMatrix(angles);
        const cv::Vec3d decoded = PlayspaceAnglesFromRotation(rot);
        const cv::Matx33d rebuilt = EulerAnglesToRotationMatrix(decoded);

        // the decoded pitch must stay in the range ManualCalib and the manual calibrator use
        CHECK(decoded[0] >= 90.0 * DEG_2_RAD);
        CHECK(decoded[0] <= 270.0 * DEG_2_RAD);
        for (int row = 0; row < 3; ++row)
        {
            for (int col = 0; col < 3; ++col)
            {
                CHECK(rebuilt(row, col) == doctest::Approx(rot(row, col)).epsilon(1e-12));
            }
        }
    }
}

TEST_CASE("Reference marker hand eye solve recovers a known camera pose and board mount")
{
    const SyntheticWorld world = MakeWorld();
    const auto samples = MakeSamples(world, MakeHeadMotion(12));

    const auto solved = SolveReferenceMarkerCalib(samples);

    REQUIRE(solved.has_value());
    CheckCalibNear(solved->playspace, world.playspace, 1e-6, 1e-6);
    CHECK(solved->playspace.scale == doctest::Approx(world.playspace.scale).epsilon(1e-9));
    const double offsetPosError =
        cv::norm(cv::Vec3d(solved->hmdToMarker.position) - cv::Vec3d(world.hmdToMarker.position));
    const double offsetRotError =
        RotationAngleBetween(solved->hmdToMarker.rotation, world.hmdToMarker.rotation);
    CAPTURE(offsetPosError);
    CAPTURE(offsetRotError);
    CHECK(offsetPosError < 1e-6);
    CHECK(offsetRotError < 1e-6);
    CHECK(solved->positionResidual < 1e-9);
    CHECK(solved->rotationResidual < 1e-9);
    CHECK(solved->sampleCount == 12);
}

TEST_CASE("Reference marker hand eye solve recovers a camera scale other than one")
{
    SyntheticWorld world = MakeWorld();
    world.playspace.scale = 1.08;
    const auto samples = MakeSamples(world, MakeHeadMotion(12));

    const auto solved = SolveReferenceMarkerCalib(samples);

    REQUIRE(solved.has_value());
    CHECK(solved->playspace.scale == doctest::Approx(1.08).epsilon(1e-9));
    CheckCalibNear(solved->playspace, world.playspace, 1e-6, 1e-6);
}

TEST_CASE("Reference marker hand eye solve rejects head motion about a single axis")
{
    // Turning the head about one axis leaves the mount rotation about that axis unobservable.
    // Accepting such a fit would hand G4 a camera pose that is wrong in a way no residual
    // computed from the same samples can reveal.
    const SyntheticWorld world = MakeWorld();
    const auto samples = MakeSamples(world, MakeYawOnlyHeadMotion(20));

    CHECK_NOT(SolveReferenceMarkerCalib(samples).has_value());
}

TEST_CASE("Reference marker hand eye solve rejects observations that do not fit one rigid mount")
{
    // A board that is not rigidly mounted, or detections from the wrong marker range, cannot
    // be described by any fixed offset. The residual check is what has to notice.
    const SyntheticWorld world = MakeWorld();
    const auto headPoses = MakeHeadMotion(12);
    std::vector<ReferenceMarkerSample> samples;
    for (std::size_t index = 0; index < headPoses.size(); ++index)
    {
        SyntheticWorld shifting = world;
        // mount slipping by centimeters between observations
        shifting.hmdToMarker = Pose{
            world.hmdToMarker.position + cv::Point3d(0.15 * std::sin(static_cast<double>(index)), 0, 0),
            world.hmdToMarker.rotation};
        samples.push_back(MakeSample(shifting, headPoses[index]));
    }

    CHECK_NOT(SolveReferenceMarkerCalib(samples).has_value());
}

TEST_CASE("Reference marker hand eye solve waits for enough observations")
{
    const SyntheticWorld world = MakeWorld();
    const auto samples = MakeSamples(world, MakeHeadMotion(3));

    CHECK_NOT(SolveReferenceMarkerCalib(samples).has_value());
}

TEST_CASE("Reference marker single frame solve recovers the camera pose from a known mount")
{
    const SyntheticWorld world = MakeWorld();
    const auto samples = MakeSamples(world, MakeHeadMotion(4));

    for (const auto& sample : samples)
    {
        const auto solved = SolvePlayspaceFromSample(sample, world.hmdToMarker, world.playspace.scale);
        REQUIRE(solved.has_value());
        CheckCalibNear(*solved, world.playspace, 1e-9, 1e-9);
    }
}

TEST_CASE("Reference marker prediction reproduces the pose the camera reported")
{
    // This is the prediction the search mask relies on. If it disagreed with the detection,
    // masking would hide the board and detection would stop.
    const SyntheticWorld world = MakeWorld();
    const auto samples = MakeSamples(world, MakeHeadMotion(4));

    for (const auto& sample : samples)
    {
        const auto predicted =
            PredictMarkerPosInCamera(world.playspace, world.hmdToMarker, sample.hmdInPlayspace);
        REQUIRE(predicted.has_value());
        const double error = cv::norm(*predicted - cv::Vec3d(sample.markerInCamera.position));
        CAPTURE(error);
        CHECK(error < 1e-9);

        const Pose hmd = PredictHmdPose(world.playspace, world.hmdToMarker, sample.markerInCamera);
        const double hmdPosError = cv::norm(cv::Vec3d(hmd.position) - cv::Vec3d(sample.hmdInPlayspace.position));
        CAPTURE(hmdPosError);
        CHECK(hmdPosError < 1e-9);
        CHECK(RotationAngleBetween(hmd.rotation, sample.hmdInPlayspace.rotation) < 1e-9);
    }
}

TEST_CASE("Playspace calibrations blend along a rotation rather than through their angles")
{
    const cfg::ManualCalib::Real from{cv::Vec3d{0, 1, 2}, cv::Vec3d{180.0 * DEG_2_RAD, 170.0 * DEG_2_RAD, 0}, 1.0};
    const cfg::ManualCalib::Real to{cv::Vec3d{2, 1, 0}, cv::Vec3d{180.0 * DEG_2_RAD, -170.0 * DEG_2_RAD, 0}, 1.1};

    const auto blended = BlendPlayspaceCalib(from, to, 0.5);

    // yaw crosses the wrap point, so the halfway orientation is 180 degrees, not 0
    const cfg::ManualCalib::Real expected{cv::Vec3d{1, 1, 1}, cv::Vec3d{180.0 * DEG_2_RAD, PI, 0}, 1.05};
    CheckCalibNear(blended, expected, 1e-9, 1e-9);

    CheckCalibNear(BlendPlayspaceCalib(from, to, 0.0), from, 1e-9, 1e-9);
    CheckCalibNear(BlendPlayspaceCalib(from, to, 1.0), to, 1e-9, 1e-9);
}

TEST_CASE("Reference marker calibrator solves the mount from a stream of observations")
{
    const SyntheticWorld world = MakeWorld();
    ReferenceMarkerCalibrator calibrator;
    const auto samples = MakeSamples(world, MakeHeadMotion(20));

    CHECK(calibrator.GetState() == ReferenceMarkerCalibrator::State::WaitingForData);

    int solvedCount = 0;
    std::optional<cfg::ManualCalib::Real> lastCalib;
    for (const auto& sample : samples)
    {
        const auto outcome = calibrator.Update(sample.hmdInPlayspace, sample.markerInCamera);
        if (outcome.offsetSolved) ++solvedCount;
        if (outcome.calib) lastCalib = outcome.calib;
    }

    CHECK(solvedCount == 1);
    CHECK(calibrator.GetState() == ReferenceMarkerCalibrator::State::Calibrated);
    CHECK(calibrator.IsCalibrated());
    REQUIRE(lastCalib.has_value());
    CheckCalibNear(*lastCalib, world.playspace, 1e-6, 1e-6);
    REQUIRE(calibrator.GetOffset().has_value());
    CHECK(cv::norm(cv::Vec3d(calibrator.GetOffset()->position) - cv::Vec3d(world.hmdToMarker.position)) < 1e-6);
}

TEST_CASE("Reference marker calibrator keeps its calibration while inputs are missing")
{
    const SyntheticWorld world = MakeWorld();
    ReferenceMarkerCalibrator calibrator;
    calibrator.SetOffset(world.hmdToMarker, world.playspace.scale);
    const auto samples = MakeSamples(world, MakeHeadMotion(3));

    const auto first = calibrator.Update(samples[0].hmdInPlayspace, samples[0].markerInCamera);
    REQUIRE(first.calib.has_value());
    CHECK_NOT(first.offsetSolved);
    CheckCalibNear(*first.calib, world.playspace, 1e-9, 1e-9);

    // board out of view, HMD tracked
    const auto noBoard = calibrator.Update(samples[1].hmdInPlayspace, std::nullopt);
    CHECK_NOT(noBoard.calib.has_value());
    CHECK(noBoard.state == ReferenceMarkerCalibrator::State::Calibrated);
    // HMD lost, board visible
    const auto noHmd = calibrator.Update(std::nullopt, samples[1].markerInCamera);
    CHECK_NOT(noHmd.calib.has_value());
    // the calibration itself is untouched
    REQUIRE(calibrator.GetCalib().has_value());
    CheckCalibNear(*calibrator.GetCalib(), world.playspace, 1e-9, 1e-9);
}

TEST_CASE("Reference marker calibrator holds back one bad solve but follows a moved camera")
{
    const SyntheticWorld world = MakeWorld();
    ReferenceMarkerCalibrator::Options options;
    options.outliersToAccept = 5;
    ReferenceMarkerCalibrator calibrator{options};
    calibrator.SetOffset(world.hmdToMarker, world.playspace.scale);

    const auto headPoses = MakeHeadMotion(12);
    const auto samples = MakeSamples(world, headPoses);
    calibrator.Update(samples[0].hmdInPlayspace, samples[0].markerInCamera);
    REQUIRE(calibrator.GetCalib().has_value());

    // the camera is knocked half a meter sideways and turned
    SyntheticWorld moved = world;
    moved.playspace.posOffset += cv::Vec3d{0.5, 0, 0};
    moved.playspace.angleOffset += cv::Vec3d{0, 20.0 * DEG_2_RAD, 0};
    const auto movedSamples = MakeSamples(moved, headPoses);

    const auto firstAfterMove = calibrator.Update(movedSamples[1].hmdInPlayspace, movedSamples[1].markerInCamera);
    CHECK(firstAfterMove.outlier);
    CHECK_NOT(firstAfterMove.calib.has_value());
    // the held back solution is visible as a large residual rather than being hidden
    REQUIRE(firstAfterMove.positionError.has_value());
    CHECK(*firstAfterMove.positionError > 0.1);
    CheckCalibNear(*calibrator.GetCalib(), world.playspace, 1e-9, 1e-9);

    std::optional<cfg::ManualCalib::Real> accepted;
    for (int index = 2; index < static_cast<int>(movedSamples.size()); ++index)
    {
        const auto& sample = movedSamples[static_cast<std::size_t>(index)];
        const auto outcome = calibrator.Update(sample.hmdInPlayspace, sample.markerInCamera);
        if (outcome.calib) accepted = outcome.calib;
    }

    REQUIRE(accepted.has_value());
    CheckCalibNear(*accepted, moved.playspace, 1e-6, 1e-6);
}

TEST_CASE("Reference marker calibrator can be told not to keep re-solving")
{
    const SyntheticWorld world = MakeWorld();
    ReferenceMarkerCalibrator::Options options;
    options.continuous = false;
    ReferenceMarkerCalibrator calibrator{options};
    calibrator.SetOffset(world.hmdToMarker, world.playspace.scale);
    const auto samples = MakeSamples(world, MakeHeadMotion(4));

    const auto first = calibrator.Update(samples[0].hmdInPlayspace, samples[0].markerInCamera);
    CHECK(first.calib.has_value());
    for (std::size_t index = 1; index < samples.size(); ++index)
    {
        const auto outcome = calibrator.Update(samples[index].hmdInPlayspace, samples[index].markerInCamera);
        CHECK_NOT(outcome.calib.has_value());
        // residuals keep being reported, so the calibration can still be judged
        CHECK(outcome.positionError.has_value());
    }
}

} // namespace

#endif // ATT_TESTING

} // namespace tracker
