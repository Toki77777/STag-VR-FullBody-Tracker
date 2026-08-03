#include "CalibrationStatus.hpp"

#include "utils/Test.hpp"

#include <algorithm>

namespace tracker
{

namespace
{

/// Reprojection error above this is a bad calibration whatever the stored one looks like.
constexpr double MAX_ACCEPTABLE_MEAN_ERROR = 1.0;
/// How much worse than the stored error a fresh calibration may be before asking.
constexpr double WORSE_ERROR_RATIO = 1.5;
/// Views a fresh calibration may be short by before it counts as a step backwards.
constexpr int VIEW_COUNT_MARGIN = 2;

/// Focal length entry of a camera matrix, 0 when the matrix cannot be read as one.
double DiagonalOrZero(const cv::Mat& matrix, int index)
{
    if (matrix.empty() || matrix.rows != 3 || matrix.cols != 3) return 0;
    if (matrix.type() == CV_64F) return matrix.at<double>(index, index);
    if (matrix.type() == CV_32F) return static_cast<double>(matrix.at<float>(index, index));
    return 0;
}

} // namespace

CameraCalibScore ScoreCameraCalib(const cfg::CameraCalib& calib)
{
    CameraCalibScore score;
    // A stored matrix of the right shape with positive focal lengths is what the tracking and
    // calibration paths check for, so agree with them rather than only testing for empty.
    score.present = DiagonalOrZero(calib.cameraMatrix, 0) > 0 &&
                    DiagonalOrZero(calib.cameraMatrix, 1) > 0;

    if (calib.perViewErrors.empty()) return score;
    score.views = static_cast<int>(calib.perViewErrors.size());
    double sum = 0;
    for (const double error : calib.perViewErrors)
    {
        sum += error;
        score.maxError = std::max(score.maxError, error);
    }
    score.meanError = sum / static_cast<double>(score.views);
    return score;
}

bool ShouldConfirmCameraCalibReplacement(const CameraCalibScore& stored, const CameraCalibScore& fresh)
{
    // nothing stored, so nothing can be lost
    if (!stored.present) return false;
    // the fresh one is not usable at all; refusing silently would be worse than asking
    if (!fresh.present) return true;
    // one of them carries no error data, for instance an older calibration or the legacy
    // chessboard path, so the two cannot be compared and the stored one gets the benefit
    if (!stored.HasErrorData() || !fresh.HasErrorData()) return true;

    if (fresh.meanError > MAX_ACCEPTABLE_MEAN_ERROR) return true;
    if (fresh.meanError > stored.meanError * WORSE_ERROR_RATIO) return true;
    if (fresh.views + VIEW_COUNT_MARGIN < stored.views) return true;
    return false;
}

CalibrationStatus GetCalibrationStatus(const UserConfig& userConfig, const CalibrationConfig& calibConfig)
{
    CalibrationStatus status;
    if (calibConfig.cameras.GetSize() > 0)
    {
        status.camera = ScoreCameraCalib(*calibConfig.cameras[0]).present;
    }

    const Index trackerSlots = std::max(userConfig.trackers.GetSize(), calibConfig.trackers.GetSize());
    status.trackerCount = static_cast<int>(trackerSlots);
    for (Index index = 0; index < calibConfig.trackers.GetSize(); ++index)
    {
        const auto tracker = calibConfig.trackers[index];
        // matches tracker::TrackerUnit::IsCalibrated, which is what starting tracking checks
        if (!tracker->ids.empty() && tracker->ids.size() == tracker->corners.size())
        {
            ++status.calibratedTrackers;
        }
    }
    return status;
}

CalibrationStep GetNextCalibrationStep(const CalibrationStatus& status)
{
    if (!status.camera) return CalibrationStep::Camera;
    if (!status.AllTrackers()) return CalibrationStep::Trackers;
    return CalibrationStep::Done;
}

#ifdef ATT_TESTING

namespace
{

cfg::CameraCalib MakeCameraCalib(std::vector<double> perViewErrors)
{
    cfg::CameraCalib calib;
    calib.cameraMatrix = (cv::Mat_<double>(3, 3) << 612.5, 0, 319.5, 0, 611.25, 239.5, 0, 0, 1);
    calib.distortionCoeffs = (cv::Mat_<double>(1, 5) << 0.1, -0.25, 0.001, -0.002, 0.05);
    calib.perViewErrors = std::move(perViewErrors);
    return calib;
}

TEST_CASE("A camera calibration is scored as present only when it can be used")
{
    cfg::CameraCalib empty;
    CHECK_NOT(ScoreCameraCalib(empty).present);

    // right shape, but a zero focal length cannot project anything
    cfg::CameraCalib zeroed;
    zeroed.cameraMatrix = cv::Mat::zeros(3, 3, CV_64F);
    CHECK_NOT(ScoreCameraCalib(zeroed).present);

    const auto scored = ScoreCameraCalib(MakeCameraCalib({0.2, 0.4, 0.3}));
    CHECK(scored.present);
    CHECK(scored.views == 3);
    CHECK(scored.meanError == doctest::Approx(0.3));
    CHECK(scored.maxError == doctest::Approx(0.4));
    CHECK(scored.HasErrorData());
}

TEST_CASE("Replacing a stored camera calibration is confirmed only when it looks like a loss")
{
    const auto stored = ScoreCameraCalib(MakeCameraCalib({0.2, 0.3, 0.25, 0.2, 0.3, 0.25, 0.2, 0.3, 0.25, 0.2}));

    // a first calibration has nothing to overwrite
    CHECK_NOT(ShouldConfirmCameraCalibReplacement(ScoreCameraCalib(cfg::CameraCalib{}), stored));

    // a comparable recalibration goes through without interrupting the user
    const auto comparable = ScoreCameraCalib(MakeCameraCalib({0.22, 0.28, 0.26, 0.21, 0.3, 0.24, 0.2, 0.31, 0.25, 0.2}));
    CHECK_NOT(ShouldConfirmCameraCalibReplacement(stored, comparable));

    // clearly worse reprojection error
    const auto worseError = ScoreCameraCalib(MakeCameraCalib({0.6, 0.7, 0.65, 0.6, 0.7, 0.65, 0.6, 0.7, 0.65, 0.6}));
    CHECK(ShouldConfirmCameraCalibReplacement(stored, worseError));

    // bad in absolute terms, even against a stored calibration that was worse still
    const auto poorStored = ScoreCameraCalib(MakeCameraCalib({0.9, 0.95, 0.9, 0.95, 0.9, 0.95, 0.9, 0.95, 0.9, 0.95}));
    const auto bad = ScoreCameraCalib(MakeCameraCalib({1.2, 1.3, 1.25, 1.2, 1.3, 1.25, 1.2, 1.3, 1.25, 1.2}));
    CHECK(ShouldConfirmCameraCalibReplacement(poorStored, bad));

    // a handful of views replacing many, which is the accidental short session
    const auto fewViews = ScoreCameraCalib(MakeCameraCalib({0.2, 0.2, 0.2}));
    CHECK(ShouldConfirmCameraCalibReplacement(stored, fewViews));

    // no error data on either side leaves nothing to compare, so the stored one is protected
    const auto noErrorData = ScoreCameraCalib(MakeCameraCalib({}));
    CHECK(noErrorData.present);
    CHECK_NOT(noErrorData.HasErrorData());
    CHECK(ShouldConfirmCameraCalibReplacement(stored, noErrorData));
    CHECK(ShouldConfirmCameraCalibReplacement(noErrorData, comparable));

    // an unusable result never replaces a stored calibration unasked
    CHECK(ShouldConfirmCameraCalibReplacement(stored, ScoreCameraCalib(cfg::CameraCalib{})));
}

TEST_CASE("Stored calibration is reported per part and only counts as ready when complete")
{
    UserConfig userConfig;
    CalibrationConfig calibConfig;
    const MarkerCorners3f marker{
        cv::Point3f(-0.05F, 0.05F, 0), cv::Point3f(0.05F, 0.05F, 0),
        cv::Point3f(0.05F, -0.05F, 0), cv::Point3f(-0.05F, -0.05F, 0)};

    const auto fresh = GetCalibrationStatus(userConfig, calibConfig);
    CHECK_NOT(fresh.camera);
    CHECK(fresh.calibratedTrackers == 0);
    CHECK(fresh.trackerCount == 3);
    CHECK_NOT(fresh.IsReadyToTrack());
    CHECK(GetNextCalibrationStep(fresh) == CalibrationStep::Camera);

    *calibConfig.cameras[0] = MakeCameraCalib({0.2, 0.3});
    const auto cameraOnly = GetCalibrationStatus(userConfig, calibConfig);
    CHECK(cameraOnly.camera);
    CHECK_NOT(cameraOnly.IsReadyToTrack());
    CHECK(GetNextCalibrationStep(cameraOnly) == CalibrationStep::Trackers);

    calibConfig.trackers[0]->ids = {0};
    calibConfig.trackers[0]->corners = {marker};
    const auto partial = GetCalibrationStatus(userConfig, calibConfig);
    CHECK(partial.calibratedTrackers == 1);
    CHECK(partial.trackerCount == 3);
    CHECK_NOT(partial.AllTrackers());
    CHECK_NOT(partial.IsReadyToTrack());
    CHECK(GetNextCalibrationStep(partial) == CalibrationStep::Trackers);

    for (Index index = 1; index < calibConfig.trackers.GetSize(); ++index)
    {
        calibConfig.trackers[index]->ids = {static_cast<int>(index) * 45};
        calibConfig.trackers[index]->corners = {marker};
    }
    const auto complete = GetCalibrationStatus(userConfig, calibConfig);
    CHECK(complete.calibratedTrackers == 3);
    CHECK(complete.AllTrackers());
    CHECK(complete.IsReadyToTrack());
    CHECK(GetNextCalibrationStep(complete) == CalibrationStep::Done);
}

TEST_CASE("A tracker slot with corners missing does not count as calibrated")
{
    UserConfig userConfig;
    CalibrationConfig calibConfig;
    *calibConfig.cameras[0] = MakeCameraCalib({0.2, 0.3});
    // ids without the matching corners is the shape TrackerUnit::SetMarkers rejects
    calibConfig.trackers[0]->ids = {0, 1};
    calibConfig.trackers[0]->corners = {};

    const auto status = GetCalibrationStatus(userConfig, calibConfig);
    CHECK(status.calibratedTrackers == 0);
    CHECK(GetNextCalibrationStep(status) == CalibrationStep::Trackers);
}

} // namespace

#endif // ATT_TESTING

} // namespace tracker
