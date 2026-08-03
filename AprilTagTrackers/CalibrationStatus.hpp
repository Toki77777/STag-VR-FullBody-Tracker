#pragma once

#include "Config.hpp"
#include "utils/Types.hpp"

#include <opencv2/core.hpp>

namespace tracker
{

/// What can be told about a camera calibration from the data that gets stored with it.
struct CameraCalibScore
{
    /// a usable camera matrix is present, which is what every other stage requires
    bool present = false;
    /// views the calibration was solved from, 0 when the stored data does not say
    int views = 0;
    /// mean reprojection error over those views, in pixels
    double meanError = 0;
    double maxError = 0;

    bool HasErrorData() const { return views > 0; }
};

CameraCalibScore ScoreCameraCalib(const cfg::CameraCalib& calib);

/// Whether replacing a stored camera calibration with a freshly measured one has to be
/// confirmed by the user first. A stored calibration is work the user already did once, so a
/// new one that looks worse must not quietly take its place; there is no undo for that.
bool ShouldConfirmCameraCalibReplacement(const CameraCalibScore& stored, const CameraCalibScore& fresh);

/// Which parts of the one time setup are already stored. Reported to the user so that
/// "is my calibration still there" is answered on screen instead of guessed.
struct CalibrationStatus
{
    bool camera = false;
    int calibratedTrackers = 0;
    /// tracker slots that starting tracking requires, see GetCalibrationStatus
    int trackerCount = 0;

    bool AllTrackers() const { return trackerCount > 0 && calibratedTrackers >= trackerCount; }
    bool IsReadyToTrack() const { return camera && AllTrackers(); }
};

/// Read what is stored. trackerCount counts the configured tracker slots rather than
/// trackerNum, matching what Tracker requires before it will start, so a reported ready
/// state cannot disagree with whether starting actually works.
CalibrationStatus GetCalibrationStatus(const UserConfig& userConfig, const CalibrationConfig& calibConfig);

/// The single next thing the user has to do, so guidance names one step instead of listing
/// everything. Camera comes first because tracker calibration cannot run without it.
enum class CalibrationStep
{
    Camera,
    Trackers,
    Done
};

CalibrationStep GetNextCalibrationStep(const CalibrationStatus& status);

} // namespace tracker
