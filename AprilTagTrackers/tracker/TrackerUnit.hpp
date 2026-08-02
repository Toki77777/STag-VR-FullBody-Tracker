#pragma once

#include "config/TrackerUnit.hpp"
#include "Helpers.hpp"
#include "math/CVHelpers.hpp"
#include "math/CVTypes.hpp"
#include "utils/Error.hpp"
#include "utils/Types.hpp"

#include <vector>

namespace tracker
{

class TrackerUnit
{
    using MarkersList = std::vector<MarkerCorners3f>;
    using IdsList = std::vector<int>;

    /// move corners to the center of all markers in the list
    static void RecenterCornersList(MarkersList& markers)
    {
        cv::Point3f trackerCenter{};
        for (const auto& corners : markers)
        {
            for (const auto& corner : corners)
            {
                trackerCenter += corner;
            }
        }
        constexpr std::size_t numOfCorners = 4;
        trackerCenter /= static_cast<float>(markers.size() * numOfCorners);
        for (auto& corners : markers)
        {
            for (auto& corner : corners)
            {
                corner -= trackerCenter;
            }
        }
    }

    static void EnsureCorners(const MarkerCorners3f& corners)
    {
        if (corners.size() != 4) throw utils::MakeError("expected 4 corners, got ", corners.size());
    }
    static void EnsureMarkers(const IdsList& ids, const MarkersList& cornersList)
    {
        if (ids.size() != cornersList.size()) throw utils::MakeError("ids size ", ids.size(), " != cornersList size", cornersList.size());
        for (const auto& corners : cornersList)
        {
            EnsureCorners(corners);
        }
    }

public:
    static MarkerCorners3f CreateModelMarker(double markerSizeM)
    {
        const auto halfSize = static_cast<float>(markerSizeM / 2.0);
        return {
            cv::Point3f(-halfSize, halfSize, 0), // top right
            cv::Point3f(halfSize, halfSize, 0), // top left
            cv::Point3f(halfSize, -halfSize, 0), // bottom left
            cv::Point3f(-halfSize, -halfSize, 0)}; // bottom right
    }

    void RecenterMarkers()
    {
        RecenterCornersList(mMarkers);
        RebuildArucoBoard();
    }

    void SetMarkers(IdsList ids, MarkersList cornersList)
    {
        EnsureMarkers(ids, cornersList);
        mIds = std::move(ids);
        mMarkers = std::move(cornersList);
        RebuildArucoBoard();
    }
    void AddMarker(int id, MarkerCorners3f corners)
    {
        EnsureCorners(corners);
        mIds.push_back(id);
        mMarkers.push_back(std::move(corners));
        RebuildArucoBoard();
    }

    void SetRole(cfg::TrackerRole role) { mRole = role; }

    bool HasMarkerId(int id) const { return std::find(GetIds().begin(), GetIds().end(), id) != GetIds().end(); }
    Index GetMarkerCount() { return mIds.size(); }
    /// best tell for calibration is whether the corner offsets of markers have been set
    /// so if they are all zero this could return a false positive
    bool IsCalibrated() const { return !mMarkers.empty(); }
    /// more accurate and expensive check
    bool EnsureIsCalibrated() const
    {
        // if (!IsCalibrated()) return false;
        // TODO: check ids in range, and marker corners have some meaning, maybe as simple as not zero
        utils::Unreachable();
    }

    void SetWasVisibleLastFrame(bool isFound) { mIsFound = isFound; }
    void SetWasVisibleToDriverLastFrame(bool isFound) { mIsDriverFound = isFound; }
    bool WasVisibleLastFrame() const { return mIsFound; }
    bool WasVisibleToDriverLastFrame() const { return mIsDriverFound; }

    void SetMaskCenter(const cv::Point2d& center) { mMaskCenter = center; }
    const cv::Point2d& GetMaskCenter() const { return mMaskCenter; }

    void SetEstimatedPose(const RodrPose& pose) { mPose = pose; }
    const RodrPose& GetEstimatedPose() const { return mPose; }
    void SetPoseFromDriver(const RodrPose& pose) { mDriverPose = pose; }
    const RodrPose& GetPoseFromDriver() const { return mDriverPose; }

    const ArucoBoardSharedPtr& GetArucoBoard() const { return mArucoBoard; }
    const MarkersList& GetMarkers() const { return mMarkers; }
    const IdsList& GetIds() const { return mIds; }

private:
    /// cv::aruco::Board is immutable since OpenCV 4.7,
    /// so recreate it whenever the marker list changes
    void RebuildArucoBoard()
    {
        if (mIds.empty())
        {
            mArucoBoard.release();
            return;
        }
        // the dictionary is unused, detection happens through StagWrapper,
        // but the board requires one to be constructed
        static const cv::aruco::Dictionary dictionary =
            cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);
        mArucoBoard = cv::makePtr<cv::aruco::Board>(mMarkers, dictionary, mIds);
    }

    /// ids and corners of markers, source of truth for mArucoBoard
    IdsList mIds{};
    MarkersList mMarkers{};
    /// immutable snapshot of mIds and mMarkers, null while they are empty
    ArucoBoardSharedPtr mArucoBoard{};

    RodrPose mPose{};
    cv::Point2d mMaskCenter{};
    bool mIsFound = false;

    RodrPose mDriverPose{};
    bool mIsDriverFound = false;

    cfg::TrackerRole mRole = cfg::TrackerRole::Disabled;
};

} // namespace tracker
