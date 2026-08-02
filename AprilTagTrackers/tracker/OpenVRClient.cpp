#include "OpenVRClient.hpp"

#include "Helpers.hpp"
#include "utils/Env.hpp"
#include "utils/Test.hpp"

#include <array>
#include <optional>
#include <stdexcept>
#include <string>

namespace
{

inline void ThrowVRError(vr::EVRInitError ec)
{
    throw std::runtime_error(vr::VR_GetVRInitErrorAsEnglishDescription(ec));
}
inline void ThrowVRError(vr::EVRInputError ec)
{
    throw std::runtime_error("VRInputError: " + std::to_string(ec));
}
template <typename T>
inline void ThrowIfError(T errorCode)
{
    if (static_cast<int>(errorCode) == 0) return;
    ThrowVRError(errorCode);
}

/// update actions for this frame
inline void UpdateActionSetState(vr::VRActionSetHandle_t actionSet)
{
    vr::VRActiveActionSet_t activeSet{};
    activeSet.ulActionSet = actionSet;
    vr::VRInput()->UpdateActionState(&activeSet, sizeof(activeSet), 1);
}

/// call after UpdateActionSetState
inline bool GetDigitalState(vr::VRActionHandle_t action)
{
    vr::InputDigitalActionData_t actionData{};
    ThrowIfError(vr::VRInput()->GetDigitalActionData(action, &actionData, sizeof(actionData), vr::k_ulInvalidInputValueHandle));
    return actionData.bActive && actionData.bState;
}

std::optional<vr::HmdMatrix34_t> GetPoseForNextFrame(vr::VRActionHandle_t action)
{
    vr::InputPoseActionData_t actionData{};
    constexpr vr::ETrackingUniverseOrigin origin = vr::TrackingUniverseRawAndUncalibrated;
    ThrowIfError(vr::VRInput()->GetPoseActionDataForNextFrame(action, origin, &actionData, sizeof(actionData), vr::k_ulInvalidInputValueHandle));
    if (!actionData.bActive || !actionData.pose.bPoseIsValid) return std::nullopt;
    return actionData.pose.mDeviceToAbsoluteTracking;
}

Pose OVRPoseMatrixToPose(vr::HmdMatrix34_t matrix)
{
    const cv::Matx33d rotMat(
        matrix.m[0][0], matrix.m[0][1], matrix.m[0][2],
        matrix.m[1][0], matrix.m[1][1], matrix.m[1][2],
        matrix.m[2][0], matrix.m[2][1], matrix.m[2][2]);
    cv::Quatd rot = cv::Quatd::createFromRotMat(rotMat).normalize();

    cv::Point3d pos{matrix.m[0][3], matrix.m[1][3], matrix.m[2][3]};

    // openvr is +x right, +y up, -z forward
    // opencv is +x right, -y up, +z forward
    // So negate y and z of pos and rot to convert
    // Except thats not true?
    // transform to -x right, +y up, +z forward
    // which is apparently what ATT uses sometimes
    CoordTransformOVR(pos);
    CoordTransformOVR(rot);

    return Pose{pos, rot};
}

std::optional<Pose> OVRTrackedDevicePoseToPose(const vr::TrackedDevicePose_t& trackedPose)
{
    if (!trackedPose.bDeviceIsConnected ||
        !trackedPose.bPoseIsValid ||
        trackedPose.eTrackingResult != vr::TrackingResult_Running_OK)
    {
        return std::nullopt;
    }
    return OVRPoseMatrixToPose(trackedPose.mDeviceToAbsoluteTracking);
}

TEST_CASE("OpenVR tracked HMD pose converts valid raw coordinates to ATT coordinates")
{
    vr::TrackedDevicePose_t trackedPose{};
    trackedPose.mDeviceToAbsoluteTracking = vr::HmdMatrix34_t{{
        {1.0F, 0.0F, 0.0F, 1.0F},
        {0.0F, 1.0F, 0.0F, 2.0F},
        {0.0F, 0.0F, 1.0F, 3.0F},
    }};
    trackedPose.eTrackingResult = vr::TrackingResult_Running_OK;
    trackedPose.bPoseIsValid = true;
    trackedPose.bDeviceIsConnected = true;

    const auto pose = OVRTrackedDevicePoseToPose(trackedPose);

    REQUIRE(pose.has_value());
    CHECK(pose->position.x == doctest::Approx(-1.0));
    CHECK(pose->position.y == doctest::Approx(2.0));
    CHECK(pose->position.z == doctest::Approx(-3.0));
    // q and -q represent the same rotation; OpenCV may choose either sign.
    CHECK(std::abs(pose->rotation.w) == doctest::Approx(1.0));
    CHECK(pose->rotation.x == doctest::Approx(0.0));
    CHECK(pose->rotation.y == doctest::Approx(0.0));
    CHECK(pose->rotation.z == doctest::Approx(0.0));
}

TEST_CASE("OpenVR tracked HMD pose rejects unavailable and degraded tracking")
{
    vr::TrackedDevicePose_t trackedPose{};
    trackedPose.eTrackingResult = vr::TrackingResult_Running_OK;
    trackedPose.bPoseIsValid = true;
    trackedPose.bDeviceIsConnected = false;
    CHECK_NOT(OVRTrackedDevicePoseToPose(trackedPose).has_value());

    trackedPose.bDeviceIsConnected = true;
    trackedPose.bPoseIsValid = false;
    CHECK_NOT(OVRTrackedDevicePoseToPose(trackedPose).has_value());

    trackedPose.bPoseIsValid = true;
    trackedPose.eTrackingResult = vr::TrackingResult_Running_OutOfRange;
    CHECK_NOT(OVRTrackedDevicePoseToPose(trackedPose).has_value());
}

TEST_CASE("MockOpenVRClient returns an injected HMD pose or an explicit invalid result")
{
    tracker::MockOpenVRClient client;
    CHECK_NOT(client.IsInit());
    client.Init();

    CHECK(client.IsInit());
    CHECK_NOT(client.GetHMDPose().has_value());

    const Pose expected{{4.0, -5.0, 6.0}, {0.5, 0.5, -0.5, 0.5}};
    client.SetHMDPose(expected);
    const auto actual = client.GetHMDPose();
    REQUIRE(actual.has_value());
    CHECK(actual->position == expected.position);
    CHECK(actual->rotation == expected.rotation);

    client.SetHMDPose(std::nullopt);
    CHECK_NOT(client.GetHMDPose().has_value());
}

} // namespace

namespace tracker
{

bool OpenVRClient::CanInit() const
{
    if (IsInit()) return false;
    return vr::VR_IsHmdPresent();
}

void OpenVRClient::Init()
{
    if (mContext) throw std::runtime_error("openvr already initialized");

    vr::EVRInitError initError = vr::VRInitError_None;
    vr::IVRSystem* context = vr::VR_Init(&initError, vr::VRApplication_Overlay);
    if (initError != 0 || context == nullptr) ThrowVRError(initError);
    mContext.reset(context);
    ATT_LOG_INFO("initialized openvr client");

    const auto manifestPath = utils::GetBindingsDir() / "att_actions.json";
    if (!std::filesystem::exists(manifestPath)) throw std::runtime_error("action manifest not found");

    ThrowIfError(vr::VRInput()->SetActionManifestPath(manifestPath.generic_string().c_str()));
    ThrowIfError(vr::VRInput()->GetActionHandle("/actions/demo/in/grab_camera", &mActionGrabCamera));
    ThrowIfError(vr::VRInput()->GetActionHandle("/actions/demo/in/grab_trackers", &mActionGrabTrackers));
    ThrowIfError(vr::VRInput()->GetActionHandle("/actions/demo/in/Hand_Left", &mActionLeftControllerPose));
    ThrowIfError(vr::VRInput()->GetActionSetHandle("/actions/demo", &mActionSet));
    ATT_LOG_INFO("openvr action manifest: ", manifestPath);
}

void OpenVRClient::Shutdown()
{
    if (!mContext) throw std::runtime_error("openvr not initialized");
    mContext.reset();
}

void OpenVRClient::PollEvents()
{
    if (!mContext) throw std::runtime_error("openvr not initialized");
    vr::VREvent_t event{};
    while (mContext->PollNextEvent(&event, sizeof(event)))
    {
        if (event.eventType == vr::VREvent_Quit)
        {
            // close connection to steamvr without closing att
            mContext->AcknowledgeQuit_Exiting();
            mContext.reset();
            return;
        }
    }
}

void OpenVRClient::UpdateInputActions() const
{
    if (!mContext) throw std::runtime_error("openvr not initialized");
    UpdateActionSetState(mActionSet);
}

ButtonAction OpenVRClient::GetButtonAction() const
{
    if (!mContext) throw std::runtime_error("openvr not initialized");
    if (GetDigitalState(mActionGrabCamera)) return ButtonAction::GrabCamera;
    if (GetDigitalState(mActionGrabTrackers)) return ButtonAction::GrabTrackers;
    return ButtonAction::None;
}

Pose OpenVRClient::GetControllerPoseAction() const
{
    if (!mContext) throw std::runtime_error("openvr not initialized");
    if (const auto pose = GetPoseForNextFrame(mActionLeftControllerPose))
    {
        return OVRPoseMatrixToPose(*pose);
    }
    return Pose::Ident();
}

std::optional<Pose> OpenVRClient::GetHMDPose() const
{
    if (!mContext) throw std::runtime_error("openvr not initialized");

    // PlayspaceCalib obtains controller poses in RawAndUncalibrated space. Use the
    // same universe here so future HMD/controller calibration compares like-for-like
    // poses without a SteamVR chaperone transform applied to only one input.
    constexpr vr::ETrackingUniverseOrigin origin = vr::TrackingUniverseRawAndUncalibrated;
    std::array<vr::TrackedDevicePose_t, vr::k_unTrackedDeviceIndex_Hmd + 1> trackedPoses{};
    mContext->GetDeviceToAbsoluteTrackingPose(
        origin, 0.0F, trackedPoses.data(), static_cast<uint32_t>(trackedPoses.size()));
    return OVRTrackedDevicePoseToPose(trackedPoses[vr::k_unTrackedDeviceIndex_Hmd]);
}

} // namespace tracker
