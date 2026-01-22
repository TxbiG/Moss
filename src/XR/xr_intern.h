#pragma once

#include <Moss/Moss_XR.h>

#include <openxr/openxr.h>
#include <openxr/openxr_platform.h>

#include <vector>
#include <unordered_map>
#include <string>


struct MossXR_Capabilities {
    bool handTracking        = false;
    bool bodyTracking        = false;
    bool faceTracking        = false;
    bool eyeTracking         = false;
    bool foveatedRendering   = false;
    bool passthrough         = false;
    bool spatialAnchors      = false;
    bool planeDetection      = false;
    bool advancedHaptics     = false;
    bool depthLayers         = false;
};

struct MossXR_Context {
    // OpenXR core
    XrInstance instance = XR_NULL_HANDLE;
    XrSystemId systemId = XR_NULL_SYSTEM_ID;

    // Enabled extensions
    bool handTrackingSupported = false;
    bool eyeTrackingSupported  = false;
    bool bodyTrackingSupported = false;
    bool faceTrackingSupported = false;
    bool passthroughSupported  = false;
    bool anchorSupported       = false;

    // Graphics binding (opaque to public API)
    XrGraphicsBindingVulkanKHR   vkBinding{};
    XrGraphicsBindingOpenGLKHR   glBinding{};
    XrGraphicsBindingD3D12KHR    dx12Binding{};

    MossXR_Capabilities capabilities{};
};

struct MossXR_Session {
    XrSession session = XR_NULL_HANDLE;

    // Spaces
    XrSpace appSpace   = XR_NULL_HANDLE;   // XR_REFERENCE_SPACE_TYPE_STAGE / LOCAL
    XrSpace localSpace = XR_NULL_HANDLE;
    XrSpace viewSpace  = XR_NULL_HANDLE;

    // Frame lifecycle
    XrFrameState frameState{XR_TYPE_FRAME_STATE};
    XrSessionState xrState = XR_SESSION_STATE_UNKNOWN;
    MossXR_SessionState mossState = MossXR_SessionState::Unknown;

    bool running = false;
};



struct MossXR_Swapchain {
    XrSwapchain swapchain = XR_NULL_HANDLE;

    uint32_t width  = 0;
    uint32_t height = 0;
    uint32_t imageCount = 0;

    std::vector<XrSwapchainImageBaseHeader*> images;
};



struct MossXR_Space {
    XrSpace space = XR_NULL_HANDLE;
};

struct MossXR_Origin {
    MossXR_Space* space = nullptr;
};


struct MossXR_Action {
    XrAction action = XR_NULL_HANDLE;
    MossXR_ActionType type;

    // Cached state (updated during xrSyncActions)
    bool boolState = false;
    float floatState = 0.f;
    XrVector2f vec2State{};
    XrPosef poseState{};
};

struct MossXR_ActionSet {
    XrActionSet actionSet = XR_NULL_HANDLE;
    std::unordered_map<std::string, MossXR_Action*> actions;
};




struct MossXR_HandModifier {
    bool supported = false;
    bool hasVelocity = false;

    XrHandEXT hand = XR_HAND_LEFT_EXT;
    XrHandTrackerEXT tracker = XR_NULL_HANDLE;

    XrHandJointLocationEXT joints[XR_HAND_JOINT_COUNT_EXT];

    // Optional (XR_EXT_hand_tracking_velocity)
    XrHandJointVelocityEXT velocities[XR_HAND_JOINT_COUNT_EXT];
};


enum MossXR_BodyBackend {
    MOSS_XR_BODY_NONE,
    MOSS_XR_BODY_HTC,
    MOSS_XR_BODY_META
};

struct MossXR_BodyModifier {
    bool supported = false;
    MossXR_BodyBackend backend = MOSS_XR_BODY_NONE;

    // --- HTC ---
    XrBodyTrackerHTC htcTracker = XR_NULL_HANDLE;
    XrBodyJointLocationHTC htcJoints[XR_BODY_JOINT_COUNT_HTC];

    // --- Meta ---
    XrBodyTrackerFB metaTracker = XR_NULL_HANDLE;
    XrBodyJointLocationFB metaJoints[XR_BODY_JOINT_COUNT_FB];
};


enum MossXR_FaceBackend {
    MOSS_XR_FACE_NONE,
    MOSS_XR_FACE_META,
    MOSS_XR_FACE_HTC
};

struct MossXR_FaceModifier {
    bool supported = false;
    MossXR_FaceBackend backend = MOSS_XR_FACE_NONE;

    // Meta (XR_FB_face_tracking)
    float metaWeights[XR_FACE_EXPRESSION_COUNT_FB];

    // HTC (XR_HTC_facial_tracking)
    float htcWeights[XR_FACIAL_EXPRESSION_COUNT_HTC];
};


struct MossXR_Anchor {
    XrSpace anchorSpace = XR_NULL_HANDLE;
};



struct MossXR_Layer {
    MossXR_LayerType type;
    int priority;

    MossXR_Swapchain* swapchain;

    XrCompositionLayerBaseHeader* xrLayer;
    union {
        XrCompositionLayerProjection projection;
        XrCompositionLayerQuad quad;
    };
};

struct MossXR_Runtime {
    MossXR_Context* context = nullptr;
    MossXR_Session* session = nullptr;

    std::vector<MossXR_ActionSet*> actionSets;
    std::vector<MossXR_Swapchain*> swapchains;
    std::vector<MossXR_Layer*> layers;

    MossXR_HandModifier* leftHand  = nullptr;
    MossXR_HandModifier* rightHand = nullptr;
    MossXR_BodyModifier* body      = nullptr;
    MossXR_FaceModifier* face      = nullptr;
};


struct MossXR_Frame {
    std::vector<MossXR_Layer*> layers;
    XrTime predictedDisplayTime;
};

static MossXR_Frame g_currentFrame;

inline XrTime MossXR_ToXrTime(MossXR_Time time) {
    return static_cast<XrTime>(time);
}

inline MossXR_Time MossXR_FromXrTime(XrTime time) {
    return static_cast<MossXR_Time>(time);
}





// Helper
void SortLayersByPriority(std::vector<MossXR_Layer*>& layers);