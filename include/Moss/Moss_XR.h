//                        MIT License
//
//                  Copyright (c) 2026 Toby
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:

// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.

// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

/*!
 * @file Moss_XR.h
 * @brief Cross-platform XR (VR/AR/MR) interface built on OpenXR 1.1 for Moss Engine.
 *
 * The Moss XR module provides a unified and extensible interface for Virtual Reality (VR),
 * Augmented Reality (AR), and Mixed Reality (MR) applications using the OpenXR standard.
 *
 * ---
 *
 * ### Core Features:
 * - **OpenXR 1.1 Integration** — Cross-platform, vendor-neutral XR standard (Oculus, SteamVR, WMR, Vive, Pico, Varjo, etc).
 * - **Session Management** — Handles initialization, frame lifecycle, and synchronization between rendering and XR runtimes.
 * - **View Configuration** — Supports both stereo (VR) and mono (AR/MR) rendering pipelines.
 * - **Input System** — Abstracted hand/controller tracking via OpenXR actions and poses.
 * - **Haptic Feedback** — Vibration, impulse, and frequency-based feedback through OpenXR haptics layer.
 * - **Tracking System** — Full 6DOF head, hand, and body tracking with prediction and smoothing.
 * - **Compositor Integration** — Direct connection to Moss Renderer for eye texture management and projection matrices.
 * - **Multi-View Rendering** — Support for multi-pass, instanced, or single-pass stereo rendering (depending on GPU/driver capabilities).
 *
 * ---
 *
 * ### Supported XR Runtimes:
 * - Meta Quest / Oculus (via OpenXR)
 * - SteamVR (Vive, Index)
 * - Windows and Windows Mixed Reality
 * - Varjo XR
 * - Pico / Lynx / WaveVR
 * - Android XR devices (via OpenXR Loader)
 * - Linux
 *
 * ---
 *
 * ### Supported Renderers: OpenGL, Vulkan, DirectX 12
 *
 * ---
 *
 * ### Extended Moss Features:
 * - **XR Camera Integration**  
 *   - `CameraXR` class synchronizes head pose and eye projections with the Moss rendering pipeline.
 *   - Automatic clipping and stereo culling support.
 *
 * - **Hand Tracking / Gesture API**  
 *   - Access joint transforms (e.g., palm, finger tips) through `Moss_XRGetHandJointPose()`.  
 *   - Custom gestures and interaction mapping for grabbing, pointing, UI input.  
 *
 * - **Eye Tracking & Foveated Rendering** *(optional)*  
 *   - Dynamic foveation using gaze direction.  
 *   - Adaptive resolution rendering for performance optimization.  
 *
 * - **Spatial Anchors & Scene Understanding** *(AR/MR)*  
 *   - World anchors for persistent spatial references.  
 *   - Mesh reconstruction and real-world collision surfaces for physics alignment.  
 *
 * ---
 *
 * ### Supported Extensions:
 * - `XR_EXT_hand_tracking`
 * - `XR_EXT_eye_gaze_interaction`
 * - `XR_FB_foveation`
 * - `XR_VARJO_quad_views`
 * - `XR_EXT_local_floor`
 * - `XR_KHR_composition_layer_depth`
 *
 * ---
 *
 * ### Future Roadmap:
 * - Mixed Reality passthrough with real-time camera streaming.
 * - Vulkan-based OpenXR layer compositor.
 * - GPU-driven foveation and reprojection.
 * - Networked multi-user XR sessions (shared space).
 * - Haptic feedback synthesis via Moss_Haptics.
 *  
 * ---
 *
 * ### Design Goals:
 * - Fully modular and OpenXR-conformant.  
 * - Minimal runtime overhead — integrates directly with Moss’ job system and renderer.  
 * - Real-time, low-latency VR/AR interaction.  
 * - Plug-and-play with custom runtime layers and extensions (e.g. Varjo gaze, Meta passthrough).  
*/

// WARNING I don't have a headset to test this code, please if you have one let me know if theres any problems.

#ifndef MOSS_XR_H
#define MOSS_XR_H

#include <Moss/Moss_stdinc.h>
#include <Moss/Moss_Renderer.h>

/* 
Swapchains & renderer
- Passthrough layers fully integrated.
- Depth/composition layers for mixed reality or advanced rendering.
- Frame graph management: currently, layers are stored in g_XR.layers but no prioritization/ordering, no multi-layer passes.

Input / Actions
- Full input profile auto-binding for common controllers (HTC Vive, Meta Quest).
- Subaction paths per hand for actions (currently hardcoded XR_NULL_PATH).
- Eye gaze / head gestures support.

Hand / Body / Face Tracking
- HTC / Meta runtime support fully integrated (per-hand joint updates).
- Body joint tracking (spine, limbs) with vendor-specific APIs.
- Face tracking with blendshapes/facial expressions.
- Eye tracking support (if using XR_EXT_eye_gaze_interaction).

Frame / Timing
- Frame graph formalization: ordering multiple layers (projection, quad, passthrough).
- Multi-threaded rendering integration (if required).

Capabilities / Extensions
- Per-feature capability probing at runtime (detecting whether body tracking, hand tracking, foveation, passthrough are actually supported).
- Validation layer support for debugging OpenXR calls.

Debug / Utilities
- Performance markers / frame timing integration.
- Optional verbose logging for vendor-specific extensions.
*/

/* ======================================================
 * Forward Declarations
 * =================================================== */
struct MossXR_Context;
struct MossXR_Session;
struct MossXR_Swapchain;
struct MossXR_Space;
struct MossXR_Action;
struct MossXR_ActionSet;
struct MossXR_Layer;
struct MossXR_Anchor;
struct MossXR_Origin;

struct MossXR_HandModifier;   // Hand Modifier tracks Hand movement.
struct MossXR_BodyModifier;   // Body Modifier tracks body.
struct MossXR_FaceModifier;   // Facial Modifier tracks facial expressions.

using MossXR_Time = int64_t;

/* ======================================================
 * Enums
 * =================================================== */

enum class MossXR_SessionState : uint8_t {
    Unknown,
    Ready,
    Synchronized,
    Visible,
    Focused,
    Stopping,
    Exiting,
    LossPending
};

enum class MossXR_ActionType : uint8_t {
    Boolean,
    Float,
    Vec2,
    Pose,
    Haptic
};

enum class MossXR_ViewType : uint8_t {
    Mono,
    Stereo
};

enum class MossXR_Handedness : uint8_t {
    Left,
    Right
};


enum class Moss_XRImageLayout {
    COLOR_ATTACHMENT,
    SHADER_READ,
};


enum class Moss_XREventType {
    SESSION_STATE_CHANGED,
    USER_PRESENCE_CHANGED,
    REFERENCE_SPACE_CHANGED,
    INSTANCE_LOSS_PENDING,
    INTERACTION_PROFILE_CHANGED,
    VISIBILITY_CHANGED,
};

enum class XRBodyJoint {
    XR_BODY_ROOT,
    XR_BODY_SPINE,
    XR_BODY_CHEST,
    XR_BODY_NECK,
    XR_BODY_HEAD,
    XR_BODY_LEFT_SHOULDER,
    XR_BODY_LEFT_ELBOW,
    XR_BODY_LEFT_HAND,
    XR_BODY_RIGHT_SHOULDER,
    XR_BODY_RIGHT_ELBOW,
    XR_BODY_RIGHT_HAND,
    XR_BODY_LEFT_HIP,
    XR_BODY_LEFT_KNEE,
    XR_BODY_LEFT_FOOT,
    XR_BODY_RIGHT_HIP,
    XR_BODY_RIGHT_KNEE,
    XR_BODY_RIGHT_FOOT,
    XR_BODY_JOINT_COUNT
};

enum class XRHandJoint {
    XR_HAND_WRIST,
    XR_HAND_THUMB_METACARPAL,
    XR_HAND_THUMB_PROXIMAL,
    XR_HAND_THUMB_DISTAL,
    XR_HAND_THUMB_TIP,
    XR_HAND_INDEX_METACARPAL,
    XR_HAND_INDEX_PROXIMAL,
    XR_HAND_INDEX_INTERMEDIATE,
    XR_HAND_INDEX_DISTAL,
    XR_HAND_INDEX_TIP,
    XR_HAND_MIDDLE_METACARPAL,
    XR_HAND_MIDDLE_PROXIMAL,
    XR_HAND_MIDDLE_INTERMEDIATE,
    XR_HAND_MIDDLE_DISTAL,
    XR_HAND_MIDDLE_TIP,
    XR_HAND_RING_METACARPAL,
    XR_HAND_RING_PROXIMAL,
    XR_HAND_RING_INTERMEDIATE,
    XR_HAND_RING_DISTAL,
    XR_HAND_RING_TIP,
    XR_HAND_PINKY_METACARPAL,
    XR_HAND_PINKY_PROXIMAL,
    XR_HAND_PINKY_INTERMEDIATE,
    XR_HAND_PINKY_DISTAL,
    XR_HAND_PINKY_TIP,
    XR_HAND_JOINT_COUNT
};



/* ======================================================
 * Math / Pose
 * =================================================== */

struct MossXR_Pose {
    Vec3 position;
    Quat orientation;
};

struct MossXR_Fov {
    float left;
    float right;
    float up;
    float down;
};


/* ======================================================
 * View / Camera
 * =================================================== */

struct MossXR_View {
    MossXR_Pose pose;
    MossXR_Fov  fov;
    Mat44      view;
    Mat44      projection;
};



/* ======================================================
 * Capabilities
 * =================================================== */

struct MossXR_Capabilities {
    uint32_t viewCount;
    MossXR_ViewType viewType;

    bool handTracking;
    bool eyeTracking;
    bool bodyTracking;
    bool faceTracking;

    bool passthrough;
    bool anchors;
    bool depthLayers;
};

/* ======================================================
 * Instance
 * =================================================== */
/**
 * @brief Initializes the Moss XR subsystem using the supplied Moss renderer. @param renderer Renderer used to integrate XR rendering with Moss. Must remain valid as required by the implementation. @return `true` if XR initialization succeeds; otherwise `false`.
 */
MOSS_API bool Moss_XR_Initialize(Moss_Renderer* renderer);
/**
 * @brief Shuts down the Moss XR subsystem and releases its owned resources. Call when XR functionality is no longer needed.
 */
MOSS_API void Moss_XR_Shutdown(void);

/**
 * @brief Returns the capabilities reported by the initialized XR subsystem. @return Pointer to the capability description, or `nullptr` if no capability data is available. The returned pointer is owned by the subsystem.
 */
MOSS_API const MossXR_Capabilities* Moss_XR_GetCapabilities(void);
/**
 * @brief Enables OpenXR validation layers when supported by the runtime and build configuration. @return `true` if validation layers are enabled; otherwise `false`.
 */
MOSS_API bool Moss_XR_EnableValidationLayers();
/* ======================================================
 * Session
 * =================================================== */
/**
 * @brief Creates an XR session for the active XR instance. @return Pointer to the created session, or `nullptr` if session creation fails.
 */
MOSS_API MossXR_Session* Moss_XR_CreateSession(void);
/**
 * @brief Destroys an XR session and releases resources owned by it. @param session Session to destroy. A null pointer should be treated as invalid unless the implementation explicitly permits it.
 */
MOSS_API void Moss_XR_DestroySession(MossXR_Session* session);
/*! @brief Returns the current lifecycle state of an XR session. @param session Session whose state is queried. @return Current session state, or `MossXR_SessionState::Unknown` when the state cannot be determined.
 */
MOSS_API MossXR_SessionState Moss_XR_GetSessionState(MossXR_Session* session);

// Frame Loop
/**
 * @brief Begins an XR frame and performs the frame-start synchronization required by the runtime.
 * @param session Session for which to begin the frame. @return `true` if the frame can proceed; otherwise `false`.
 */
MOSS_API bool Moss_XR_BeginFrame(MossXR_Session* session);
/**
 * @brief Ends the current XR frame and submits frame completion to the runtime. @param session Session whose current frame is being ended.
 */
MOSS_API void Moss_XR_EndFrame(MossXR_Session* session);
/**
 * @brief Gets the predicted display time for the session's current frame. @param session Session whose predicted display time is requested. @return Predicted display time in the XR time representation used by this API.
 */
MOSS_API MossXR_Time Moss_XR_GetPredictedDisplayTime(MossXR_Session* session);
/**
 * @brief Returns the XR frame delta time in seconds. @return Frame delta time in seconds.
 */
MOSS_API float Moss_XR_GetDeltaSeconds(void);

/**
 * @brief Polls and processes pending XR runtime events for the session. @param session Session whose events should be handled.
 */
MOSS_API void Moss_XR_HandleEvents(MossXR_Session* session);

// Views
/**
 * @brief Returns the number of views configured for the session. @param session Session to query. @return Number of views, such as the views used for mono or stereo rendering.
 */
MOSS_API uint32_t Moss_XR_GetViewCount(MossXR_Session* session);
/*! @brief Retrieves the pose, field of view, and view/projection matrices for one XR view. @param session Session to query.
 * @param index Zero-based index of the requested view. @param outView Output structure that receives the view data; must point to writable storage. @return `true` if the view data was retrieved; otherwise `false`.
 */
MOSS_API bool Moss_XR_GetView(MossXR_Session* session, uint32_t index, MossXR_View* outView);


/* ======================================================
 * Layers
 * =================================================== */
// Swapchains

/* ======================================================
 * Actions
 * =================================================== */
/**
 * @brief Creates an action set with the supplied name. @param name Name identifying the action set. @return Pointer to the created action set, or `nullptr` if creation fails.
 */
MOSS_API MossXR_ActionSet* Moss_XR_CreateActionSet(const char* name);
/** @brief Destroys an action set and releases its resources. @param set Action set to destroy.
 */
MOSS_API void Moss_XR_DestroyActionSet(MossXR_ActionSet* set);
/**
 * @brief Creates an action of the specified type in an action set. @param set Action set that will contain the action.
 * @param name Name identifying the action. @param type Value type and interaction category of the action.
 * @return Pointer to the created action, or `nullptr` if creation fails.
 */
MOSS_API MossXR_Action* Moss_XR_CreateAction(MossXR_ActionSet* set, const char* name, MossXR_ActionType type);
/**
 * @brief Destroys an action and releases its resources. @param action Action to destroy.
 */
MOSS_API void Moss_XR_DestroyAction(MossXR_Action* action);
/**
 * @brief Attaches an action set to a session so its actions can be used by that session. @param session Session to which the action set is attached. @param set Action set to attach.
 */
MOSS_API void Moss_XR_AttachActionSet(MossXR_Session* session, MossXR_ActionSet* set);
/*! @brief Synchronizes the session's attached actions with the current XR input state. @param session Session whose actions should be synchronized.
 */
MOSS_API void Moss_XR_SyncActions(MossXR_Session* session);
/**
 * @brief Reads the current Boolean value of an action. @param action Boolean action to query. @return Current action value. The declaration does not expose a separate error status.
 */
MOSS_API bool  Moss_XR_GetActionBoolean(MossXR_Action* action);
/**
 * @brief Reads the current floating-point value of an action. @param action Float action to query. @return Current action value. The declaration does not expose a separate error status.
 */
MOSS_API float Moss_XR_GetActionFloat(MossXR_Action* action);
/**
 * @brief Retrieves the current pose associated with a pose action.
 * @param action Pose action to query. @param[out] outPose Receives the action pose when available. @return `true` if a pose was retrieved; otherwise `false`.
 */
MOSS_API bool  Moss_XR_GetActionPose(MossXR_Action* action, MossXR_Pose* outPose);
/**
 * @brief Plays haptic feedback for an action and the specified hand.
 * @param action Haptic action used to issue the feedback. @param hand OpenXR path identifying the hand or subaction receiving feedback.
 * @param amplitude Requested vibration amplitude, according to the backend's supported range. @param durationSeconds Requested feedback duration in seconds.
 * @note This declaration is currently commented out and is not part of the compiled API.
 */
//MOSS_API void Moss_XR_PlayHaptic(MossXR_Action* action, XrPath hand, float amplitude, float durationSeconds);
/**
 * @brief Stops haptic output associated with an action.
 * @param action Haptic action whose output should be stopped.
 */
MOSS_API void Moss_XR_StopHaptic(MossXR_Action* action);

/* ======================================================
 * Tracking Objects
 * =================================================== */
/**
 * @brief Creates a hand-tracking modifier or handle. @return Pointer to the created hand object, or `nullptr` if creation fails.
 */
MOSS_API MossXR_HandModifier* Moss_XR_CreateHand(void);
/**
 * @brief Retrieves the tracked pose of a specified hand joint.
 * @param hand Hand-tracking object to query. @param joint Joint whose pose is requested. @param[out] outPose Receives the joint pose when available.
 * @return `true` if the joint pose was retrieved; otherwise `false`.
 */
MOSS_API bool Moss_XR_GetHandJointPose(MossXR_HandModifier* hand, XRHandJoint joint, MossXR_Pose* outPose);
/**
 * @brief Destroys a hand-tracking object and releases its resources.
 * @param hand Hand object to destroy.
 */
MOSS_API void Moss_XR_DestroyHand(MossXR_HandModifier* hand);
/**
 * @brief Retrieves the overall pose reported for a tracked hand.
 * @param hand Hand-tracking object to query. @param[out] outPose Receives the hand pose when available.
 * @return `true` if the pose was retrieved; otherwise `false`.
 */
MOSS_API bool Moss_XR_GetHandPose(MossXR_HandModifier* hand, MossXR_Pose* outPose);
/**
 * @brief Creates a body-tracking modifier or handle. @return Pointer to the created body object, or `nullptr` if creation fails.
 */
MOSS_API MossXR_BodyModifier* Moss_XR_CreateBody(void);
/**
 * @brief Retrieves the tracked pose of a specified body joint.
 * @param body Body-tracking object to query. @param joint Body joint whose pose is requested.
 * @param[out] outPose Receives the joint pose when available. @return `true` if the joint pose was retrieved; otherwise `false`.
 */
MOSS_API bool Moss_XR_GetBodyJointPose(MossXR_BodyModifier* body, XRBodyJoint joint, MossXR_Pose* outPose);
/**
 * @brief Destroys a body-tracking object and releases its resources.
 * @param body Body object to destroy.
 */
MOSS_API void Moss_XR_DestroyBody(MossXR_BodyModifier* body);
/**
 * @brief Creates a face-tracking modifier or handle.
 * @return Pointer to the created face object, or `nullptr` if creation fails.
 */
MOSS_API MossXR_FaceModifier* Moss_XR_CreateFace(void);
/**
 * @brief Destroys a face-tracking object and releases its resources.
 * @param face Face object to destroy.
 */
MOSS_API void Moss_XR_DestroyFace(MossXR_FaceModifier* face);
/**
 * @brief Updates body-tracking data for the session.
 * @param session Session whose body-tracking state should be updated.
 */
MOSS_API void Moss_XR_UpdateBodyTracking(MossXR_Session* session);
/**
 * @brief Updates hand-tracking data for the session.
 * @param session Session whose hand-tracking state should be updated.
 */
MOSS_API void Moss_XR_UpdateHandTracking(MossXR_Session* session);

/* ======================================================
 * Passthrough & Foveation
 * =================================================== */
/*! @brief Enables or disables XR passthrough, if supported by the active runtime. @param enable `true` to request passthrough; `false` to disable it. @return `true` if the requested state is supported and applied; otherwise `false`.
 */
MOSS_API bool Moss_XR_EnablePassthrough(bool enable);
/**
 * @brief Enables or disables foveated rendering, if supported by the active runtime and renderer. @param enable `true` to request foveated rendering; `false` to disable it.
 * @return `true` if the requested state is supported and applied; otherwise `false`.
 */
MOSS_API bool Moss_XR_EnableFoveatedRendering(bool enable);

/* ======================================================
 * Backend
 * =================================================== */
/**
 * @brief Returns the name of the active XR backend. @return Null-terminated backend name, or `nullptr` if no backend name is available. The returned string is owned by the subsystem.
 */
MOSS_API const char* Moss_XR_GetBackendName(void);
/**
 * @brief Returns the version number reported by the active XR backend. @return Backend version encoded according to the implementation's versioning convention.
 */
MOSS_API uint32_t Moss_XR_GetBackendVersion(void);

/* ======================================================
 * Debug
 * =================================================== */
/*! @brief Begins a debug label scope for XR operations, when supported by the backend. @param label Human-readable label for the scope. */
MOSS_API void Moss_XR_BeginDebugLabel(const char* label);
/*! @brief Ends the most recently begun XR debug label scope, when supported by the backend. */
MOSS_API void Moss_XR_EndDebugLabel(void);

#endif // MOSS_XR_H
