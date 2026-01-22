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

typedef struct MossXR_Context        MossXR_Context;
typedef struct MossXR_Session        MossXR_Session;
typedef struct MossXR_Swapchain      MossXR_Swapchain;
typedef struct MossXR_Space          MossXR_Space;
typedef struct MossXR_Action         MossXR_Action;
typedef struct MossXR_ActionSet      MossXR_ActionSet;
typedef struct MossXR_Layer          MossXR_Layer;
typedef struct MossXR_Anchor         MossXR_Anchor;
typedef struct MossXR_Origin         MossXR_Origin;


typedef struct MossXR_HandModifier   MossXR_HandModifier;   // Hand Modifier tracks Hand movement.
typedef struct MossXR_BodyModifier   MossXR_BodyModifier;   // Body Modifier tracks body.
typedef struct MossXR_FaceModifier   MossXR_FaceModifier;   // Facial Modifier tracks facial expressions.

typedef int64_t MossXR_Time;

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

typedef struct MossXR_Pose {
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

struct MossXR_InitInfo {
    ERendererBackend renderer;
    void* graphicsDevice;
    void* graphicsContext;
};

/* ======================================================
 * Instance
 * =================================================== */
/*! @brief X */
MOSS_API bool Moss_XR_Initialize(const MossXR_InitInfo* info);
/*! @brief X */
MOSS_API void Moss_XR_Shutdown(void);

/*! @brief X @param X */
MOSS_API const MossXR_Capabilities* Moss_XR_GetCapabilities(void);
/*! @brief X @param X */
MOSS_API bool Moss_XR_EnableValidationLayers();
/* ======================================================
 * Session
 * =================================================== */
/*! @brief X */
MOSS_API MossXR_Session* Moss_XR_CreateSession(void);
/*! @brief X @param X */
MOSS_API void Moss_XR_DestroySession(MossXR_Session* session);
/*! @brief X @param X */
MOSS_API MossXR_SessionState Moss_XR_GetSessionState(MossXR_Session* session);

// Frame Loop
/*! @brief X @param X */
MOSS_API bool Moss_XR_BeginFrame(MossXR_Session* session);
/*! @brief X @param X */
MOSS_API void Moss_XR_EndFrame(MossXR_Session* session);
/*! @brief X @param X */
MOSS_API MossXR_Time Moss_XR_GetPredictedDisplayTime(MossXR_Session* session);
/*! @brief X @return */
MOSS_API float Moss_XR_GetDeltaSeconds(void);

/*! @brief XR Poll events */
MOSS_API void Moss_XR_HandleEvents(MossXR_Session* session);

// Views
/*! @brief X @param X */
MOSS_API uint32_t Moss_XR_GetViewCount(MossXR_Session* session);
/*! @brief X @param X */
MOSS_API bool Moss_XR_GetView(MossXR_Session* session, uint32_t index, MossXR_View* outView);


/* ======================================================
 * Layers
 * =================================================== */
// Swapchains
/*! @brief X @param X */
MOSS_API MossXR_Swapchain* Moss_XR_CreateSwapchain(MossXR_Session* session, uint32_t width, uint32_t height, ETextureFormat format);
/*! @brief X @param X */
MOSS_API void Moss_XR_DestroySwapchain(MossXR_Swapchain* swapchain);
/*! @brief X @param X */
MOSS_API void* Moss_XR_AcquireSwapchainImage(MossXR_Swapchain* swapchain, uint32_t* outImageIndex);
/*! @brief X @param X */
MOSS_API void Moss_XR_ReleaseSwapchainImage(MossXR_Swapchain* swapchain, uint32_t imageIndex);

// Anchors
/*! @brief X @param X */
MOSS_API MossXR_Anchor* Moss_XR_CreateAnchor(const MossXR_Pose* pose);
/*! @brief X @param X */
MOSS_API void Moss_XR_DestroyAnchor(MossXR_Anchor* anchor);
/*! @brief X @param X */
MOSS_API bool Moss_XR_LocateAnchor( MossXR_Anchor* anchor, MossXR_Time time, MossXR_Pose* outPose);

// Composition Layers
/*! @brief X @param X */
MOSS_API MossXR_Layer* Moss_XR_CreateProjectionLayer(MossXR_Swapchain* swapchain);
/*! @brief X @param X */
MOSS_API MossXR_Layer* Moss_XR_CreateQuadLayer(MossXR_Swapchain* swapchain, const MossXR_Pose* pose, Vec2 size);
/*! @brief X @param X */
MOSS_API void Moss_XR_DestroyLayer(MossXR_Layer* layer);
/*! @brief X @param X */
MOSS_API void Moss_XR_SubmitLayers(MossXR_Session* session, uint32_t layerCount, MossXR_Layer** layers);

/* ======================================================
 * Actions
 * =================================================== */
/*! @brief X @param X */
MOSS_API MossXR_ActionSet* Moss_XR_CreateActionSet(const char* name);
/*! @brief X @param X */
MOSS_API void Moss_XR_DestroyActionSet(MossXR_ActionSet* set);
/*! @brief X @param X */
MOSS_API MossXR_Action* Moss_XR_CreateAction(MossXR_ActionSet* set, const char* name, MossXR_ActionType type);
/*! @brief X @param X */
MOSS_API void Moss_XR_DestroyAction(MossXR_Action* action);
/*! @brief X @param X */
MOSS_API void Moss_XR_AttachActionSet(MossXR_Session* session, MossXR_ActionSet* set);
/*! @brief X @param X */
MOSS_API void Moss_XR_SyncActions(MossXR_Session* session);
/*! @brief X @param X */
MOSS_API bool  Moss_XR_GetActionBoolean(MossXR_Action* action);
/*! @brief X @param X */
MOSS_API float Moss_XR_GetActionFloat(MossXR_Action* action);
/*! @brief X @param X */
MOSS_API bool  Moss_XR_GetActionPose(MossXR_Action* action, MossXR_Pose* outPose);
/*! @brief X @param X */
MOSS_API void Moss_XR_PlayHaptic(MossXR_Action* action, XrPath hand, float amplitude, float durationSeconds);
/*! @brief X @param X */
MOSS_API void Moss_XR_StopHaptic(MossXR_Action* action);

/* ======================================================
 * Tracking Objects
 * =================================================== */
/*! @brief X @return X*/
MOSS_API MossXR_HandModifier* Moss_XR_CreateHand(void);
/*! @brief X @param X */
MOSS_API bool Moss_XR_GetHandJointPose(MossXR_HandModifier* hand, XRHandJoint joint, MossXR_Pose* outPose);
/*! @brief X @param X */
MOSS_API void Moss_XR_DestroyHand(MossXR_HandModifier* hand);
/*! @brief X @param X */
MOSS_API bool Moss_XR_GetHandPose(MossXR_HandModifier* hand, MossXR_Pose* outPose);
/*! @brief X @return X*/
MOSS_API MossXR_BodyModifier* Moss_XR_CreateBody(void);
/*! @brief X @param X */
MOSS_API bool Moss_XR_GetBodyJointPose(MossXR_BodyModifier* body, XRBodyJoint joint, MossXR_Pose* outPose);
/*! @brief X @param X */
MOSS_API void Moss_XR_DestroyBody(MossXR_BodyModifier* body);
/*! @brief X @return X*/
MOSS_API MossXR_FaceModifier* Moss_XR_CreateFace(void);
/*! @brief X @param X */
MOSS_API void Moss_XR_DestroyFace(MossXR_FaceModifier* face);
/*! @brief X @param X */
void Moss_XR_UpdateBodyTracking(MossXR_Session* session);
/*! @brief X @param X */
void Moss_XR_UpdateHandTracking(MossXR_Session* session);

/* ======================================================
 * Passthrough & Foveation
 * =================================================== */
/*! @brief X @param X */
MOSS_API bool Moss_XR_EnablePassthrough(bool enable);
/*! @brief X @param X */
MOSS_API bool Moss_XR_EnableFoveatedRendering(bool enable);

/* ======================================================
 * Backend
 * =================================================== */
/*! @brief X @return X*/
MOSS_API const char* Moss_XR_GetBackendName(void);
/*! @brief X @return X*/
MOSS_API uint32_t Moss_XR_GetBackendVersion(void);

/* ======================================================
 * Debug
 * =================================================== */
/*! @brief X @param X */
MOSS_API void Moss_XR_BeginDebugLabel(const char* label);
/*! @brief X */
MOSS_API void Moss_XR_EndDebugLabel(void);

#endif // MOSS_XR_H