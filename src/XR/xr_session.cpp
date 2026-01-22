// Moss_XR.cpp
#include <Moss/Moss_XR.h>
#include <Moss/XR/xr_intern.h>

#include <openxr/openxr.h>
#include <openxr/openxr_platform.h>

#include <vector>
#include <cstring>

static XrTime g_lastPredictedTime = 0;
static float g_deltaSeconds = 0.0f;


MossXR_Session* Moss_XR_CreateSession(void) {
    if (!g_XR.context) return nullptr;

    MossXR_Session* s = new MossXR_Session();

    XrSessionCreateInfo sci{XR_TYPE_SESSION_CREATE_INFO};
    sci.systemId = g_XR.context->systemId;

    XrResult res = xrCreateSession(
        g_XR.context->instance,
        &sci,
        &s->session
    );

    if (res != XR_SUCCESS) {
        delete s;
        return nullptr;
    }

    // Create local space
    XrReferenceSpaceCreateInfo rci{XR_TYPE_REFERENCE_SPACE_CREATE_INFO};
    rci.referenceSpaceType = XR_REFERENCE_SPACE_TYPE_LOCAL;
    rci.poseInReferenceSpace.orientation.w = 1.0f;

    xrCreateReferenceSpace(s->session, &rci, &s->localSpace);
    s->appSpace = s->localSpace;

    g_XR.session = s;
    return s;
}

void Moss_XR_DestroySession(MossXR_Session* session) {
    if (!session) return;

    xrDestroySpace(session->localSpace);
    xrDestroySession(session->session);

    if (g_XR.session == session)
        g_XR.session = nullptr;

    delete session;
}

bool Moss_XR_BeginFrame(MossXR_Session* session) {
    XrFrameWaitInfo wait { XR_TYPE_FRAME_WAIT_INFO };
    xrWaitFrame(session->xrSession, &wait, &g_XR.frameState.xrFrameState);

    XrFrameBeginInfo begin { XR_TYPE_FRAME_BEGIN_INFO };
    xrBeginFrame(session->xrSession, &begin);

    if (g_lastTime)
        g_deltaSeconds = float((s->frameState.predictedDisplayTime - g_lastTime) * 1e-9);

    g_lastTime = s->frameState.predictedDisplayTime;
    return true;
}

void Moss_XR_EndFrame(MossXR_Session* session) {
    if (!session) return;

    std::vector<XrCompositionLayerBaseHeader*> xrLayers;
    for (auto* l : g_XR.layers)
        xrLayers.push_back(l->xrLayer);

    XrFrameEndInfo end{XR_TYPE_FRAME_END_INFO};
    end.displayTime =
        session->frameState.predictedDisplayTime;
    end.environmentBlendMode = XR_ENVIRONMENT_BLEND_MODE_OPAQUE;
    end.layerCount = (uint32_t)xrLayers.size();
    end.layers = xrLayers.data();

    xrEndFrame(session->session, &end);
}


uint32_t Moss_XR_GetViewCount(MossXR_Session*) {
    return 2; // stereo HMD default
}


/*! @brief X @param X */
MossXR_Time Moss_XR_GetPredictedDisplayTime() {
    if (!g_ctx) return 0;
    XrTime predictedTime = g_ctx->frameState.xrFrameState.predictedDisplayTime;
    if (g_lastPredictedTime != 0)
        g_deltaSeconds = (float)((predictedTime - g_lastPredictedTime) * 1e-9);
    g_lastPredictedTime = predictedTime;
    return predictedTime;
}

/*! @brief X @return */
float Moss_XR_GetDeltaSeconds(void) { return g_deltaSeconds; }


bool Moss_XR_GetView(MossXR_Session* session, uint32_t index, MossXR_View* outView) {
    if (!session || !outView || index >= 2)
        return false;

    XrView views[2]{{XR_TYPE_VIEW},{XR_TYPE_VIEW}};
    XrViewLocateInfo vli{XR_TYPE_VIEW_LOCATE_INFO};
    vli.viewConfigurationType = XR_VIEW_CONFIGURATION_TYPE_PRIMARY_STEREO;
    vli.displayTime = session->frameState.predictedDisplayTime;
    vli.space = session->appSpace;

    XrViewState vs{XR_TYPE_VIEW_STATE};
    uint32_t count = 0;

    xrLocateViews(session->session, &vli, &vs, 2, &count, views);

    const XrView& v = views[index];

    outView->pose.position = {v.pose.position.x, v.pose.position.y, v.pose.position.z};
    outView->pose.orientation = {
        v.pose.orientation.x,
        v.pose.orientation.y,
        v.pose.orientation.z,
        v.pose.orientation.w
    };

    outView->fov.left  = v.fov.angleLeft;
    outView->fov.right = v.fov.angleRight;
    outView->fov.up    = v.fov.angleUp;
    outView->fov.down  = v.fov.angleDown;

    return true;
}

MossXR_SessionState Moss_XR_GetSessionState(MossXR_Session* session) {
    if (!session) return MOSS_XR_SESSION_UNKNOWN;
    return session->state;
}





void Moss_XR_HandleEvents(MossXR_Session* session) {
    XrEventDataBuffer event{XR_TYPE_EVENT_DATA_BUFFER};
    while (xrPollEvent(g_XR.context->instance, &event) == XR_SUCCESS) {
        switch (event.type) {
            case XR_TYPE_EVENT_DATA_SESSION_STATE_CHANGED: {
                auto* ev = (XrEventDataSessionStateChanged*)&event;
                g_XR.sessionState = (MossXR_SessionState)ev->state;
                if (g_XR.sessionState == XR_SESSION_STATE_READY) {
                    XrSessionBeginInfo beginInfo{XR_TYPE_SESSION_BEGIN_INFO};
                    beginInfo.primaryViewConfigurationType = XR_VIEW_CONFIGURATION_TYPE_PRIMARY_STEREO;
                    xrBeginSession(session->xrSession, &beginInfo);
                }
                if (g_XR.sessionState == XR_SESSION_STATE_STOPPING ||
                    g_XR.sessionState == XR_SESSION_STATE_EXITING) {
                    session->running = false;
                }
                break;
            }
            default: break;
        }
        event.type = XR_TYPE_EVENT_DATA_BUFFER;
    }
}
