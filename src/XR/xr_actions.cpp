// Moss_XR.cpp
#include <Moss/Moss_XR.h>
#include <Moss/XR/xr_intern.h>

#include <openxr/openxr.h>
#include <openxr/openxr_platform.h>

#include <vector>
#include <cstring>

struct XrActionSuggestedBinding {
    XrAction action;
    XrPath binding;
};


void Moss_XR_AutoBindActions(MossXR_ActionSet* set, const char* interactionProfile) {
    std::vector<XrActionSuggestedBinding> bindings;

    auto bind = [&](const char* actionName, const char* pathStr) {
        XrPath path;
        xrStringToPath(g_XR.context->instance, pathStr, &path);
        bindings.push_back({set->actions[actionName]->xrAction, path});
    };

    bind("select_left",  "/user/hand/left/input/select/click");
    bind("select_right", "/user/hand/right/input/select/click");

    XrPath profilePath;
    xrStringToPath(g_XR.context->instance, interactionProfile, &profilePath);

    XrInteractionProfileSuggestedBinding suggested{ XR_TYPE_INTERACTION_PROFILE_SUGGESTED_BINDING };
    suggested.interactionProfile = profilePath;
    suggested.countSuggestedBindings = (uint32_t)bindings.size();
    suggested.suggestedBindings = bindings.data();

    XR_CHECK(xrSuggestInteractionProfileBindings(g_XR.context->instance, &suggested));
}

// =======================================================
// Actions
// =======================================================
bool Moss_XR_GetActionBoolean(MossXR_Action* action, XrPath subaction) {
    XrActionStateGetInfo info{XR_TYPE_ACTION_STATE_GET_INFO};
    info.action = action->xrAction;
    info.subactionPath = subaction;

    XrActionStateBoolean state{XR_TYPE_ACTION_STATE_BOOLEAN};
    xrGetActionStateBoolean(g_XR.session->session, &info, &state);

    return state.isActive && state.currentState;
}

float Moss_XR_GetActionFloat(MossXR_Action* action) {
    if (!action)
        return 0.0f;

    XrActionStateGetInfo getInfo{ XR_TYPE_ACTION_STATE_GET_INFO };
    getInfo.action = action->xrAction;

    XrActionStateFloat state{ XR_TYPE_ACTION_STATE_FLOAT };

    if (xrGetActionStateFloat(g_XR->session, &getInfo, &state) != XR_SUCCESS)
        return 0.0f;

    return state.isActive ? state.currentState : 0.0f;
}


bool Moss_XR_GetActionPose(
    MossXR_Action* action,
    MossXR_Pose* outPose)
{
    if (!action || !outPose)
        return false;

    XrActionStateGetInfo getInfo{ XR_TYPE_ACTION_STATE_GET_INFO };
    getInfo.action = action->xrAction;

    XrActionStatePose state{ XR_TYPE_ACTION_STATE_POSE };

    if (xrGetActionStatePose(
            g_XR->session,
            &getInfo,
            &state) != XR_SUCCESS)
        return false;

    if (!state.isActive || action->space == XR_NULL_HANDLE)
        return false;

    XrSpaceLocation loc{ XR_TYPE_SPACE_LOCATION };

    if (xrLocateSpace(
            action->space,
            g_XR->appSpace,
            g_XR->frameState.predictedDisplayTime,
            &loc) != XR_SUCCESS)
        return false;

    if (!(loc.locationFlags & XR_SPACE_LOCATION_POSITION_VALID_BIT))
        return false;

    outPose->position[0] = loc.pose.position.x;
    outPose->position[1] = loc.pose.position.y;
    outPose->position[2] = loc.pose.position.z;

    outPose->orientation[0] = loc.pose.orientation.x;
    outPose->orientation[1] = loc.pose.orientation.y;
    outPose->orientation[2] = loc.pose.orientation.z;
    outPose->orientation[3] = loc.pose.orientation.w;

    return true;
}

MossXR_Action* Moss_XR_CreateAction(MossXR_ActionSet* set, const char* name, MossXR_ActionType type) {
    MossXR_Action* a = new MossXR_Action();
    a->type = type;

    XrPath subactionPaths[2];
    uint32_t subactionCount = 0;

    if (perHand) {
        xrStringToPath(g_XR.context->instance,
            "/user/hand/left", &subactionPaths[0]);
        xrStringToPath(g_XR.context->instance,
            "/user/hand/right", &subactionPaths[1]);
        subactionCount = 2;
    }

    XrActionCreateInfo ci{XR_TYPE_ACTION_CREATE_INFO};
    strcpy(ci.actionName, name);
    strcpy(ci.localizedActionName, name);
    ci.actionType = ToXrActionType(type);
    ci.countSubactionPaths = subactionCount;
    ci.subactionPaths = subactionPaths;

    XR_CHECK(xrCreateAction(set->xrSet, &ci, &a->xrAction));

    set->actions[name] = a;
    return a;
}


MossXR_ActionSet* Moss_XR_CreateActionSet(const char* name)
{
    MossXR_ActionSet* set = new MossXR_ActionSet();

    XrActionSetCreateInfo info{XR_TYPE_ACTION_SET_CREATE_INFO};
    strcpy(info.actionSetName, name);
    strcpy(info.localizedActionSetName, name);
    info.priority = 0;

    XR_CHECK(xrCreateActionSet(g_XR.context->instance, &info, &set->xrSet));

    g_XR.actionSets.push_back(set);
    return set;
}

void Moss_XR_DestroyActionSet(MossXR_ActionSet* set)
{
    if (!set) return;
    xrDestroyActionSet(set->xrSet);
    free(set);
}

void Moss_XR_DestroyAction(MossXR_Action* action)
{
    if (!action) return;
    xrDestroyAction(action->xrAction);
    free(action);
}

void Moss_XR_AttachActionSet(MossXR_Session* session, MossXR_ActionSet* set)
{
    XrSessionActionSetsAttachInfo attachInfo{ XR_TYPE_SESSION_ACTION_SETS_ATTACH_INFO };
    attachInfo.countActionSets = 1;
    attachInfo.actionSets = &set->xrSet;
    xrAttachSessionActionSets(session->xrSession, &attachInfo);
}

void Moss_XR_SyncActions(MossXR_Session* session)
{
    XrActiveActionSet active{};
    active.actionSet = session->actionSet->xrSet;

    XrActionsSyncInfo syncInfo{ XR_TYPE_ACTIONS_SYNC_INFO };
    syncInfo.countActiveActionSets = 1;
    syncInfo.activeActionSets = &active;

    xrSyncActions(session->xrSession, &syncInfo);
}

MossXR_Anchor* Moss_XR_CreateAnchor(const MossXR_Pose* pose) {
    MossXR_Anchor* anchor = (MossXR_Anchor*)calloc(1, sizeof(MossXR_Anchor));

    XrSpatialAnchorCreateInfoMSFT info{ XR_TYPE_SPATIAL_ANCHOR_CREATE_INFO_MSFT };
    info.pose = pose->ToXrPose();
    info.space = g_XR->appSpace;
    info.time = g_XR->frameState.predictedDisplayTime;

    xrCreateSpatialAnchorMSFT(g_XR->session, &info, &anchor->xrAnchor);
    xrCreateSpatialAnchorSpaceMSFT(g_XR->session, &anchor->spaceInfo, &anchor->space);

    return anchor;
}

void Moss_XR_DestroyAnchor(MossXR_Anchor* anchor) {
    if (!anchor) return;
    xrDestroySpatialAnchorMSFT(anchor->xrAnchor);
    xrDestroySpace(anchor->space);
    free(anchor);
}

bool Moss_XR_LocateAnchor(MossXR_Anchor* anchor, MossXR_Time time, MossXR_Pose* outPose) {
    XrSpaceLocation loc{ XR_TYPE_SPACE_LOCATION };
    xrLocateSpace(anchor->space, g_XR->appSpace, time, &loc);

    if (!(loc.locationFlags & XR_SPACE_LOCATION_POSITION_VALID_BIT))
        return false;

    outPose->FromXrPose(loc.pose);
    return true;
}


MossXR_Layer* Moss_XR_CreateProjectionLayer(MossXR_Swapchain* swapchain) {
    MossXR_Layer* layer = new MossXR_Layer();
    layer->type = MOSS_XR_LAYER_PROJECTION;
    layer->swapchain = swapchain;
    return layer;
}

MossXR_Layer* Moss_XR_CreateQuadLayer(MossXR_Swapchain* swapchain, const MossXR_Pose* pose, Vec2 size) {
    MossXR_Layer* layer = new MossXR_Layer();
    layer->type = MOSS_XR_LAYER_QUAD;
    layer->pose = *pose;
    layer->size = size;
    return layer;
}

void Moss_XR_DestroyLayer(MossXR_Layer* layer) { delete layer; }

void Moss_XR_SubmitLayers(MossXR_Session* session) {
    std::sort(g_XR.frameGraph.begin(), g_XR.frameGraph.end(),
        [](auto& a, auto& b){ return a.priority < b.priority; });
    
    std::vector<XrCompositionLayerBaseHeader*> baseLayers;
    for (auto layer : g_XR.layers)
        baseLayers.push_back((XrCompositionLayerBaseHeader*)layer->xrLayer);

    XrFrameEndInfo endInfo{XR_TYPE_FRAME_END_INFO};
    endInfo.displayTime = g_XR.frameState.xrFrameState.predictedDisplayTime;
    endInfo.environmentBlendMode = XR_ENVIRONMENT_BLEND_MODE_OPAQUE;
    endInfo.layerCount = (uint32_t)baseLayers.size();
    endInfo.layers = baseLayers.data();

    xrEndFrame(session->xrSession, &endInfo);
}

void Moss_XR_PlayHaptic(MossXR_Action* action, XrPath hand, float amplitude, float durationSeconds) {
    if (!action)
        return;

    XrHapticVibration vib{XR_TYPE_HAPTIC_VIBRATION};
    vib.amplitude = amplitude;
    vib.duration = (XrDuration)(duration * 1e9);

    XrHapticActionInfo info{XR_TYPE_HAPTIC_ACTION_INFO};
    info.action = action->xrAction;
    info.subactionPath = hand;

    xrApplyHapticFeedback(g_XR.session->session, &info, (XrHapticBaseHeader*)&vib);
}

void Moss_XR_StopHaptic(MossXR_Action* action) {
    if (!action)
        return;

    XrHapticActionInfo info{ XR_TYPE_HAPTIC_ACTION_INFO };
    info.action = action->xrAction;
    info.subactionPath = XR_NULL_PATH;

    xrStopHapticFeedback(g_XR->session, &info);
}