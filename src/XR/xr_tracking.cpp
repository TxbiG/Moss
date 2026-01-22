// Moss_XR.cpp
#include <Moss/Moss_XR.h>
#include <Moss/XR/xr_intern.h>

#include <openxr/openxr.h>
#include <openxr/openxr_platform.h>

#include <cstdlib>
#include <cstring>



MossXR_BodyModifier* Moss_XR_CreateBody() {
#if defined(XR_HTC_body_tracking) || defined(XR_EXT_body_tracking)
    MossXR_BodyModifier* body = (MossXR_BodyModifier*)malloc(sizeof(MossXR_BodyModifier));
    memset(body, 0, sizeof(MossXR_BodyModifier));

#if defined(XR_HTC_body_tracking)
    if (g_XR.capabilities.bodyTracking)
    {
        body->supported = true;
        body->backend   = MOSS_XR_BODY_HTC;

        XrBodyTrackerCreateInfoHTC ci{XR_TYPE_BODY_TRACKER_CREATE_INFO_HTC};
        ci.bodyJointSet = XR_BODY_JOINT_SET_DEFAULT_HTC;

        if (XR_FAILED(
                xrCreateBodyTrackerHTC(
                    g_XR.session->session,
                    &ci,
                    &body->htcTracker)))
        {
            free(body);
            return nullptr;
        }
    }
#endif

#if defined(XR_FB_body_tracking)
    if (g_XR.capabilities.bodyTracking)
    {
        body->supported = true;
        body->backend   = MOSS_XR_BODY_META;

        XrBodyTrackerCreateInfoFB ci{XR_TYPE_BODY_TRACKER_CREATE_INFO_FB};
        ci.bodyJointSet = XR_BODY_JOINT_SET_DEFAULT_FB;

        if (XR_FAILED(
                xrCreateBodyTrackerFB(
                    g_XR.session->session,
                    &ci,
                    &body->metaTracker)))
        {
            free(body);
            return nullptr;
        }
    }
#endif

    return body;
}


MossXR_FaceModifier* Moss_XR_CreateFace(void) {
    return (MossXR_FaceModifier*)calloc(1, sizeof(MossXR_FaceModifier));
}


/*! @brief X @return X*/
MossXR_HandModifier* Moss_XR_CreateHand(MossXR_Handedness handedness) {
    if (!g_XR.session || !g_XR.capabilities.handTracking)
        return nullptr;

    MossXR_HandModifier* hand = (MossXR_HandModifier*)calloc(1, sizeof(MossXR_HandModifier));

    hand->supported = true;
    hand->hand = (handedness == MOSS_XR_HAND_LEFT) ? XR_HAND_LEFT_EXT : XR_HAND_RIGHT_EXT;

    XrHandTrackerCreateInfoEXT ci{XR_TYPE_HAND_TRACKER_CREATE_INFO_EXT};
    ci.hand = hand->hand;
    ci.handJointSet = XR_HAND_JOINT_SET_DEFAULT_EXT;

    if (XR_FAILED(xrCreateHandTrackerEXT(g_XR.session->session, &ci, &hand->tracker))) {
        free(hand);
        return nullptr;
    }

    return hand;
}

// Terminates

void Moss_XR_DestroyHand(MossXR_HandModifier* hand) { if (!hand) return; if (hand->tracker != XR_NULL_HANDLE) xrDestroyHandTrackerEXT(hand->tracker); free(hand); }

void Moss_XR_DestroyBody(MossXR_BodyModifier* body) { 
    if (!body) return; 
#if defined(XR_HTC_body_tracking)
    if (body->backend == MOSS_XR_BODY_HTC && body->htcTracker != XR_NULL_HANDLE)
        xrDestroyBodyTrackerHTC(body->htcTracker);
#endif
#if defined(XR_FB_body_tracking)
    if (body->backend == MOSS_XR_BODY_META && body->metaTracker != XR_NULL_HANDLE)
        xrDestroyBodyTrackerFB(body->metaTracker);
#endif
    free(body); }

void Moss_XR_DestroyFace(MossXR_FaceModifier* face) { if (!face) free(face); }




/*! @brief X @param X */
bool Moss_XR_GetHandJointPose(MossXR_HandModifier* hand, XRHandJoint joint, MossXR_Pose* outPose) {
    if (!hand || !outPose)
        return false;

    const XrHandJointLocationEXT& loc = hand->joints[joint];

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

/*! @brief X @param X */
bool Moss_XR_GetHandPose(MossXR_HandModifier* hand, MossXR_Pose* outPose) {
    if (!hand || !hand->supported || !outPose)
        return false;

    XrHandJointsLocateInfoEXT locateInfo{ XR_TYPE_HAND_JOINTS_LOCATE_INFO_EXT };
    locateInfo.baseSpace = g_XR.session->appSpace;
    locateInfo.time = g_XR.session->frameState.predictedDisplayTime;

    XrHandJointLocationsEXT locations{ XR_TYPE_HAND_JOINT_LOCATIONS_EXT };
    locations.jointCount = XR_HAND_JOINT_COUNT_EXT;
    locations.jointLocations = hand->joints;

    if (XR_FAILED(xrLocateHandJointsEXT(hand->tracker, &locate, &joints)))
        return false;

    if (!locations.isActive)
        return false;

    // Use palm as hand pose
    const XrHandJointLocationEXT& palm = hand->jointLocations[XR_HAND_JOINT_PALM_EXT];

    if (!(palm.locationFlags & XR_SPACE_LOCATION_POSITION_VALID_BIT))
        return false;

    outPose->position[0] = palm.pose.position.x;
    outPose->position[1] = palm.pose.position.y;
    outPose->position[2] = palm.pose.position.z;

    outPose->orientation[0] = palm.pose.orientation.x;
    outPose->orientation[1] = palm.pose.orientation.y;
    outPose->orientation[2] = palm.pose.orientation.z;
    outPose->orientation[3] = palm.pose.orientation.w;

    return true;
}


bool Moss_XR_GetBodyJointPose(MossXR_BodyModifier* body, XRBodyJoint joint, MossXR_Pose* outPose) {
    if (!body || !body->supported || !outPose)
        return false;

#if defined(XR_HTC_body_tracking)
    if (body->backend == MOSS_XR_BODY_HTC)
    {
        const XrBodyJointLocationHTC& loc = body->htcJoints[joint];

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
#endif

#if defined(XR_FB_body_tracking)
    if (body->backend == MOSS_XR_BODY_META)
    {
        const XrBodyJointLocationFB& loc = body->metaJoints[joint];

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
#endif

    return false;
}


// Updates
void Moss_XR_UpdateBodyTracking(MossXR_Session* session) {
#if defined(MOSS_XR_HTC_BODY_TRACKING) || defined(MOSS_XR_META_BODY_TRACKING)
    if (!g_XR.body) return;

    XrBodyJointsLocateInfoHTC locate{XR_TYPE_BODY_JOINTS_LOCATE_INFO_HTC};
    locate.baseSpace = appSpace;
    locate.time = predictedTime;

    XrBodyJointLocationsHTC joints{XR_TYPE_BODY_JOINT_LOCATIONS_HTC};
    joints.jointCount = XR_BODY_JOINT_COUNT_HTC;
    joints.jointLocations = body->jointLocations;

    xrLocateBodyJointsHTC(body->tracker, &locate, &joints);

    // Query each body joint from body tracker (HTC / Meta specific)
    for (uint32_t i = 0; i < g_XR.body->jointCount; i++) {
        XrPosef jointPose;
        if (XR_SUCCEEDED(xrLocateBodyJointHTC(g_XR.body->bodyTracker, (XRBodyJoint)i,
                                             g_XR.context->app_space,
                                             Moss_XR_GetPredictedDisplayTime(g_XR.session),
                                             &jointPose))) {
            g_XR.body->jointPoses[i] = jointPose;
        }
    }
#endif
}

void Moss_XR_UpdateHandTracking(MossXR_Session* session) {
    if (!g_XR.leftHand || !g_XR.rightHand) return;

    XrHandTrackerEXT handTrackLeft = g_XR.leftHand->handTracker;
    XrHandTrackerEXT handTrackRight = g_XR.rightHand->handTracker;

    XrHandJointLocationEXT jointLocations[XR_HAND_JOINT_COUNT_EXT];

    // Left hand
    XrHandJointsLocateInfoEXT locateInfo{XR_TYPE_HAND_JOINTS_LOCATE_INFO_EXT};
    locateInfo.baseSpace = g_XR.context->app_space;
    locateInfo.time = Moss_XR_GetPredictedDisplayTime(g_XR.session);
    xrLocateHandJointsEXT(handTrackLeft, &locateInfo, &g_XR.leftHand->jointCount, jointLocations);
    memcpy(g_XR.leftHand->jointPoses, jointLocations, sizeof(jointLocations));

    // Right hand
    locateInfo.hand = XR_HAND_RIGHT_EXT;
    xrLocateHandJointsEXT(handTrackRight, &locateInfo, &g_XR.rightHand->jointCount, jointLocations);
    memcpy(g_XR.rightHand->jointPoses, jointLocations, sizeof(jointLocations));
}