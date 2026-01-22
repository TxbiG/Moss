// Moss_XR.cpp
#include <Moss/Moss_XR.h>
#include <Moss/XR/xr_intern.h>

#include <openxr/openxr.h>
#include <openxr/openxr_platform.h>

#include <vector>
#include <cstring>


// -------------------------------
// Swapchains
// -------------------------------
MossXR_Swapchain* Moss_XR_CreateSwapchain(MossXR_Session* session, uint32_t width, uint32_t height, ETextureFormat format) {
    MossXR_Swapchain* sc = (MossXR_Swapchain*)calloc(1, sizeof(MossXR_Swapchain));

    XrSwapchainCreateInfo ci{XR_TYPE_SWAPCHAIN_CREATE_INFO};
    ci.usageFlags = XR_SWAPCHAIN_USAGE_COLOR_ATTACHMENT_BIT | XR_SWAPCHAIN_USAGE_SAMPLED_BIT;
    ci.format = ToXrFormat(format); // your mapping
    ci.width = width;
    ci.height = height;
    ci.arraySize = 1;
    ci.mipCount = 1;
    ci.faceCount = 1;
    ci.sampleCount = 1;

    XR_CHECK(xrCreateSwapchain(session->session, &ci, &sc->xrSwapchain));

    uint32_t imageCount = 0;
    xrEnumerateSwapchainImages(sc->xrSwapchain, 0, &imageCount, nullptr);

    sc->images.resize(imageCount);
    for (auto& img : sc->images)
        img.type = XR_TYPE_SWAPCHAIN_IMAGE_VULKAN_KHR; // backend-specific

    xrEnumerateSwapchainImages(sc->xrSwapchain, imageCount, &imageCount,(XrSwapchainImageBaseHeader*)sc->images.data());

    return sc;
}

void Moss_XR_DestroySwapchain(MossXR_Swapchain* swapchain)
{
    if (!swapchain) return;

    if (swapchain->xrSwapchain != XR_NULL_HANDLE)
        xrDestroySwapchain(swapchain->xrSwapchain);

    free(swapchain);
}

void* Moss_XR_AcquireSwapchainImage(MossXR_Swapchain* swapchain, uint32_t* outImageIndex) {
    if (!swapchain) return nullptr;

    XrSwapchainImageAcquireInfo acquireInfo{ XR_TYPE_SWAPCHAIN_IMAGE_ACQUIRE_INFO };
    xrAcquireSwapchainImage(
        swapchain->xrSwapchain,
        &acquireInfo,
        outImageIndex
    );

    XrSwapchainImageWaitInfo waitInfo{ XR_TYPE_SWAPCHAIN_IMAGE_WAIT_INFO };
    waitInfo.timeout = XR_INFINITE_DURATION;
    xrWaitSwapchainImage(swapchain->xrSwapchain, &waitInfo);

    return swapchain->images[*outImageIndex];
}

void Moss_XR_ReleaseSwapchainImage(MossXR_Swapchain* swapchain, uint32_t /*imageIndex*/) {
    if (!swapchain) return;

    XrSwapchainImageReleaseInfo releaseInfo{ XR_TYPE_SWAPCHAIN_IMAGE_RELEASE_INFO };
    xrReleaseSwapchainImage(swapchain->xrSwapchain, &releaseInfo);
}

MossXR_Anchor* Moss_XR_CreateAnchor(const MossXR_Pose* pose) {
    if (!g_XR.capabilities.spatialAnchors)
        return nullptr;

    MossXR_Anchor* a = (MossXR_Anchor*)calloc(1, sizeof(MossXR_Anchor));

    XrSpatialAnchorCreateInfoMSFT ci{
        XR_TYPE_SPATIAL_ANCHOR_CREATE_INFO_MSFT};
    ci.space = g_XR.session->appSpace;
    ci.pose = pose->ToXrPose();
    ci.time = g_XR.session->frameState.predictedDisplayTime;

    xrCreateSpatialAnchorMSFT(g_XR.session->session, &ci, &a->anchor);

    XrSpatialAnchorSpaceCreateInfoMSFT sci{
        XR_TYPE_SPATIAL_ANCHOR_SPACE_CREATE_INFO_MSFT};
    sci.anchor = a->anchor;
    sci.poseInAnchorSpace.orientation.w = 1.0f;

    xrCreateSpatialAnchorSpaceMSFT(g_XR.session->session, &sci, &a->space);

    return a;
}

void Moss_XR_DestroyAnchor(MossXR_Anchor* anchor) {
    if (!anchor) return;
    xrDestroySpatialAnchorMSFT(anchor->xrAnchor);
    xrDestroySpace(anchor->space);
    free(anchor);
}

bool Moss_XR_LocateAnchor(MossXR_Anchor* anchor, MossXR_Time time, MossXR_Pose* outPose) {
    XrSpaceLocation loc{ XR_TYPE_SPACE_LOCATION };
    xrLocateSpace(anchor->space, g_ctx->appSpace, time, &loc);

    if (!(loc.locationFlags & XR_SPACE_LOCATION_POSITION_VALID_BIT))
        return false;

    outPose->FromXrPose(loc.pose);
    return true;
}


MossXR_Layer* Moss_XR_CreateProjectionLayer(MossXR_Swapchain* sc) {
    MossXR_Layer* layer = new MossXR_Layer();
    layer->type = MOSS_XR_LAYER_PROJECTION;
    layer->swapchain = sc;

    layer->projection = {XR_TYPE_COMPOSITION_LAYER_PROJECTION};
    layer->projection.space = g_XR.session->appSpace;

    layer->xrLayer =
        (XrCompositionLayerBaseHeader*)&layer->projection;

    g_XR.layers.push_back(layer);
    return layer;
}

MossXR_Layer* Moss_XR_CreateQuadLayer(MossXR_Swapchain* sc, const MossXR_Pose* pose, Vec2 size) {
    MossXR_Layer* layer = new MossXR_Layer();
    layer->type = MOSS_XR_LAYER_QUAD;
    layer->swapchain = sc;

    layer->quad = {XR_TYPE_COMPOSITION_LAYER_QUAD};
    layer->quad.space = g_XR.session->appSpace;
    layer->quad.pose = pose->ToXrPose();
    layer->quad.size = { size.x, size.y };

    layer->xrLayer =
        (XrCompositionLayerBaseHeader*)&layer->quad;

    g_XR.layers.push_back(layer);
    return layer;
}

void Moss_XR_DestroyLayer(MossXR_Layer* layer) { delete layer; }

void Moss_XR_SubmitLayers(MossXR_Session* session) {
    std::sort(g_XR.frameGraph.begin(), g_XR.frameGraph.end(), [](auto& a, auto& b){ return a.priority < b.priority; });
    
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
