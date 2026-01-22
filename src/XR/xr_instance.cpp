// Moss_XR.cpp
#include <Moss/Moss_XR.h>
#include <Moss/XR/xr_intern.h>

#include <openxr/openxr.h>
#include <openxr/openxr_platform.h>

#include <vector>
#include <cstring>


// Globals
extern MossXR_Runtime g_XR;

static std::unordered_set<std::string> g_extensions;
static std::unordered_set<std::string> g_layers;
static MossXR_Capabilities g_caps = {};


bool HasExtension(const char* name)
{
    uint32_t count = 0;
    xrEnumerateInstanceExtensionProperties(nullptr, 0, &count, nullptr);

    std::vector<XrExtensionProperties> exts(count, {XR_TYPE_EXTENSION_PROPERTIES});
    xrEnumerateInstanceExtensionProperties(nullptr, count, &count, exts.data());

    for (auto& e : exts)
        if (strcmp(e.extensionName, name) == 0)
            return true;

    return false;
}

bool HasLayer(const char* name);


static void EnumerateExtensions() {
    uint32_t count = 0;
    xrEnumerateInstanceExtensionProperties(nullptr, 0, &count, nullptr);

    std::vector<XrExtensionProperties> props(count, {XR_TYPE_EXTENSION_PROPERTIES});
    xrEnumerateInstanceExtensionProperties(nullptr, count, &count, props.data());

    for (auto& p : props)
        g_extensions.insert(p.extensionName);
}

static void EnumerateLayers() {
    uint32_t count = 0;
    xrEnumerateApiLayerProperties(0, &count, nullptr);

    std::vector<XrApiLayerProperties> props(count, {XR_TYPE_API_LAYER_PROPERTIES});
    xrEnumerateApiLayerProperties(count, &count, props.data());

    for (auto& p : props)
        g_layers.insert(p.layerName);
}


// =======================================================
// Capabilities Probing
// =======================================================
const MossXR_Capabilities* Moss_XR_GetCapabilities() {
    g_caps.handTracking = g_extensions.count(XR_EXT_HAND_TRACKING_EXTENSION_NAME);
    g_caps.bodyTracking = g_extensions.count(XR_HTC_BODY_TRACKING_EXTENSION_NAME) || g_extensions.count(XR_FB_BODY_TRACKING_EXTENSION_NAME);

    g_caps.eyeTracking =
        g_extensions.count(XR_EXT_EYE_GAZE_INTERACTION_EXTENSION_NAME);

    g_caps.passthrough =
        g_extensions.count(XR_FB_PASSTHROUGH_EXTENSION_NAME);

    g_caps.foveatedRendering =
        g_extensions.count(XR_FB_FOVEATION_EXTENSION_NAME);

    g_caps.spatialAnchors =
        g_extensions.count(XR_MSFT_SPATIAL_ANCHOR_EXTENSION_NAME);

    return &g_caps;
}

// =======================================================
// OpenXR Validation Layer Support
// =======================================================
bool Moss_XR_EnableValidationLayers() {
#ifdef XR_EXT_debug_utils
    XrDebugUtilsMessengerCreateInfoEXT debugCreateInfo{ XR_TYPE_DEBUG_UTILS_MESSENGER_CREATE_INFO_EXT };
    debugCreateInfo.messageSeverities =
        XR_DEBUG_UTILS_MESSAGE_SEVERITY_VERBOSE_BIT_EXT |
        XR_DEBUG_UTILS_MESSAGE_SEVERITY_INFO_BIT_EXT |
        XR_DEBUG_UTILS_MESSAGE_SEVERITY_WARNING_BIT_EXT |
        XR_DEBUG_UTILS_MESSAGE_SEVERITY_ERROR_BIT_EXT;

    debugCreateInfo.messageTypes =
        XR_DEBUG_UTILS_MESSAGE_TYPE_GENERAL_BIT_EXT |
        XR_DEBUG_UTILS_MESSAGE_TYPE_VALIDATION_BIT_EXT |
        XR_DEBUG_UTILS_MESSAGE_TYPE_PERFORMANCE_BIT_EXT;

    debugCreateInfo.userCallback = [](XrDebugUtilsMessageSeverityFlagsEXT severity,
                                     XrDebugUtilsMessageTypeFlagsEXT types,
                                     const XrDebugUtilsMessengerCallbackDataEXT* data,
                                     void* userData) -> XrBool32
    {
        printf("[OpenXR][%d] %s\n", severity, data->message);
        return XR_FALSE;
    };

    PFN_xrCreateDebugUtilsMessengerEXT pfnCreateDebugMessenger = nullptr;
    xrGetInstanceProcAddr(g_ctx->instance, "xrCreateDebugUtilsMessengerEXT",
                          (PFN_xrVoidFunction*)&pfnCreateDebugMessenger);

    if (pfnCreateDebugMessenger)
    {
        pfnCreateDebugMessenger(g_ctx->instance, &debugCreateInfo, &g_ctx->debugUtilsMessenger);
        return true;
    }
#endif
    return false;
}




bool Moss_XR_Initialize(const MossXR_InitInfo* info) {
    if (!info) return false;


    EnumerateExtensions();
    EnumerateLayers();

    g_XR.context = new MossXR_Context();

    // --- Instance creation ---
    std::vector<const char*> enabledLayers;
    if (g_layers.count("XR_APILAYER_LUNARG_core_validation"))
        enabledLayers.push_back("XR_APILAYER_LUNARG_core_validation");

    XrInstanceCreateInfo ici{XR_TYPE_INSTANCE_CREATE_INFO};
    strcpy(ici.applicationInfo.applicationName, "Moss Engine");
    ici.applicationInfo.apiVersion = XR_CURRENT_API_VERSION;

    ici.enabledApiLayerCount = enabledLayers.size();
    ici.enabledApiLayerNames = enabledLayers.data();

    xrCreateInstance(&ici, &g_XR.context->instance);
    if (res != XR_SUCCESS)
        return false;

    // --- System ---
    XrSystemGetInfo sgi{XR_TYPE_SYSTEM_GET_INFO};
    sgi.formFactor = XR_FORM_FACTOR_HEAD_MOUNTED_DISPLAY;

    res = xrGetSystem(g_XR.context->instance, &sgi, &g_XR.context->systemId);

    if (res != XR_SUCCESS)
        return false;

    return true;
}


void Moss_XR_Shutdown(void) {
    if (g_XR.session) {
        xrDestroySession(g_XR.session->session);
        delete g_XR.session;
        g_XR.session = nullptr;
    }

    if (g_XR.context) {
        xrDestroyInstance(g_XR.context->instance);
        delete g_XR.context;
        g_XR.context = nullptr;
    }
}



bool Moss_XR_EnablePassthrough(bool enable) {
#if defined(XR_HTC_passthrough)
    // runtime specific
    return true;
#else
    return false;
#endif
}

bool Moss_XR_EnableFoveatedRendering(bool enable) {
#if defined(XR_FB_foveation)
    // xrCreateFoveationProfileFB(...)
    return true;
#else
    return false;
#endif
}

void Moss_XR_BeginDebugLabel(const char* label) {
#ifdef XR_EXT_debug_utils
    XrDebugUtilsLabelEXT labelInfo{XR_TYPE_DEBUG_UTILS_LABEL_EXT};
    strncpy(labelInfo.labelName, label, XR_MAX_DEBUG_UTILS_NAME_SIZE);
    PFN_xrBeginDebugUtilsLabelEXT fn;
    xrGetInstanceProcAddr(g_XR.context->instance, "xrBeginDebugUtilsLabelEXT", (PFN_xrVoidFunction*)&fn);
    if (fn) fn(g_XR.session->xrSession, &labelInfo);
#endif
}

void Moss_XR_EndDebugLabel(void) {
#ifdef XR_EXT_debug_utils
    PFN_xrEndDebugUtilsLabelEXT fn;
    xrGetInstanceProcAddr(g_XR.context->instance, "xrEndDebugUtilsLabelEXT", (PFN_xrVoidFunction*)&fn);
    if (fn) fn(g_XR.session->xrSession);
#endif
}

const char* Moss_XR_GetBackendName(void)
{ return "OpenXR"; }

uint32_t Moss_XR_GetBackendVersion(void) { return (uint32_t)XR_CURRENT_API_VERSION; }