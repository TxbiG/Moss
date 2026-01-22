#include <Moss/Moss_Platform.h>
#include <Moss/Moss_stdinc.h>

Moss_CameraID* Moss_GetCameras(int* count);
const char* Moss_GetCameraName(Moss_CameraID camera_id);
Moss_CameraPosition Moss_GetCameraPosition(Moss_CameraID camera_id);
const char* Moss_GetCurrentCameraDriver(void);
int Moss_GetNumCameraDrivers(void);
const Moss_CameraSpec* Moss_GetCameraSupportedFormats(Moss_CameraID camera_id, int* count);
Moss_Surface* Moss_AcquireCameraFrame(Moss_Camera* camera, uint64_t* timestamp_ns);
void Moss_ReleaseCameraFrame(Moss_Camera* camera, Moss_Surface* frame);
bool Moss_GetCameraFormat(Moss_Camera* camera, Moss_CameraSpec* out_spec);
Moss_CameraPermissionState Moss_GetCameraPermissionState(Moss_Camera* camera);
Moss_PropertiesID Moss_GetCameraProperties(Moss_Camera* camera);
void Moss_CloseCamera(Moss_Camera *camera);
Moss_CameraID Moss_GetCameraID(Moss_Camera *camera);
Moss_PropertiesID Moss_GetCameraProperties(Moss_Camera *camera);
Moss_Camera * Moss_OpenCamera(Moss_CameraID instance_id, const Moss_CameraSpec *spec);
Moss_VideoCapture* Moss_OpenVideoCapture(Moss_VideoCaptureID captureID);

void Moss_CloseVideoCapture(Moss_VideoCapture* cap);

uint8_t* Moss_VideoCaptureReadFrame(Moss_VideoCapture* cap);