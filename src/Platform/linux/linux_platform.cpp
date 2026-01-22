


int Moss_GetAvailableCPUCores(void);
int Moss_GetCPUCacheLineSize(void);
int Moss_GetSystemRAM(void);

bool Moss_OpenURL(const char *url);
Moss_PlatformTheme Moss_GetPlatformTheme();
Moss_Locale* Moss_GetLocale();
Moss_PowerState Moss_GetPowerInfo(int *seconds, int *percent);
bool Moss_IsProcessRunningByName(const char* executable_path);
void* Moss_LoadDynamicLibrary(const char* lib_path);
void* Moss_GetLibrarySymbol(void* handle, const char* symbol_name);
void Moss_UnloadDynamicLibrary(void* handle);


bool Moss_CopyFile(const char* src_path, const char* dst_path, bool overwrite);
bool Moss_CreateDirectory(const char* path, bool recursive);
bool Moss_RemovePath(const char* path, bool recursive);
bool Moss_RenamePath(const char* old_path, const char* new_path, bool overwrite);


bool Moss_GetPathInfo(const char* path, Moss_PathInfo* out_info);
bool Moss_GetCurrentDirectory(char* out_path, int max_len);
bool Moss_GetBasePath(char* out_path, int max_len);

bool Moss_GetUserFolder(Moss_UserFolder folder, char* out_path, int max_len);
bool Moss_GetPrefPath(const char* org_name, const char* app_name, char* out_path, int max_len);
bool Moss_EnumerateDirectory( const char* path, bool recursive, Moss_DirectoryIterateFn callback, void* user_data);
bool Moss_GlobDirectory(const char* pattern, Moss_DirectoryIterateFn callback, void* user_data);


void Moss_ShowFileDialogWithProperties(Moss_DialogFileCallback callback, void *userdata, Moss_Window *window, const Moss_DialogFileFilter *filters, int nfilters, const char *default_location);
void Moss_ShowOpenFileDialog(Moss_DialogFileCallback callback, void *userdata, Moss_Window *window, const char *default_location, bool allow_many);
void Moss_ShowOpenFolderDialog(Moss_DialogFileCallback callback, void *userdata, Moss_Window *window, const Moss_DialogFileFilter *filters, int nfilters, const char *default_location, bool allow_many);
void Moss_ShowSaveFileDialog(Moss_FileDialogType type, Moss_DialogFileCallback callback, void *userdata, Moss_PropertiesID props);

bool Moss_CloseStorage(Moss_Storage *storage);
bool Moss_CopyStorageFile(Moss_Storage *storage, const char *oldpath, const char *newpath);
bool Moss_CreateStorageDirectory(Moss_Storage *storage, const char *path);
bool Moss_EnumerateStorageDirectory(Moss_Storage *storage, const char *path, Moss_EnumerateDirectoryCallback callback, void *userdata);
bool Moss_GetStorageFileSize(Moss_Storage *storage, const char *path, uint64 *length);
uint64_t Moss_GetStorageSpaceRemaining(Moss_Storage *storage);
bool Moss_GetStoragePathInfo(Moss_Storage *storage, const char *path, Moss_PathInfo *info);
char** Moss_GlobStorageDirectory(Moss_Storage *storage, const char *path, const char *pattern, Moss_GlobFlags flags, int *count);
Moss_Storage* Moss_OpenFileStorage(const char *path);
Moss_Storage* Moss_OpenStorage(const Moss_Storage *iface, void *userdata);
Moss_Storage* Moss_OpenTitleStorage(const char *override, Moss_PropertiesID props);
Moss_Storage* Moss_OpenUserStorage(const char *org, const char *app, Moss_PropertiesID props);
bool Moss_ReadStorageFile(Moss_Storage *storage, const char *path, void *destination, uint64 length);
bool Moss_RemoveStoragePath(Moss_Storage *storage, const char *path);
bool Moss_RenameStoragePath(Moss_Storage *storage, const char *oldpath, const char *newpath);
bool Moss_StorageReady(Moss_Storage *storage);
bool Moss_WriteStorageFile(Moss_Storage *storage, const char *path, const void *source, uint64 length);