#include <Moss/Platform/Windows/win32_platform.h>

#define WIN32_LEAN_AND_MEAN
#define NOMINMAX
#include <windows.h>
#include <shlobj.h>
#include <shlwapi.h>
#include <psapi.h>
#include <stdint.h>
#include <stdbool.h>

#pragma comment(lib, "Shlwapi.lib")


// Helpers:

// Storange helper
static void moss_storage_fullpath(Moss_Storage *storage, const char *path, char *out) {
    if (path && path[0])
        wsprintfA(out, "%s\\%s", storage->root, path);
    else
        lstrcpyA(out, storage->root);
}






int Moss_GetAvailableCPUCores(void) {
    SYSTEM_INFO sysInfo;
    GetSystemInfo(&sysInfo);
    return (int)sysInfo.dwNumberOfProcessors;
}

int Moss_GetCPUCacheLineSize(void) {
    DWORD buffer_size = 0;
    SYSTEM_LOGICAL_PROCESSOR_INFORMATION *buffer = NULL;
    GetLogicalProcessorInformation(NULL, &buffer_size);
    buffer = (SYSTEM_LOGICAL_PROCESSOR_INFORMATION*)malloc(buffer_size);
    if (!buffer) return 64;

    if (GetLogicalProcessorInformation(buffer, &buffer_size))
    {
        for (DWORD i = 0; i < buffer_size / sizeof(SYSTEM_LOGICAL_PROCESSOR_INFORMATION); i++)
        {
            if (buffer[i].Relationship == RelationCache &&
                buffer[i].Cache.Level == 1)
            {
                int line_size = buffer[i].Cache.LineSize;
                free(buffer);
                return line_size;
            }
        }
    }
    free(buffer);
    return 64;
}

int Moss_GetSystemRAM(void) {
    MEMORYSTATUSEX statex;
    statex.dwLength = sizeof(statex);
    if (GlobalMemoryStatusEx(&statex))
        return (int)(statex.ullTotalPhys / (1024 * 1024));
    return -1;
}

bool Moss_OpenURL(const char* url) {
    if (!url) {  MOSS_ERROR("URL is null.");  return false; }

    const wchar_t* wurl = convertCharToWchar(url);

    // Initialize COM as per MSDN recommendation
    HRESULT hr = CoInitializeEx(nullptr, COINIT_APARTMENTTHREADED);
    bool comInitialized = SUCCEEDED(hr);

    HINSTANCE rc = ShellExecuteW(nullptr, L"open", wurl, nullptr, nullptr, SW_SHOWNORMAL);

    if (comInitialized)
        CoUninitialize();

    if ((INT_PTR)rc <= 32) {
        MOSS_ERROR("Couldn't open given URL.");
        return false;
    }

    return true;
}

Moss_Locale* Moss_GetLocale() {
    WCHAR wbuf[LOCALE_NAME_MAX_LENGTH];
    if (GetUserDefaultLocaleName(wbuf, LOCALE_NAME_MAX_LENGTH) == 0) {
        locale->language = strdup_safe("en");
        locale->country  = strdup_safe("US");
        return locale;
    }

    // Convert wide char to UTF-8
    int size_needed = WideCharToMultiByte(CP_UTF8, 0, wbuf, -1, NULL, 0, NULL, NULL);
    char* buf = (char*)malloc(size_needed);
    if (!buf) {
        locale->language = strdup_safe("en");
        locale->country  = strdup_safe("US");
        return locale;
    }
    WideCharToMultiByte(CP_UTF8, 0, wbuf, -1, buf, size_needed, NULL, NULL);

    // Split into lang + country
    char* dash = strchr(buf, '-');
    if (dash) {
        *dash = '\0';
        locale->language = strdup_safe(buf);
        locale->country  = strdup_safe(dash + 1);
    } else {
        locale->language = strdup_safe(buf);
        locale->country  = strdup_safe("US");
    }
    free(buf);
}


/* ---------------------------------------------------------- */
/* Power / Process                                            */
/* ---------------------------------------------------------- */

Moss_PowerState Moss_GetPowerInfo(int *seconds, int *percent) {
    SYSTEM_POWER_STATUS sps;
    if (!GetSystemPowerStatus(&sps))
        return MOSS_POWERSTATE_UNKNOWN;

    if (percent)
        *percent = (sps.BatteryLifePercent == 255) ? -1 : sps.BatteryLifePercent;

    if (seconds)
        *seconds = (sps.BatteryLifeTime == (DWORD)-1) ? -1 : (int)sps.BatteryLifeTime;

    if (sps.ACLineStatus == 1)
        return MOSS_POWERSTATE_CHARGING;

    return (sps.BatteryLifePercent < 10)
        ? MOSS_POWERSTATE_LOW
        : MOSS_POWERSTATE_ON_BATTERY;
}

bool Moss_IsProcessRunningByName(const char* executable_path) {
    bool found = false;
    HANDLE snap = CreateToolhelp32Snapshot(TH32CS_SNAPPROCESS, 0);
    if (snap == INVALID_HANDLE_VALUE)
        return false;

    PROCESSENTRY32 pe = { sizeof(pe) };
    if (Process32First(snap, &pe)) {
        do {
            if (_stricmp(pe.szExeFile, executable_path) == 0) {
                found = true;
                break;
            }
        } while (Process32Next(snap, &pe));
    }

    CloseHandle(snap);
    return found;
}

void* Moss_LoadDynamicLibrary(const char* dll) { HMODULE result = LoadLibraryA(dll); return result; }
void* Moss_GetLibrarySymbol(void* handle, const char* symbol_name) { return (void*)GetProcAddress((HMODULE)handle, symbol_name); }
void Moss_UnloadDynamicLibrary(void* handle) { if (handle) FreeLibrary((HMODULE)handle); }


/* ---------------------------------------------------------- */
/* Files / Directories                                        */
/* ---------------------------------------------------------- */
bool Moss_CopyFile(const char* src_path, const char* dst_path, bool overwrite) { return CopyFileA(src_path, dst_path, !overwrite); }

bool Moss_CreateDirectory(const char* path, bool recursive) {
    if (!recursive) { return CreateDirectoryA(path, NULL) || GetLastError() == ERROR_ALREADY_EXISTS; }

    char tmp[MAX_PATH];
    lstrcpyA(tmp, path);

    for (char* p = tmp + 1; *p; p++) {
        if (*p == '\\' || *p == '/') {
            char old = *p;
            *p = 0;
            CreateDirectoryA(tmp, NULL);
            *p = old;
        }
    }
    return CreateDirectoryA(tmp, NULL) || GetLastError() == ERROR_ALREADY_EXISTS;
}

bool Moss_RemovePath(const char* path, bool recursive) {
    DWORD attrs = GetFileAttributesA(path);
    if (attrs == INVALID_FILE_ATTRIBUTES) { return false; }
    if (attrs & FILE_ATTRIBUTE_DIRECTORY) { return RemoveDirectoryA(path); }
    else { return DeleteFileA(path); }
}

bool Moss_RenamePath(const char* old_path, const char* new_path, bool overwrite) {
    DWORD flags = MOVEFILE_COPY_ALLOWED;
    if (overwrite) { flags |= MOVEFILE_REPLACE_EXISTING; }
    return MoveFileExA(old_path, new_path, flags);
}


/* ---------------------------------------------------------- */
/* Path info                                                  */
/* ---------------------------------------------------------- */

bool Moss_GetPathInfo(const char* path, Moss_PathInfo* out_info) {
    WIN32_FILE_ATTRIBUTE_DATA data;
    if (!GetFileAttributesExA(path, GetFileExInfoStandard, &data)) { return false; }

    out_info->is_directory = (data.dwFileAttributes & FILE_ATTRIBUTE_DIRECTORY) != 0;

    LARGE_INTEGER size;
    size.HighPart = data.nFileSizeHigh;
    size.LowPart  = data.nFileSizeLow;
    out_info->size = (uint64_t)size.QuadPart;

    return true;
}

bool Moss_GetCurrentDirectory(char* out_path, int max_len) { return GetCurrentDirectoryA(max_len, out_path) > 0; }

bool Moss_GetBasePath(char* out_path, int max_len) { return GetModuleFileNameA(NULL, out_path, max_len) > 0; }


/* ---------------------------------------------------------- */
/* User folders                                               */
/* ---------------------------------------------------------- */

bool Moss_GetUserFolder(Moss_UserFolder folder, char* out_path, int max_len) {
    if (!out_path || max_len <= 0)
        return false;

    REFKNOWNFOLDERID kfid;

    switch (folder) {
        case Moss_UserFolder::DESKTOP:    kfid = FOLDERID_Desktop; break;
        case Moss_UserFolder::HOME:       kfid = FOLDERID_Profile; break;
        case Moss_UserFolder::DOCUMENTS:  kfid = FOLDERID_Documents; break;
        case Moss_UserFolder::APPDATA:    kfid = FOLDERID_RoamingAppData; break;
        case Moss_UserFolder::PICTURES:   kfid = FOLDERID_Pictures; break;
        case Moss_UserFolder::MUSIC:      kfid = FOLDERID_Music; break;
        case Moss_UserFolder::VIDEOS:     kfid = FOLDERID_Videos; break;
        case Moss_UserFolder::CACHE:      kfid = FOLDERID_InternetCache; break;
        case Moss_UserFolder::DOWNLOADS:  kfid = FOLDERID_Downloads; break;
        default:
            return false;
    }

    PWSTR wpath = NULL;
    if (FAILED(SHGetKnownFolderPath(kfid, 0, NULL, &wpath)))
        return false;

    int written = WideCharToMultiByte(CP_UTF8, 0, wpath, -1, out_path, max_len, NULL, NULL);

    CoTaskMemFree(wpath);

    if (written == 0 || written >= max_len)
        return false;

    return true;
}

bool Moss_GetPrefPath(const char* org_name, const char* app_name, char* out_path, int max_len) {
    if (SHGetFolderPathA(NULL, CSIDL_APPDATA, NULL, 0, out_path) != S_OK)
        return false;

    PathAppendA(out_path, org_name);
    CreateDirectoryA(out_path, NULL);
    PathAppendA(out_path, app_name);
    CreateDirectoryA(out_path, NULL);
    return true;
}

/* ---------------------------------------------------------- */
/* Directory enumeration                                     */
/* ---------------------------------------------------------- */

bool Moss_EnumerateDirectory(const char* path, bool recursive, Moss_DirectoryIterateFn callback, void* user_data) {
    char search[MAX_PATH];
    wsprintfA(search, "%s\\*", path);

    WIN32_FIND_DATAA fd;
    HANDLE h = FindFirstFileA(search, &fd);
    if (h == INVALID_HANDLE_VALUE)
        return false;

    do {
        if (!lstrcmpA(fd.cFileName, ".") || !lstrcmpA(fd.cFileName, ".."))
            continue;

        char full[MAX_PATH];
        wsprintfA(full, "%s\\%s", path, fd.cFileName);

        callback(full, user_data);

        if (recursive && (fd.dwFileAttributes & FILE_ATTRIBUTE_DIRECTORY))
            Moss_EnumerateDirectory(full, true, callback, user_data);

    } while (FindNextFileA(h, &fd));

    FindClose(h);
    return true;
}

bool Moss_GlobDirectory(const char* pattern, Moss_DirectoryIterateFn callback, void* user_data) {
    WIN32_FIND_DATAA fd;
    HANDLE h = FindFirstFileA(pattern, &fd);
    if (h == INVALID_HANDLE_VALUE)
        return false;

    do {
        callback(fd.cFileName, user_data);
    } while (FindNextFileA(h, &fd));

    FindClose(h);
    return true;
}



/* ---------------------------------------------------------- */
/* File dialogs (simple, legacy)                              */
/* ---------------------------------------------------------- */

void Moss_ShowOpenFileDialog(Moss_DialogFileCallback callback, void *userdata, Moss_Window *window, const char *default_location, bool allow_many) {
    OPENFILENAMEA ofn = { sizeof(ofn) };
    char buffer[4096] = {0};

    ofn.lpstrFile = buffer;
    ofn.nMaxFile  = sizeof(buffer);
    ofn.Flags     = OFN_EXPLORER | OFN_FILEMUSTEXIST;
    if (allow_many)
        ofn.Flags |= OFN_ALLOWMULTISELECT;

    if (GetOpenFileNameA(&ofn))
        callback(buffer, userdata);
}

void Moss_ShowSaveFileDialog(Moss_FileDialogType type, Moss_DialogFileCallback callback, void *userdata, Moss_PropertiesID props) {
    OPENFILENAMEA ofn = { sizeof(ofn) };
    char buffer[MAX_PATH] = {0};

    ofn.lpstrFile = buffer;
    ofn.nMaxFile  = MAX_PATH;
    ofn.Flags     = OFN_EXPLORER | OFN_OVERWRITEPROMPT;

    if (GetSaveFileNameA(&ofn))
        callback(buffer, userdata);
}

bool Moss_CopyStorageFile(Moss_Storage *storage, const char *oldpath, const char *newpath){
    char src[MAX_PATH], dst[MAX_PATH];
    moss_storage_fullpath(storage, oldpath, src);
    moss_storage_fullpath(storage, newpath, dst);
    return CopyFileA(src, dst, FALSE);
}

bool Moss_CreateStorageDirectory(Moss_Storage *storage, const char *path) {
    char full[MAX_PATH];
    moss_storage_fullpath(storage, path, full);
    return CreateDirectoryA(full, NULL) || GetLastError() == ERROR_ALREADY_EXISTS;
}

bool Moss_EnumerateStorageDirectory(Moss_Storage *storage, const char *path, Moss_EnumerateDirectoryCallback callback, void *userdata) {
    char base[MAX_PATH];
    moss_storage_fullpath(storage, path, base);

    char search[MAX_PATH];
    wsprintfA(search, "%s\\*", base);

    WIN32_FIND_DATAA fd;
    HANDLE h = FindFirstFileA(search, &fd);
    if (h == INVALID_HANDLE_VALUE)
        return false;

    do {
        if (!lstrcmpA(fd.cFileName, ".") ||
            !lstrcmpA(fd.cFileName, ".."))
            continue;

        callback(fd.cFileName, userdata);
    } while (FindNextFileA(h, &fd));

    FindClose(h);
    return true;
}

bool Moss_GetStorageFileSize(Moss_Storage *storage, const char *path, uint64 *length) {
    char full[MAX_PATH];
    moss_storage_fullpath(storage, path, full);

    WIN32_FILE_ATTRIBUTE_DATA data;
    if (!GetFileAttributesExA(full, GetFileExInfoStandard, &data))
        return false;

    LARGE_INTEGER size;
    size.HighPart = data.nFileSizeHigh;
    size.LowPart  = data.nFileSizeLow;
    *length = (uint64)size.QuadPart;
    return true;
}

uint64_t Moss_GetStorageSpaceRemaining(Moss_Storage *storage) {
    ULARGE_INTEGER free_bytes;
    if (!GetDiskFreeSpaceExA(storage->root, &free_bytes, NULL, NULL))
        return 0;
    return (uint64_t)free_bytes.QuadPart;
}

bool Moss_GetStoragePathInfo(Moss_Storage *storage, const char *path, Moss_PathInfo *info) {
    char full[MAX_PATH];
    moss_storage_fullpath(storage, path, full);
    return Moss_GetPathInfo(full, info);
}

char** Moss_GlobStorageDirectory(Moss_Storage *storage, const char *path, const char *pattern, Moss_GlobFlags flags, int *count) {
    char base[MAX_PATH];
    moss_storage_fullpath(storage, path, base);

    char search[MAX_PATH];
    wsprintfA(search, "%s\\%s", base, pattern);

    WIN32_FIND_DATAA fd;
    HANDLE h = FindFirstFileA(search, &fd);
    if (h == INVALID_HANDLE_VALUE) {
        *count = 0;
        return NULL;
    }

    int cap = 16;
    int n = 0;
    char **results = (char**)malloc(sizeof(char*) * cap);

    do {
        if (!lstrcmpA(fd.cFileName, ".") ||
            !lstrcmpA(fd.cFileName, ".."))
            continue;

        if (n == cap) {
            cap *= 2;
            results = (char**)realloc(results, sizeof(char*) * cap);
        }

        results[n] = _strdup(fd.cFileName);
        n++;
    } while (FindNextFileA(h, &fd));

    FindClose(h);
    *count = n;
    return results;
}

Moss_Storage* Moss_OpenStorage(const Moss_Storage *iface, void *userdata) {
    Moss_Storage *s = (Moss_Storage*)malloc(sizeof(Moss_Storage));
    *s = *iface;
    return s;
}
Moss_Storage* Moss_OpenTitleStorage(const char *override, Moss_PropertiesID props) {
    char path[MAX_PATH];
    if (override)
        lstrcpyA(path, override);
    else
        GetModuleFileNameA(NULL, path, MAX_PATH);

    PathRemoveFileSpecA(path);
    return Moss_OpenFileStorage(path);
}

Moss_Storage* Moss_OpenUserStorage(const char *org, const char *app, Moss_PropertiesID props){
    char path[MAX_PATH];
    if (!Moss_GetPrefPath(org, app, path, MAX_PATH))
        return NULL;
    return Moss_OpenFileStorage(path);
}

bool Moss_RemoveStoragePath(Moss_Storage *storage, const char *path) {
    char full[MAX_PATH];
    moss_storage_fullpath(storage, path, full);
    return Moss_RemovePath(full, false);
}

bool Moss_RenameStoragePath(Moss_Storage *storage, const char *oldpath, const char *newpath) {
    char oldf[MAX_PATH], newf[MAX_PATH];
    moss_storage_fullpath(storage, oldpath, oldf);
    moss_storage_fullpath(storage, newpath, newf);
    return MoveFileExA(oldf, newf, MOVEFILE_REPLACE_EXISTING);
}

bool Moss_StorageReady(Moss_Storage *storage) {
    DWORD attrs = GetFileAttributesA(storage->root);
    return attrs != INVALID_FILE_ATTRIBUTES && (attrs & FILE_ATTRIBUTE_DIRECTORY);
}


Moss_Storage* Moss_OpenFileStorage(const char *path) {
    Moss_Storage* s = (Moss_Storage*)malloc(sizeof(Moss_Storage));
    lstrcpyA(s->root, path);
    return s;
}

bool Moss_ReadStorageFile(Moss_Storage *storage, const char *path, void *destination, uint64 length) {
    char full[MAX_PATH];
    wsprintfA(full, "%s\\%s", storage->root, path);

    HANDLE h = CreateFileA(full, GENERIC_READ, FILE_SHARE_READ, NULL, OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, NULL);
    if (h == INVALID_HANDLE_VALUE)
        return false;

    DWORD read;
    bool ok = ReadFile(h, destination, (DWORD)length, &read, NULL);
    CloseHandle(h);
    return ok;
}

bool Moss_WriteStorageFile(Moss_Storage *storage, const char *path, const void *source, uint64 length) {
    char full[MAX_PATH];
    wsprintfA(full, "%s\\%s", storage->root, path);

    HANDLE h = CreateFileA(full, GENERIC_WRITE, 0, NULL, CREATE_ALWAYS, FILE_ATTRIBUTE_NORMAL, NULL);
    if (h == INVALID_HANDLE_VALUE)
        return false;

    DWORD written;
    bool ok = WriteFile(h, source, (DWORD)length, &written, NULL);
    CloseHandle(h);
    return ok;
}

bool Moss_CloseStorage(Moss_Storage *storage) { free(storage); return true; }