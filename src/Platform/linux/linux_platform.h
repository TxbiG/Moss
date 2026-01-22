#ifndef MOSS_PLATFORM_LINUX_H
#define MOSS_PLATFORM_LINUX_H

// TODO: Add Support for Xbox and Playstation (4 & 5) Controllers

#include <Moss/Platform/platform_intern.h>


// Inputs
#include <linux/input.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdio.h>

#include <dlfcn.h>
#include <sys/sysinfo.h>
#include <sys/types.h>
#include <sys/param.h>
#include <sys/sysctl.h>

void ListenController(const char* device) {
    int fd = open(device, O_RDONLY);
    if (fd < 0) { perror("open"); return; }

    struct input_event ev;
    while (1) {
        if (read(fd, &ev, sizeof(ev)) == sizeof(ev)) {
            if (ev.type == EV_KEY) {
                printf("Button %d %s\n", ev.code, ev.value ? "pressed" : "released");
            } else if (ev.type == EV_ABS) {
                printf("Axis %d = %d\n", ev.code, ev.value);
            }
        }
    }
}


static char* strdup_safe(const char* s) {
    if (!s) return NULL;
    size_t len = strlen(s);
    char* r = (char*)malloc(len + 1);
    if (r) memcpy(r, s, len + 1);
    return r;
}

Moss_Locale* Moss_GetLocale() {
    const char* lc = setlocale(LC_CTYPE, "");
    if (!lc) lc = "en_US.UTF-8";

    char lang[8] = {0};
    char country[8] = {0};

    sscanf(lc, "%2[^_]", lang);
    sscanf(lc, "%*2[^_]_%2s", country);

    if (lang[0] == '\0') strcpy(lang, "en");
    if (country[0] == '\0') strcpy(country, "US");

    locale->language = strdup_safe(lang);
    locale->country  = strdup_safe(country);
}



int Moss_GetAvailableCPUCores(void) {
    long nprocs = sysconf(_SC_NPROCESSORS_ONLN);
    return (int)((nprocs > 0) ? nprocs : 1);
}

MOSS_API int Moss_GetCPUCacheLineSize(void) {
    FILE *fp = fopen("/sys/devices/system/cpu/cpu0/cache/index0/coherency_line_size", "r");
    if (!fp) return 64;
    int line_size = 64;
    fscanf(fp, "%d", &line_size);
    fclose(fp);
    return line_size;
}

MOSS_API int Moss_GetSystemRAM(void) {
    struct sysinfo info;
    if (sysinfo(&info) == 0)
        return (int)(info.totalram * info.mem_unit / (1024 * 1024));
    return -1;
}


bool Moss_OpenURL(const char *url) {}
// ---------------------------------------------------------------------------
// Dynamic Library Management
// ---------------------------------------------------------------------------
void* Moss_LoadDynamicLibrary(const char* lib_path) { return dlopen(lib_path, RTLD_NOW | RTLD_LOCAL); }
void* Moss_GetLibrarySymbol(void* handle, const char* symbol_name) { return dlsym(handle, symbol_name); }
MOSS_API void Moss_UnloadDynamicLibrary(void* handle) { if (handle) dlclose(handle); }

#endif // MOSS_PLATFORM_LINUX_H