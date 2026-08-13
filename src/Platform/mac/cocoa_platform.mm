#include "cocoa_platform.h"

static NSArray<NSString*>* Moss_FileTypesFromFilters(const Moss_DialogFileFilter* filters, int nfilters) {
    NSMutableArray<NSString*>* types = [NSMutableArray array];
    for (int i = 0; filters && i < nfilters; ++i) {
        const char* pattern = filters[i].pattern;
        if (!pattern) continue;
        NSString* value = [NSString stringWithUTF8String:pattern];
        NSArray<NSString*>* parts = [value componentsSeparatedByCharactersInSet:[NSCharacterSet characterSetWithCharactersInString:@";, "]];
        for (NSString* part in parts) {
            NSString* trimmed = [part stringByTrimmingCharactersInSet:[NSCharacterSet whitespaceAndNewlineCharacterSet]];
            if ([trimmed hasPrefix:@"*."]) trimmed = [trimmed substringFromIndex:2];
            else if ([trimmed hasPrefix:@"."]) trimmed = [trimmed substringFromIndex:1];
            if ([trimmed length] > 0 && ![trimmed isEqualToString:@"*"]) [types addObject:trimmed];
        }
    }
    return [types count] > 0 ? types : nil;
}

static void Moss_InvokeDialogCallback(Moss_DialogFileCallback callback, void* userdata, NSURL* url) {
    if (!callback || !url) return;
    NSString* path = [url path];
    if (!path) return;
    const char* utf8 = [path UTF8String];
    if (!utf8) return;
    const char* files[2] = { utf8, nullptr };
    callback(userdata, files, 0);
}

void Moss_ShowFileDialogWithProperties(Moss_DialogFileCallback callback, void* userdata, Moss_Window* window, const Moss_DialogFileFilter* filters, int nfilters, const char* default_location) {
    @autoreleasepool {
        NSOpenPanel* panel = [NSOpenPanel openPanel];
        [panel setCanCreateDirectories:YES];
        [panel setCanChooseFiles:YES];
        [panel setCanChooseDirectories:NO];
        [panel setAllowsMultipleSelection:NO];
        NSArray<NSString*>* types = Moss_FileTypesFromFilters(filters, nfilters);
        if (types) [panel setAllowedFileTypes:types];
        if (default_location) [panel setDirectoryURL:[NSURL fileURLWithPath:[NSString stringWithUTF8String:default_location]]];
        if ([panel runModal] != NSModalResponseOK) return;
        Moss_InvokeDialogCallback(callback, userdata, [panel URL]);
    }
}

void Moss_ShowOpenFileDialog(Moss_DialogFileCallback callback, void* userdata, Moss_Window* window, const char* default_location, bool allow_many) {
    @autoreleasepool {
        NSOpenPanel* panel = [NSOpenPanel openPanel];
        [panel setCanCreateDirectories:YES];
        [panel setCanChooseFiles:YES];
        [panel setCanChooseDirectories:NO];
        [panel setAllowsMultipleSelection:allow_many ? YES : NO];
        if (default_location) [panel setDirectoryURL:[NSURL fileURLWithPath:[NSString stringWithUTF8String:default_location]]];

        NSModalResponse response = window && window->object
            ? [panel runModal]
            : [panel runModal];
        if (response != NSModalResponseOK) return;

        for (NSURL* url in [panel URLs]) {
            Moss_InvokeDialogCallback(callback, userdata, url);
            if (!allow_many) break;
        }
    }
}

void Moss_ShowOpenFolderDialog(Moss_DialogFileCallback callback, void* userdata, Moss_Window* window, const Moss_DialogFileFilter* filters, int nfilters, const char* default_location, bool allow_many) {
    (void)filters;
    (void)nfilters;
    @autoreleasepool {
        NSOpenPanel* panel = [NSOpenPanel openPanel];
        [panel setCanChooseFiles:NO];
        [panel setCanChooseDirectories:YES];
        [panel setAllowsMultipleSelection:allow_many ? YES : NO];
        if (default_location) [panel setDirectoryURL:[NSURL fileURLWithPath:[NSString stringWithUTF8String:default_location]]];

        if ([panel runModal] != NSModalResponseOK) return;
        for (NSURL* url in [panel URLs]) {
            Moss_InvokeDialogCallback(callback, userdata, url);
            if (!allow_many) break;
        }
    }
}

void Moss_ShowSaveFileDialog(Moss_FileDialogType type, Moss_DialogFileCallback callback, void* userdata, Moss_PropertiesID props) {
    (void)props;
    @autoreleasepool {
        if (type == Moss_FileDialogType::OPENFOLDER) {
            Moss_ShowOpenFolderDialog(callback, userdata, nullptr, nullptr, 0, nullptr, false);
            return;
        }
        NSSavePanel* panel = [NSSavePanel savePanel];
        [panel setCanCreateDirectories:YES];
        [panel setExtensionHidden:NO];
        if ([panel runModal] != NSModalResponseOK) return;
        Moss_InvokeDialogCallback(callback, userdata, [panel URL]);
    }
}