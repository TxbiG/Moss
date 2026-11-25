#include "audio_intern.h"


std::unordered_map<ChannelID, AudioChannel> g_channels;

AudioChannel* Moss_AudioGetChannel(ChannelID id) {
    auto it = g_channels.find(id);
    if (it == g_channels.end()) {
        return nullptr;
    }
    return &it->second;
}