#include <Moss/Moss_Audio.h>




AudioChannel* GetChannel(ChannelID id) {
    auto it = g_channels.find(id);
    if (it == g_channels.end()) {
        return nullptr;
    }
    return &it->second; // or 'return it->second;' if the map stores AudioChannel*
}