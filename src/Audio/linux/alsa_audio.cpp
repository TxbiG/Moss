
#include "alsa_audio.h"
#include <algorithm>
#include <cmath>
#include <cstring>

#include <alsa/asoundlib.h>

struct ALSAPlayback {
    snd_pcm_t* handle = nullptr;
    snd_pcm_hw_params_t* hwParams = nullptr;
    unsigned int sampleRate = 44100;
    int channels = 2;
    snd_pcm_format_t format = SND_PCM_FORMAT_FLOAT_LE;
    Speaker::RenderCallback renderCallback;
    std::atomic<bool> running = false;
    std::thread playbackThread;
};
static ALSAPlayback alsa;

AudioListener2D g_listener2D = { 0.0f, 0.0f };
AudioListener3D g_listener3D = { 0.0f, 0.0f, 0.0f };

int Moss_Init_Audio() {
    int err;

    if ((err = snd_pcm_open(&alsa.handle, "default", SND_PCM_STREAM_PLAYBACK, 0)) < 0) { fprintf(stderr, "ALSA: Failed to open device: %s\n", snd_strerror(err));  return -1; }

    snd_pcm_hw_params_alloca(&alsa.hwParams);
    snd_pcm_hw_params_any(alsa.handle, alsa.hwParams);
    snd_pcm_hw_params_set_access(alsa.handle, alsa.hwParams, SND_PCM_ACCESS_RW_INTERLEAVED);
    snd_pcm_hw_params_set_format(alsa.handle, alsa.hwParams, alsa.format);
    snd_pcm_hw_params_set_channels(alsa.handle, alsa.hwParams, alsa.channels);
    snd_pcm_hw_params_set_rate_near(alsa.handle, &alsa.hwParams, &alsa.sampleRate, 0);
    snd_pcm_hw_params(alsa.handle, alsa.hwParams);

    return 0;
}

void Moss_Terminate_Audio() {
    alsa.running = false;

    if (alsa.playbackThread.joinable())
        alsa.playbackThread.join();

    if (alsa.handle) {
        snd_pcm_drain(alsa.handle);
        snd_pcm_close(alsa.handle);
        alsa.handle = nullptr;
    }
}


Wav* loadWav(const char* path) {
    std::ifstream file(path, std::ios::binary);
    if (!file.is_open())
        return nullptr;

    auto* wav = new Wav{};
    
    // Read RIFF Header
    file.read(reinterpret_cast<char*>(&wav->riffChunkId), 4);       // "RIFF"
    file.read(reinterpret_cast<char*>(&wav->riffChunkSize), 4);
    file.read(reinterpret_cast<char*>(&wav->format), 4);            // "WAVE"

    // Check identifiers
    if (wav->riffChunkId != 0x46464952 || wav->format != 0x45564157) { // 'RIFF', 'WAVE' in little-endian
        delete wav;
        return nullptr;
    }

    // Read fmt chunk
    file.read(reinterpret_cast<char*>(&wav->formatChunkId), 4);     // "fmt "
    file.read(reinterpret_cast<char*>(&wav->formatChunkSize), 4);
    file.read(reinterpret_cast<char*>(&wav->audioFormat), 2);
    file.read(reinterpret_cast<char*>(&wav->numChannels), 2);
    file.read(reinterpret_cast<char*>(&wav->sampleRate), 4);
    file.read(reinterpret_cast<char*>(&wav->byteRate), 4);
    file.read(reinterpret_cast<char*>(&wav->blockAlign), 2);
    file.read(reinterpret_cast<char*>(&wav->bitsPerSample), 2);

    // Skip any extra fmt bytes
    if (wav->formatChunkSize > 16)
        file.ignore(wav->formatChunkSize - 16);

    // Read until we find the "data" chunk
    while (true) {
        file.read(reinterpret_cast<char*>(&wav->dataChunkId), 4);
        file.read(reinterpret_cast<char*>(&wav->dataChunkSize), 4);

        if (std::memcmp(wav->dataChunkId, "data", 4) == 0)
            break;

        file.ignore(wav->dataChunkSize);
    }

    // Allocate and read raw PCM data
    wav->dataBegin = new char[wav->dataChunkSize];
    file.read(wav->dataBegin, wav->dataChunkSize);

    if (!file) {
        delete[] wav->dataBegin;
        delete wav;
        return nullptr;
    }

    return wav;
}

void RemoveWav(Wav* wav) {
    if (!wav) return;
    delete[] wav->dataBegin;
    delete wav;
}

void SetAudioListener2D(Float2* position) { if (!gListener2D) gListener2D = new AudioListener2D(*position); else gListener2D->position = *position; }
void SetAudioListener3D(Float3* position) { if (!gListener3D) gListener3D = new AudioListener3D(*position); else gListener3D->position = *position; }
void SetAudioListenerVelocity3D(Float3* velocity) { if (gListener3D) gListener3D->velocity = *velocity; }


static bool PlayWavWithALSA(Wav& wav, bool loop, float volume) {
    snd_pcm_t* pcm;
    snd_pcm_hw_params_t* params;
    int err;

    if ((err = snd_pcm_open(&pcm, "default", SND_PCM_STREAM_PLAYBACK, 0)) < 0) {
        fprintf(stderr, "ALSA open error: %s\n", snd_strerror(err));
        return false;
    }

    snd_pcm_hw_params_malloc(&params);
    snd_pcm_hw_params_any(pcm, params);
    snd_pcm_hw_params_set_access(pcm, params, SND_PCM_ACCESS_RW_INTERLEAVED);

    snd_pcm_format_t format;
    switch (wav.bitsPerSample) {
        case 8:  format = SND_PCM_FORMAT_U8; break;
        case 16: format = SND_PCM_FORMAT_S16_LE; break;
        case 24: format = SND_PCM_FORMAT_S24_LE; break;
        case 32: format = SND_PCM_FORMAT_S32_LE; break;
        default:
            fprintf(stderr, "Unsupported format.\n");
            return false;
    }

    snd_pcm_hw_params_set_format(pcm, params, format);
    snd_pcm_hw_params_set_channels(pcm, params, wav.numChannels);
    unsigned int rate = wav.sampleRate;
    snd_pcm_hw_params_set_rate_near(pcm, params, &rate, nullptr);

    if ((err = snd_pcm_hw_params(pcm, params)) < 0) {
        fprintf(stderr, "ALSA param error: %s\n", snd_strerror(err));
        snd_pcm_close(pcm);
        return false;
    }

    snd_pcm_hw_params_free(params);

    int frameSize = wav.blockAlign;
    int totalFrames = wav.dataChunkSize / frameSize;
    char* audio = wav.dataBegin;

    do {
        int written = snd_pcm_writei(pcm, audio, totalFrames);
        if (written < 0) {
            snd_pcm_prepare(pcm);
            written = snd_pcm_writei(pcm, audio, totalFrames);
        }
    } while (loop);

    snd_pcm_drain(pcm);
    snd_pcm_close(pcm);
    return true;
}
#endif


void AudioStream::play() {
    if (playing) return;
    playing = true;
    PlayWavWithALSA(wav, loop, volume);
}

void AudioStream::stop() {
    // Not implemented with ALSA directly (requires async/thread control)
    playing = false;
}

void AudioStream2D::play() {
    // Assume global listener2D exists
    extern AudioListener2D* gListener2D;

    float distance = (gListener2D->position - position).length();
    float distFactor = std::max(0.0f, 1.0f - (distance / maxDistance));

    // Pan left (-1) to right (+1)
    float deltaX = position.x - gListener2D->position.x;
    float pan = std::clamp(deltaX / maxDistance, -1.0f, 1.0f);

    stream.setVolume(distFactor);
    stream.setPan(pan);
    stream.play();
}
void AudioStream2D::stop() { stream.stop(); }

void AudioStream3D::play() {
    extern AudioListener3D* gListener3D;

    Float3 delta = position - gListener3D->position;
    float distance = delta.length();
    float distFactor = std::max(0.0f, 1.0f - (distance / maxDistance));

    // Pan (assume X axis dominates stereo field)
    float pan = std::clamp(delta.x / maxDistance, -1.0f, 1.0f);

    stream.setVolume(distFactor);
    stream.setPan(pan);
    stream.play();
}

void AudioStream3D::stop() {
    stream.stop();
}

bool Speaker::start() {
    if (!alsa.renderCallback || !alsa.handle)
        return false;

    alsa.running = true;
    alsa.playbackThread = std::thread([]() {
        const int frames = 512;
        const int channels = alsa.channels;
        float buffer[frames * channels];

        while (alsa.running) {
            memset(buffer, 0, sizeof(buffer));

            // Call game audio mix callback
            size_t filled = alsa.renderCallback(buffer, frames);

            int err = snd_pcm_writei(alsa.handle, buffer, frames);
            if (err == -EPIPE) { snd_pcm_prepare(alsa.handle); } // Buffer underrun
            else if (err < 0) { fprintf(stderr, "ALSA write error: %s\n", snd_strerror(err)); }
        }
    });
    return true;
}
void Speaker::stop() {
    alsa.running = false;
    if (alsa.playbackThread.joinable())
        alsa.playbackThread.join();
}

void Speaker::setRenderCallback(RenderCallback cb) {
    alsa.renderCallback = std::move(cb);
}









static MicrophoneCallback = nullptr;
static AudioStreamCallback = nullptr;



int Moss_Init_Audio();

void Moss_Terminate_Audio();

void Moss_AudioUpdate(float deltaTime);


Moss_AudioSource* Moss_AudioLoadWav();

Moss_AudioSource* Moss_AudioLoadOgg(const char* filename, AudioLoadType type);

Moss_AudioSource* Moss_AudioLoadMP3(const char* filename);

Moss_AudioSource* Moss_AudioCaptureMicrophone(Microphone* mic);

// Effects

AudioEffect Moss_AudioCreateEffect(AudioEffectType type);

void Moss_AudioCreateEffect(AudioEffect* effect, const char* paramName, float value);

void Moss_AudioRemoveEffect(AudioEffect* effect);

// Channel

ChannelID Moss_AudioCreateChannel(ChannelID channel);
void Audio_RemoveChannel(ChannelID channel);

ChannelID Moss_AudioGetMasterChannel();

void Moss_AudioSetChannelVolume(ChannelID channel, float volume);

void Moss_AudioSetChannelMute(ChannelID channel, bool mute);

void Moss_AudioAddChannelEffect(ChannelID channel, AudioEffect* effect);

void Moss_AudioRemoveChannelEffect(ChannelID channel, AudioEffect* effect);

void Moss_AudioRemoveAllChannelEffects(ChannelID channel);


AudioStream* Moss_AudioStreamCreate();

void Moss_AudioStreamPlay(AudioStream* audiostream);

void Moss_AudioStreamStop(AudioStream* audiostream);

void Moss_AudioStreamSetVolume(AudioStream* audiostream, float volume);

void Moss_AudioStreamSetPitch(AudioStream* audiostream, float pitch);

void Moss_AudioStreamSetLoop(AudioStream* audiostream, bool loop);

void Moss_AudioStreamRemove(AudioStream* audiostream);


AudioStream2D* Moss_AudioStream2DCreate();

void Moss_AudioStream2DPlay(AudioStream2D* audiostream);

void Moss_AudioStream2DStop(AudioStream2D* audiostream);

void Moss_AudioStream2DSetVolume(AudioStream2D* audiostream, float volume);

void Moss_AudioStream2DSetPitch(AudioStream2D* audiostream, float pitch);

void Moss_AudioStream2DSetLoop(AudioStream2D* audiostream, bool loop);

void Moss_AudioStream3DSetPosition(AudioStream2D* audiostream);

void Moss_AudioStream3DSetVelocity(AudioStream2D* audiostream);

void Moss_AudioStream3DSetMaxDistance(AudioStream2D* audiostream);

void Moss_AudioStream2DRemove(AudioStream2D* audiostream);


AudioStream3D* Moss_AudioStream3DCreate();

void Moss_AudioStream3DPlay(AudioStream3D* audiostream);

void Moss_AudioStream3DStop(AudioStream3D* audiostream);

void Moss_AudioStream3DSetVolume(AudioStream3D* audiostream, float volume);

void Moss_AudioStream3DSetPitch(AudioStream3D* audiostream, float pitch);

void Moss_AudioStream3DSetLoop(AudioStream3D* audiostream, bool loop);

void Moss_AudioStream3DSetPosition(AudioStream3D* audiostream);

void Moss_AudioStream3DSetVelocity(AudioStream3D* audiostream);

void Moss_AudioStream3DSetMaxDistance(AudioStream3D* audiostream);

void Moss_AudioStream3DSetDistanceModel(AudioStream3D* stream, DistanceModel model);

void Moss_AudioStream3DRemove(AudioStream3D* audiostream);

// Listeners

AudioListener2D* Moss_AudioCreateAudioListener2D();

AudioListener3D* Moss_AudioCreateAudioListener3D();

RayAudioListener2D* Moss_AudioCreateRayAudioListener2D();

RayAudioListener3D* Moss_AudioCreateRayAudioListener3D();


void Moss_AudioRemoveAudioListener2D(AudioListener2D* listener);

void Moss_AudioRemoveAudioListener3D(AudioListener3D* listener);

void Moss_AudioRemoveRayAudioListener2D(RayAudioListener2D* listener);

void Moss_AudioRemoveRayAudioListener3D(RayAudioListener3D* listener);


void Moss_AudioActivateAudioListener2D(AudioListener2D* listener, bool activate);

void Moss_AudioActivateAudioListener3D(AudioListener3D* listener, bool activate);

void Moss_AudioActivateRayAudioListener2D(RayAudioListener2D* listener, bool activate);

void Moss_AudioActivateRayAudioListener3D(RayAudioListener3D* listener, bool activate);

void Moss_AudioListenerSetOrientation(AudioListener3D* listener, const Vec3& forward, const Vec3& up);

// Speakers

bool Moss_IsSpeakerDeviceReady();

void Moss_AudioSpeakerOpen();

void Moss_AudioSpeakerPause();

void Moss_AudioSpeakerResume();

bool Moss_AudioSpeakerIsPaused();
/*! @brief X. @param X X. */
bool Moss_AudioSelectSpeakerDevice(int id);

int Moss_GetCurrentSpeakerDeviceID();

const char* Moss_GetSpeakerDeviceName(int id);

int Moss_ListSpeakerDevices();

// Microphone

bool Moss_IsMicrophoneDeviceReady();

int Moss_AudioMicrophoneOpen();

void Moss_AudioMicrophoneClose();

void Moss_AudioMicrophonePlay();

void Moss_AudioMicrophoneStop();

int Moss_AudioMicrophoneID();

bool Moss_AudioSelectMicrophoneDevice(int id);

const char* Moss_GetMicrophoneDeviceName(int index);

int Moss_ListMicrophoneDevices();


void Moss_AudioMicrophoneSetGain(Microphone* mic, float gain);

int Moss_AudioMicrophoneGetSampleRate(Microphone* mic);

int Moss_AudioMicrophoneGetChannels(Microphone* mic);


void Moss_AudioStreamSetCallback(AudioStream* stream, AudioStreamCallback callback, void* userData);

void Moss_AudioMicrophoneSetCallback(Microphone* mic, MicrophoneCallback callback, void* userData);




/*



struct ALSAPlayback {
    snd_pcm_t* handle = nullptr;
    snd_pcm_hw_params_t* hwParams = nullptr;
    unsigned int sampleRate = 44100;
    int channels = 2;
    snd_pcm_format_t format = SND_PCM_FORMAT_FLOAT_LE;
    Speaker::RenderCallback renderCallback;
    std::atomic<bool> running = false;
    std::thread playbackThread;
};
static ALSAPlayback alsa;

AudioListener2D g_listener2D = { 0.0f, 0.0f };
AudioListener3D g_listener3D = { 0.0f, 0.0f, 0.0f };

int Moss_Init_Audio() {
    int err;

    if ((err = snd_pcm_open(&alsa.handle, "default", SND_PCM_STREAM_PLAYBACK, 0)) < 0) { fprintf(stderr, "ALSA: Failed to open device: %s\n", snd_strerror(err));  return -1; }

    snd_pcm_hw_params_alloca(&alsa.hwParams);
    snd_pcm_hw_params_any(alsa.handle, alsa.hwParams);
    snd_pcm_hw_params_set_access(alsa.handle, alsa.hwParams, SND_PCM_ACCESS_RW_INTERLEAVED);
    snd_pcm_hw_params_set_format(alsa.handle, alsa.hwParams, alsa.format);
    snd_pcm_hw_params_set_channels(alsa.handle, alsa.hwParams, alsa.channels);
    snd_pcm_hw_params_set_rate_near(alsa.handle, alsa.hwParams, &alsa.sampleRate, nullptr);
    snd_pcm_hw_params(alsa.handle, alsa.hwParams);

    return 0;
}

void Moss_Terminate_Audio() {
    alsa.running = false;

    if (alsa.playbackThread.joinable())
        alsa.playbackThread.join();

    if (alsa.handle) {
        snd_pcm_drain(alsa.handle);
        snd_pcm_close(alsa.handle);
        alsa.handle = nullptr;
    }
}


Wav* loadWav(const char* path) {
    std::ifstream file(path, std::ios::binary);
    if (!file.is_open())
        return nullptr;

    auto* wav = new Wav{};
    
    // Read RIFF Header
    file.read(reinterpret_cast<char*>(&wav->riffChunkId), 4);       // "RIFF"
    file.read(reinterpret_cast<char*>(&wav->riffChunkSize), 4);
    file.read(reinterpret_cast<char*>(&wav->format), 4);            // "WAVE"

    // Check identifiers
    if (wav->riffChunkId != 0x46464952 || wav->format != 0x45564157) { // 'RIFF', 'WAVE' in little-endian
        delete wav;
        return nullptr;
    }

    // Read fmt chunk
    file.read(reinterpret_cast<char*>(&wav->formatChunkId), 4);     // "fmt "
    file.read(reinterpret_cast<char*>(&wav->formatChunkSize), 4);
    file.read(reinterpret_cast<char*>(&wav->audioFormat), 2);
    file.read(reinterpret_cast<char*>(&wav->numChannels), 2);
    file.read(reinterpret_cast<char*>(&wav->sampleRate), 4);
    file.read(reinterpret_cast<char*>(&wav->byteRate), 4);
    file.read(reinterpret_cast<char*>(&wav->blockAlign), 2);
    file.read(reinterpret_cast<char*>(&wav->bitsPerSample), 2);

    // Skip any extra fmt bytes
    if (wav->formatChunkSize > 16)
        file.ignore(wav->formatChunkSize - 16);

    // Read until we find the "data" chunk
    while (true) {
        file.read(reinterpret_cast<char*>(&wav->dataChunkId), 4);
        file.read(reinterpret_cast<char*>(&wav->dataChunkSize), 4);

        if (std::memcmp(wav->dataChunkId, "data", 4) == 0)
            break;

        file.ignore(wav->dataChunkSize);
    }

    // Allocate and read raw PCM data
    wav->dataBegin = new char[wav->dataChunkSize];
    file.read(wav->dataBegin, wav->dataChunkSize);

    if (!file) {
        delete[] wav->dataBegin;
        delete wav;
        return nullptr;
    }

    return wav;
}

void RemoveWav(Wav* wav) {
    if (!wav) return;
    delete[] wav->dataBegin;
    delete wav;
}

void SetAudioListener2D(Float2* position) { if (position) g_listener2D.position = *position; }
void SetAudioListener3D(Float3* position) { if (position) g_listener3D.position = *position; }
void SetAudioListenerVelocity3D(Float3* velocity) { if (velocity) g_listener3D.velocity = *velocity; }


static bool PlayWavWithALSA(AudioStream* stream) {
    if (!stream || !stream->wav || !stream->wav->dataBegin || stream->wav->blockAlign == 0) {
        return false;
    }

    Wav* wav = stream->wav;
    snd_pcm_t* pcm = nullptr;
    snd_pcm_hw_params_t* params = nullptr;
    int err = 0;

    if ((err = snd_pcm_open(&pcm, "default", SND_PCM_STREAM_PLAYBACK, 0)) < 0) {
        fprintf(stderr, "ALSA open error: %s\n", snd_strerror(err));
        return false;
    }
    stream->pcm = pcm;

    snd_pcm_hw_params_malloc(&params);
    snd_pcm_hw_params_any(pcm, params);
    snd_pcm_hw_params_set_access(pcm, params, SND_PCM_ACCESS_RW_INTERLEAVED);

    snd_pcm_format_t format;
    switch (wav->bitsPerSample) {
        case 8:  format = SND_PCM_FORMAT_U8; break;
        case 16: format = SND_PCM_FORMAT_S16_LE; break;
        case 24: format = SND_PCM_FORMAT_S24_LE; break;
        case 32: format = SND_PCM_FORMAT_S32_LE; break;
        default:
            fprintf(stderr, "Unsupported WAV bit depth: %u\n", wav->bitsPerSample);
            snd_pcm_hw_params_free(params);
            snd_pcm_close(pcm);
            stream->pcm = nullptr;
            return false;
    }

    snd_pcm_hw_params_set_format(pcm, params, format);
    snd_pcm_hw_params_set_channels(pcm, params, wav->numChannels);
    unsigned int rate = wav->sampleRate;
    snd_pcm_hw_params_set_rate_near(pcm, params, &rate, nullptr);

    if ((err = snd_pcm_hw_params(pcm, params)) < 0) {
        fprintf(stderr, "ALSA param error: %s\n", snd_strerror(err));
        snd_pcm_hw_params_free(params);
        snd_pcm_close(pcm);
        stream->pcm = nullptr;
        return false;
    }

    snd_pcm_hw_params_free(params);

    const int frame_size = wav->blockAlign;
    const int total_frames = static_cast<int>(wav->dataChunkSize / frame_size);
    const int chunk_frames = 512;

    do {
        int frame = 0;
        while (!stream->stopRequested.load() && frame < total_frames) {
            const int frames_to_write = std::min(chunk_frames, total_frames - frame);
            const char* audio = wav->dataBegin + static_cast<size_t>(frame) * frame_size;
            snd_pcm_sframes_t written = snd_pcm_writei(pcm, audio, frames_to_write);
            if (written == -EPIPE) {
                snd_pcm_prepare(pcm);
                continue;
            }
            if (written < 0) {
                fprintf(stderr, "ALSA write error: %s\n", snd_strerror(static_cast<int>(written)));
                break;
            }
            frame += static_cast<int>(written);
        }
    } while (stream->loop && !stream->stopRequested.load());

    if (stream->stopRequested.load()) {
        snd_pcm_drop(pcm);
    } else {
        snd_pcm_drain(pcm);
    }
    snd_pcm_close(pcm);
    stream->pcm = nullptr;
    return true;
}



void AudioStream::setVolume(float value) {
    volume = std::clamp(value, 0.0f, 1.0f);
}

void AudioStream::setPitch(float value) {
    pitch = std::clamp(value, 0.25f, 4.0f);
}

void AudioStream::setPlaybackRate(float value) {
    playbackRate = std::clamp(value, 0.25f, 4.0f);
    setPitch(playbackRate);
}

void AudioStream::setPan(float value) {
    pan = std::clamp(value, -1.0f, 1.0f);
}

void AudioStream::setLooping(bool value) {
    loop = value;
}
AudioStream::~AudioStream() {
    stop();
}

void AudioStream::play() {
    if (playing.load()) return;
    if (playbackThread.joinable()) {
        playbackThread.join();
    }
    stopRequested = false;
    playing = true;
    playbackThread = std::thread([this]() {
        PlayWavWithALSA(this);
        playing = false;
    });
}

void AudioStream::stop() {
    stopRequested = true;
    if (pcm) {
        snd_pcm_drop(pcm);
    }
    if (playbackThread.joinable()) {
        playbackThread.join();
    }
    playing = false;
}

void AudioStream2D::play() {
    AudioListener2D* listener = &g_listener2D;
    const float distance = (listener->position - position).length();
    const float distFactor = std::max(0.0f, 1.0f - (distance / std::max(maxDistance, 0.001f)));
    const float deltaX = position.x - listener->position.x;
    const float computedPan = std::clamp(deltaX / std::max(maxDistance, 0.001f), -1.0f, 1.0f);

    stream.setVolume(distFactor);
    stream.setPan(computedPan);
    stream.play();
}
void AudioStream2D::stop() { stream.stop(); }

void AudioStream3D::play() {
    AudioListener3D* listener = &g_listener3D;
    Float3 delta = position - listener->position;
    const float distance = delta.length();
    const float distFactor = std::max(0.0f, 1.0f - (distance / std::max(maxDistance, 0.001f)));
    const float computedPan = std::clamp(delta.x / std::max(maxDistance, 0.001f), -1.0f, 1.0f);

    stream.setVolume(distFactor);
    stream.setPan(computedPan);
    stream.play();
}

void AudioStream3D::stop() {
    stream.stop();
}

bool Speaker::start() {
    if (!alsa.renderCallback || !alsa.handle)
        return false;

    alsa.running = true;
    alsa.playbackThread = std::thread([]() {
        const int frames = 512;
        const int channels = alsa.channels;
        float buffer[frames * channels];

        while (alsa.running) {
            memset(buffer, 0, sizeof(buffer));

            // Call game audio mix callback
            size_t filled = alsa.renderCallback(buffer, frames);

            int err = snd_pcm_writei(alsa.handle, buffer, frames);
            if (err == -EPIPE) { snd_pcm_prepare(alsa.handle); } // Buffer underrun
            else if (err < 0) { fprintf(stderr, "ALSA write error: %s\n", snd_strerror(err)); }
        }
    });
    return true;
}
void Speaker::stop() {
    alsa.running = false;
    if (alsa.playbackThread.joinable())
        alsa.playbackThread.join();
}

void Speaker::setRenderCallback(RenderCallback cb) {
    alsa.renderCallback = std::move(cb);
}









static MicrophoneCallback g_microphoneCallback = nullptr;
static AudioStreamCallback g_streamCallback = nullptr;



int Moss_Init_Audio();

void Moss_Terminate_Audio();

void Moss_AudioUpdate(float deltaTime);


bool Moss_AudioRegisterDecoder(Moss_AudioEncodedFormat format, Moss_AudioDecodeCallback callback, void* userdata) {
    return Moss_AudioRegisterDecoderShared(format, callback, userdata);
}

Moss_AudioSource* Moss_AudioLoadWavFile(const char* filename) { return Moss_AudioLoadWaveSourceShared(filename); }

Moss_AudioSource* Moss_AudioLoadWav() { return nullptr; }

Moss_AudioSource* Moss_AudioLoadOgg(const char* filename, AudioLoadType type) { if (Moss_AudioSource* source = Moss_AudioDecodeWithRegisteredDecoderShared(Moss_AudioEncodedFormat::OGG, filename, type)) return source; return Moss_AudioLoadWaveSourceShared(filename); }

Moss_AudioSource* Moss_AudioLoadMP3(const char* filename) { if (Moss_AudioSource* source = Moss_AudioDecodeWithRegisteredDecoderShared(Moss_AudioEncodedFormat::MP3, filename, AudioLoadType::FULLY_LOADED)) return source; return Moss_AudioLoadWaveSourceShared(filename); }


void Moss_AudioSourceDestroy(Moss_AudioSource* source) {
    if (source && source->destroy) {
        source->destroy(source);
    }
}
Moss_AudioSource* Moss_AudioCaptureMicrophone(Microphone* mic);
Moss_AudioSource* Moss_AudioCaptureMossMicrophone(Moss_Microphone* mic);

// Effects

AudioEffect Moss_AudioCreateEffect(AudioEffectType type);

void Moss_AudioCreateEffect(AudioEffect* effect, const char* paramName, float value);

void Moss_AudioRemoveEffect(AudioEffect* effect);

// Channel

ChannelID Moss_AudioCreateChannel(ChannelID channel);
void Audio_RemoveChannel(ChannelID channel);

ChannelID Moss_AudioGetMasterChannel();

void Moss_AudioSetChannelVolume(ChannelID channel, float volume);

void Moss_AudioSetChannelMute(ChannelID channel, bool mute);

void Moss_AudioAddChannelEffect(ChannelID channel, AudioEffect* effect);

void Moss_AudioRemoveChannelEffect(ChannelID channel, AudioEffect* effect);

void Moss_AudioRemoveAllChannelEffects(ChannelID channel);


AudioStream* Moss_AudioStreamCreate();

void Moss_AudioStreamPlay(AudioStream* audiostream);

void Moss_AudioStreamStop(AudioStream* audiostream);

void Moss_AudioStreamSetVolume(AudioStream* audiostream, float volume);

void Moss_AudioStreamSetPitch(AudioStream* audiostream, float pitch);

void Moss_AudioStreamSetLoop(AudioStream* audiostream, bool loop);

void Moss_AudioStreamRemove(AudioStream* audiostream);


AudioStream2D* Moss_AudioStream2DCreate();

void Moss_AudioStream2DPlay(AudioStream2D* audiostream);

void Moss_AudioStream2DStop(AudioStream2D* audiostream);

void Moss_AudioStream2DSetVolume(AudioStream2D* audiostream, float volume);

void Moss_AudioStream2DSetPitch(AudioStream2D* audiostream, float pitch);

void Moss_AudioStream2DSetLoop(AudioStream2D* audiostream, bool loop);

void Moss_AudioStream3DSetPosition(AudioStream2D* audiostream);

void Moss_AudioStream3DSetVelocity(AudioStream2D* audiostream);

void Moss_AudioStream3DSetMaxDistance(AudioStream2D* audiostream);

void Moss_AudioStream2DRemove(AudioStream2D* audiostream);


AudioStream3D* Moss_AudioStream3DCreate();

void Moss_AudioStream3DPlay(AudioStream3D* audiostream);

void Moss_AudioStream3DStop(AudioStream3D* audiostream);

void Moss_AudioStream3DSetVolume(AudioStream3D* audiostream, float volume);

void Moss_AudioStream3DSetPitch(AudioStream3D* audiostream, float pitch);

void Moss_AudioStream3DSetLoop(AudioStream3D* audiostream, bool loop);

void Moss_AudioStream3DSetPosition(AudioStream3D* audiostream);

void Moss_AudioStream3DSetVelocity(AudioStream3D* audiostream);

void Moss_AudioStream3DSetMaxDistance(AudioStream3D* audiostream);

void Moss_AudioStream3DSetDistanceModel(AudioStream3D* stream, DistanceModel model);

void Moss_AudioStream3DRemove(AudioStream3D* audiostream);

// Listeners

AudioListener2D* Moss_AudioCreateAudioListener2D();

AudioListener3D* Moss_AudioCreateAudioListener3D();

RayAudioListener2D* Moss_AudioCreateRayAudioListener2D();

RayAudioListener3D* Moss_AudioCreateRayAudioListener3D();


void Moss_AudioRemoveAudioListener2D(AudioListener2D* listener);

void Moss_AudioRemoveAudioListener3D(AudioListener3D* listener);

void Moss_AudioRemoveRayAudioListener2D(RayAudioListener2D* listener);

void Moss_AudioRemoveRayAudioListener3D(RayAudioListener3D* listener);


void Moss_AudioActivateAudioListener2D(AudioListener2D* listener, bool activate);

void Moss_AudioActivateAudioListener3D(AudioListener3D* listener, bool activate);

void Moss_AudioActivateRayAudioListener2D(RayAudioListener2D* listener, bool activate);

void Moss_AudioActivateRayAudioListener3D(RayAudioListener3D* listener, bool activate);

void Moss_AudioListenerSetOrientation(AudioListener3D* listener, const Vec3& forward, const Vec3& up);

// Speakers

bool Moss_IsSpeakerDeviceReady();

void Moss_AudioSpeakerOpen();

void Moss_AudioSpeakerPause();

void Moss_AudioSpeakerResume();

bool Moss_AudioSpeakerIsPaused();
bool Moss_AudioSelectSpeakerDevice(int id);

int Moss_GetCurrentSpeakerDeviceID();

const char* Moss_GetSpeakerDeviceName(int id);

int Moss_ListSpeakerDevices();

// Microphone

bool Moss_IsMicrophoneDeviceReady();

int Moss_AudioMicrophoneOpen();

void Moss_AudioMicrophoneClose();

void Moss_AudioMicrophonePlay();

void Moss_AudioMicrophoneStop();

int Moss_AudioMicrophoneID();

bool Moss_AudioSelectMicrophoneDevice(int id);

const char* Moss_GetMicrophoneDeviceName(int index);

int Moss_ListMicrophoneDevices();


void Moss_AudioMicrophoneSetGain(Microphone* mic, float gain);

int Moss_AudioMicrophoneGetSampleRate(Microphone* mic);

int Moss_AudioMicrophoneGetChannels(Microphone* mic);


void Moss_AudioStreamSetCallback(AudioStream* stream, AudioStreamCallback callback, void* userData);

void Moss_AudioMicrophoneSetCallback(Microphone* mic, MicrophoneCallback callback, void* userData);
void Moss_AudioStreamSetVolume(AudioStream* audiostream, float volume) { if (audiostream) audiostream->setVolume(volume); }
void Moss_AudioStreamSetPitch(AudioStream* audiostream, float pitch) { if (audiostream) audiostream->setPitch(pitch); }
void Moss_AudioStreamSetPlaybackRate(AudioStream* audiostream, float rate) { if (audiostream) audiostream->setPlaybackRate(rate); }
void Moss_AudioStreamSetPan(AudioStream* audiostream, float pan) { if (audiostream) audiostream->setPan(pan); }
void Moss_AudioStreamSetLoop(AudioStream* audiostream, bool loop) { if (audiostream) audiostream->setLooping(loop); }

void Moss_AudioStream2DSetVolume(AudioStream2D* audiostream, float volume) { if (audiostream) audiostream->stream.setVolume(volume); }
void Moss_AudioStream2DSetPitch(AudioStream2D* audiostream, float pitch) { if (audiostream) audiostream->stream.setPitch(pitch); }
void Moss_AudioStream2DSetPlaybackRate(AudioStream2D* audiostream, float rate) { if (audiostream) audiostream->stream.setPlaybackRate(rate); }
void Moss_AudioStream2DSetPan(AudioStream2D* audiostream, float pan) { if (audiostream) { audiostream->pan = std::clamp(pan, -1.0f, 1.0f); audiostream->stream.setPan(audiostream->pan); } }
void Moss_AudioStream2DSetLoop(AudioStream2D* audiostream, bool loop) { if (audiostream) audiostream->stream.setLooping(loop); }

void Moss_AudioStream3DSetVolume(AudioStream3D* audiostream, float volume) { if (audiostream) audiostream->stream.setVolume(volume); }
void Moss_AudioStream3DSetPitch(AudioStream3D* audiostream, float pitch) { if (audiostream) audiostream->stream.setPitch(pitch); }
void Moss_AudioStream3DSetPlaybackRate(AudioStream3D* audiostream, float rate) { if (audiostream) audiostream->stream.setPlaybackRate(rate); }
void Moss_AudioStream3DSetPan(AudioStream3D* audiostream, float pan) { if (audiostream) { audiostream->pan = std::clamp(pan, -1.0f, 1.0f); audiostream->stream.setPan(audiostream->pan); } }
void Moss_AudioStream3DSetLoop(AudioStream3D* audiostream, bool loop) { if (audiostream) audiostream->stream.setLooping(loop); }
*/