
// alsa_audio.cpp
#include "alsa_audio.h"
#include "../audio_intern.h"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <fstream>
#include <iostream>
#include <atomic>
#include <thread>
#include <vector>

#include <alsa/asoundlib.h>

namespace Moss {
namespace Audio {

struct ALSAPlayback {
    snd_pcm_t* handle = nullptr;
    snd_pcm_hw_params_t* hwParams = nullptr;
    unsigned int sampleRate = 44100;
    int channels = 2;
    snd_pcm_format_t format = SND_PCM_FORMAT_FLOAT_LE;
    Speaker::RenderCallback renderCallback;
    std::atomic<bool> running{false};
    std::thread playbackThread;
};
static ALSAPlayback alsa;

AudioListener2D g_listener2D;
AudioListener3D g_listener3D;

int Moss_Init_Audio() {
    int err;

    if ((err = snd_pcm_open(&alsa.handle, "default", SND_PCM_STREAM_PLAYBACK, 0)) < 0) { 
        fprintf(stderr, "ALSA: Failed to open device: %s\n", snd_strerror(err));  
        return -1; 
    }

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
    
    file.read(reinterpret_cast<char*>(&wav->riffChunkId), 4);
    file.read(reinterpret_cast<char*>(&wav->riffChunkSize), 4);
    file.read(reinterpret_cast<char*>(&wav->format), 4);

    if (wav->riffChunkId != 0x46464952 || wav->format != 0x45564157) {
        delete wav;
        return nullptr;
    }

    file.read(reinterpret_cast<char*>(&wav->formatChunkId), 4);
    file.read(reinterpret_cast<char*>(&wav->formatChunkSize), 4);
    file.read(reinterpret_cast<char*>(&wav->audioFormat), 2);
    file.read(reinterpret_cast<char*>(&wav->numChannels), 2);
    file.read(reinterpret_cast<char*>(&wav->sampleRate), 4);
    file.read(reinterpret_cast<char*>(&wav->byteRate), 4);
    file.read(reinterpret_cast<char*>(&wav->blockAlign), 2);
    file.read(reinterpret_cast<char*>(&wav->bitsPerSample), 2);

    if (wav->formatChunkSize > 16)
        file.ignore(wav->formatChunkSize - 16);

    while (true) {
        file.read(reinterpret_cast<char*>(&wav->dataChunkId), 4);
        file.read(reinterpret_cast<char*>(&wav->dataChunkSize), 4);

        if (std::memcmp(wav->dataChunkId, "data", 4) == 0)
            break;

        file.ignore(wav->dataChunkSize);
    }

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

void AudioStream::setVolume(float value) { volume = std::clamp(value, 0.0f, 1.0f); }
void AudioStream::setPitch(float value) { pitch = std::clamp(value, 0.25f, 4.0f); }
void AudioStream::setPlaybackRate(float value) { playbackRate = std::clamp(value, 0.25f, 4.0f); setPitch(playbackRate); }
void AudioStream::setPan(float value) { pan = std::clamp(value, -1.0f, 1.0f); }
void AudioStream::setLooping(bool value) { loop = value; }
AudioStream::~AudioStream() { stop(); }

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
        snd_pcm_drop((snd_pcm_t*)pcm);
    }
    if (playbackThread.joinable()) {
        playbackThread.join();
    }
    playing = false;
}

void AudioStream2D::play() {
    AudioListener2D* listener = &g_listener2D;
    const float distance = (listener->position - position).Length();
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
    const float distance = delta.Length();
    const float distFactor = std::max(0.0f, 1.0f - (distance / std::max(maxDistance, 0.001f)));
    const float computedPan = std::clamp(delta.x / std::max(maxDistance, 0.001f), -1.0f, 1.0f);

    stream.setVolume(distFactor);
    stream.setPan(computedPan);
    stream.play();
}
void AudioStream3D::stop() { stream.stop(); }

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

            size_t filled = alsa.renderCallback(buffer, frames);

            int err = snd_pcm_writei(alsa.handle, buffer, frames);
            if (err == -EPIPE) { snd_pcm_prepare(alsa.handle); } 
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

} // namespace Audio
} // namespace Moss