// win32_audio.cpp

#include <Moss/Audio/xaudio/win32_audio.h>

#include <initguid.h>       // This must be first
#include <xaudio2.h>
#include <mmdeviceapi.h>
#include <audioclient.h>
#include <functiondiscoverykeys_devpkey.h>
#include <comdef.h>
#include <thread>
#include <atomic>
#include <vector>
#include <audiopolicy.h>
#include <string>
#include <math.h>
#include <mmreg.h> // For WAVE_FORMAT_*, WAVEFORMATEXTENSIBLE
#include <ksmedia.h> // For KSDATAFORMAT_SUBTYPE_*

#pragma comment(lib, "ole32.lib")
#pragma comment(lib, "mmdevapi.lib")
#pragma comment(lib, "uuid.lib")

std::vector<AudioStream*> g_activeStreams;

static AudioSpec_t g_outputSpec { AudioFormat::UNKNOWN, 0, 0 };

Float2* g_activeListener2D;
Float2* g_activeListenerVelocity2D;
Float3* g_activeListener3D;
Float3* g_activeListenerVelocity3D;

/*              Reminder
* IMMDeviceEnumerator: an interface in the Windows MMDevice API that provides methods for enumerating (listing) multimedia device resources, specifically audio endpoint devices
* IMMDevice: an interface that represents a multimedia device, specifically, an audio endpoint device in the current implementation
* IAudioClient: allows a client application to manage an audio stream, enabling it to interact with the audio engine for shared or exclusive mode streams
* IAudioRenderClient: enables a client application to write audio data to a rendering endpoint buffer, allowing it to output audio to a sound device
* WAVEFORMATEX: defines the format of waveform-audio data (Wav)
* IMMDeviceCollection: 	Represents a collection of audio devices.
* IMMEndpoint: Represents an audio endpoint device.
*/

void SetAudioListener2D(Float2* position) { g_activeListener2D = position; }
void SetAudioListener3D(Float3* position) { g_activeListener3D = position; }
void SetAudioListenerVelocity3D(Float3* velocity) { g_activeListenerVelocity3D = velocity; }

// Helpers
static AudioSpec_t WaveFormatToSpec(const WAVEFORMATEX *fmt) {
    AudioSpec_t spec {};
    spec.frequency = fmt->nSamplesPerSec;
    spec.channels  = fmt->nChannels;

    switch (fmt->wFormatTag) {
        case WAVE_FORMAT_PCM:
            switch (fmt->wBitsPerSample) {
                case 8:
                    spec.format = AudioFormat::U8;
                    break;
                case 16:
                    spec.format = AudioFormat::S16LE;
                    break;
                case 24:
                    spec.format = AudioFormat::S24LE;
                    break;
                case 32:
                    spec.format = AudioFormat::S32LE;
                    break;
                default:
                    spec.format = AudioFormat::UNKNOWN;
                    break;
            }
            break;

        case WAVE_FORMAT_IEEE_FLOAT:
            switch (fmt->wBitsPerSample) {
                case 32:
                    spec.format = AudioFormat::F32LE;
                    break;
                case 64:
                    spec.format = AudioFormat::F64LE;
                    break;
                default:
                    spec.format = AudioFormat::UNKNOWN;
                    break;
            }
            break;

        case WAVE_FORMAT_EXTENSIBLE: {
            const WAVEFORMATEXTENSIBLE* ext = reinterpret_cast<const WAVEFORMATEXTENSIBLE*>(fmt);
            const GUID& subFormat = ext->SubFormat;

            if (subFormat == KSDATAFORMAT_SUBTYPE_PCM) {
                switch (fmt->wBitsPerSample) {
                    case 16: spec.format = AudioFormat::S16LE; break;
                    case 24: spec.format = AudioFormat::S24LE; break;
                    case 32: spec.format = AudioFormat::S32LE; break;
                    default: spec.format = AudioFormat::UNKNOWN; break;
                }
            }
            else if (subFormat == KSDATAFORMAT_SUBTYPE_IEEE_FLOAT)
            {
                switch (fmt->wBitsPerSample) {
                    case 32: spec.format = AudioFormat::F32LE; break;
                    case 64: spec.format = AudioFormat::F64LE; break;
                    default: spec.format = AudioFormat::UNKNOWN; break;
                }
            }
            else { spec.format = AudioFormat::UNKNOWN; }
            break;
        }

        default:
            spec.format = AudioFormat::UNKNOWN;
            break;
    }

    return spec;
}


// -----------------------------------------------------------------------------
// Pretty‑printer for AudioFormat
// -----------------------------------------------------------------------------
static const char* AudioFormatName(AudioFormat format)
{
    switch (format)
    {
        case AudioFormat::U8:     return "U8";
        case AudioFormat::S8:     return "S8";
        case AudioFormat::S16LE:  return "S16LE";
        case AudioFormat::S16BE:  return "S16BE";
        case AudioFormat::S24LE: return "S24LE";
        case AudioFormat::S32LE:  return "S32LE";
        case AudioFormat::S32BE:  return "S32BE";
        case AudioFormat::F32LE:  return "F32LE";
        case AudioFormat::F32BE:  return "F32BE";
        case AudioFormat::F64BE:  return "F64BE";
        case AudioFormat::F64LE:  return "F64LE";
        default:                  return "UNKNOWN";
    }
}


bool Moss_Init_Audio() {
    HRESULT hr;

    // 1. Initialize COM
    hr = CoInitializeEx(nullptr, COINIT_MULTITHREADED);
    if (FAILED(hr)) return false;

    // 2. Create XAudio2 engine
    hr = XAudio2Create(&g_xaudio2, 0, XAUDIO2_DEFAULT_PROCESSOR);
    if (FAILED(hr)) {
        CoUninitialize();
        return false;
    }

    // 3. Create MMDeviceEnumerator
    hr = CoCreateInstance(__uuidof(MMDeviceEnumerator), nullptr, CLSCTX_ALL, IID_PPV_ARGS(&g_deviceEnumerator));
    if (FAILED(hr)) {
        g_xaudio2->Release();
        g_xaudio2 = nullptr;
        CoUninitialize();
        return false;
    }

    // 4. Get default **render** endpoint (for speakers/headset layout!)
    IMMDevice *renderDevice = nullptr;
    hr = g_deviceEnumerator->GetDefaultAudioEndpoint(eRender, eConsole, &renderDevice);
    if (FAILED(hr)) {
        g_deviceEnumerator->Release();
        g_deviceEnumerator = nullptr;
        g_xaudio2->Release();
        g_xaudio2 = nullptr;
        CoUninitialize();
        return false;
    }

    // 5. Activate IAudioClient for the render endpoint
    IAudioClient *renderClient = nullptr;
    hr = renderDevice->Activate(__uuidof(IAudioClient), CLSCTX_ALL, nullptr,
                                (void**)&renderClient);
    if (FAILED(hr)) {
        renderDevice->Release();
        g_deviceEnumerator->Release();
        g_deviceEnumerator = nullptr;
        g_xaudio2->Release();
        g_xaudio2 = nullptr;
        CoUninitialize();
        return false;
    }

    // 6. Get the mix format
    WAVEFORMATEX *pwfx = nullptr;
    hr = renderClient->GetMixFormat(&pwfx);
    if (FAILED(hr)) {
        renderClient->Release();
        renderDevice->Release();
        g_deviceEnumerator->Release();
        g_deviceEnumerator = nullptr;
        g_xaudio2->Release();
        g_xaudio2 = nullptr;
        CoUninitialize();
        return false;
    }

    // 7. Fill output spec from render format
    g_outputSpec = WaveFormatToSpec(pwfx);
    MOSS_INFO("Render format: %d Hz, %d channels, %s", g_outputSpec.frequency, g_outputSpec.channels, AudioFormatName(g_outputSpec.format));

    CoTaskMemFree(pwfx);
    renderClient->Release();
    renderDevice->Release();

    // 8. Create mastering voice using the default device layout
    hr = g_xaudio2->CreateMasteringVoice(&g_masteringVoice);
    if (FAILED(hr)) {
        g_deviceEnumerator->Release();
        g_deviceEnumerator = nullptr;
        g_xaudio2->Release();
        g_xaudio2 = nullptr;
        CoUninitialize();
        return false;
    }

    // 9. Get default **microphone** device (for voice input)
    hr = g_deviceEnumerator->GetDefaultAudioEndpoint(eCapture, eCommunications, &g_microphoneDevice);
    if (FAILED(hr)) {
        g_masteringVoice->DestroyVoice();
        g_masteringVoice = nullptr;
        g_deviceEnumerator->Release();
        g_deviceEnumerator = nullptr;
        g_xaudio2->Release();
        g_xaudio2 = nullptr;
        CoUninitialize();
        return false;
    }

    // 10. Activate IAudioClient for capture
    hr = g_microphoneDevice->Activate(IID_IAudioClient, CLSCTX_ALL, nullptr, (void**)&g_audioClient);
    if (FAILED(hr)) {
        g_microphoneDevice->Release();
        g_microphoneDevice = nullptr;
        g_deviceEnumerator->Release();
        g_deviceEnumerator = nullptr;
        g_masteringVoice->DestroyVoice();
        g_masteringVoice = nullptr;
        g_xaudio2->Release();
        g_xaudio2 = nullptr;
        CoUninitialize();
        return false;
    }

    // 11. Get microphone mix format (not used in g_outputSpec!)
    WAVEFORMATEX *capFormat = nullptr;
    hr = g_audioClient->GetMixFormat(&capFormat);
    if (FAILED(hr)) {
        g_audioClient->Release();
        g_audioClient = nullptr;
        g_microphoneDevice->Release();
        g_microphoneDevice = nullptr;
        g_deviceEnumerator->Release();
        g_deviceEnumerator = nullptr;
        g_masteringVoice->DestroyVoice();
        g_masteringVoice = nullptr;
        g_xaudio2->Release();
        g_xaudio2 = nullptr;
        CoUninitialize();
        return false;
    }
    constexpr REFERENCE_TIME CAPTURE_BUFFER_TIME = 10 * 1000 * 1000; // 1 second
    hr = g_audioClient->Initialize(AUDCLNT_SHAREMODE_SHARED, 0, CAPTURE_BUFFER_TIME, 0, capFormat, nullptr);
    CoTaskMemFree(capFormat);
    if (FAILED(hr)) {
        g_audioClient->Release();
        g_audioClient = nullptr;
        g_microphoneDevice->Release();
        g_microphoneDevice = nullptr;
        g_deviceEnumerator->Release();
        g_deviceEnumerator = nullptr;
        g_masteringVoice->DestroyVoice();
        g_masteringVoice = nullptr;
        g_xaudio2->Release();
        g_xaudio2 = nullptr;
        CoUninitialize();
        return false;
    }

    // 12. Get capture client
    hr = g_audioClient->GetService(IID_IAudioCaptureClient, (void**)&g_captureClient);
    if (FAILED(hr)) {
        g_audioClient->Release();
        g_audioClient = nullptr;
        g_microphoneDevice->Release();
        g_microphoneDevice = nullptr;
        g_deviceEnumerator->Release();
        g_deviceEnumerator = nullptr;
        g_masteringVoice->DestroyVoice();
        g_masteringVoice = nullptr;
        g_xaudio2->Release();
        g_xaudio2 = nullptr;
        CoUninitialize();
        return false;
    }

    // 13. Start capturing
    hr = g_audioClient->Start();
    if (FAILED(hr)) {
        g_captureClient->Release();
        g_captureClient = nullptr;
        g_audioClient->Release();
        g_audioClient = nullptr;
        g_microphoneDevice->Release();
        g_microphoneDevice = nullptr;
        g_deviceEnumerator->Release();
        g_deviceEnumerator = nullptr;
        g_masteringVoice->DestroyVoice();
        g_masteringVoice = nullptr;
        g_xaudio2->Release();
        g_xaudio2 = nullptr;
        CoUninitialize();
        return false;
    }

    return true;
}

void Moss_Terminate_Audio() {
    if (g_audioClient) { g_audioClient->Stop(); }
    if (g_captureClient) { g_captureClient->Release(); g_captureClient = nullptr; }
    if (g_audioClient) { g_audioClient->Release(); g_audioClient = nullptr; }
    if (g_microphoneDevice) { g_microphoneDevice->Release(); g_microphoneDevice = nullptr; }
    if (g_deviceEnumerator) { g_deviceEnumerator->Release(); g_deviceEnumerator = nullptr; }
    if (g_masteringVoice) { g_masteringVoice->DestroyVoice(); g_masteringVoice = nullptr; }
    if (g_xaudio2) { g_xaudio2->Release(); g_xaudio2 = nullptr; }
    CoUninitialize();
}


// ====================================================================================
AudioStream::AudioStream(Wav& WavFile, float volume, float pitch, bool loop) : wav(WavFile), volume(volume), pitch(pitch), loop(loop)  {
    WAVEFORMATEX wfx = {};
    wfx.wFormatTag = wav.audioFormat;        // e.g., WAVE_FORMAT_PCM = 1
    wfx.nChannels = wav.numChannels;         // number of audio channels
    wfx.nSamplesPerSec = wav.sampleRate;     // samples per second (Hz)
    wfx.nAvgBytesPerSec = wav.byteRate;      // byte rate = SampleRate * NumChannels * BitsPerSample/8
    wfx.nBlockAlign = wav.blockAlign;        // NumChannels * BitsPerSample/8
    wfx.wBitsPerSample = wav.bitsPerSample;  // bits per sample
    wfx.cbSize = 0;                          // usually 0 for PCM
    HRESULT hr = g_xaudio2->CreateSourceVoice(&sourceVoice, &wfx, 0, XAUDIO2_DEFAULT_FREQ_RATIO, nullptr, nullptr, nullptr);
    if (FAILED(hr)) {
        MOSS_ERROR("CreateSourceVoice failed: 0x%08X", hr);
        sourceVoice = nullptr;
        return;
    }


    XAUDIO2_BUFFER buffer = {};
    buffer.AudioBytes = wav.dataChunkSize;
    buffer.pAudioData = reinterpret_cast<BYTE*>(wav.dataBegin);
    buffer.Flags = XAUDIO2_END_OF_STREAM;

    sourceVoice->SubmitSourceBuffer(&buffer);
}

AudioStream::~AudioStream() { if (sourceVoice) { sourceVoice->DestroyVoice(); sourceVoice = nullptr;} }


//HRESULT SetChannelVolumes( UINT32 Channels, const float* pVolumes, UINT32 OperationSet = XAUDIO2_COMMIT_NOW);

void AudioStream::setPan(float pan) {
    if (!sourceVoice) return;

    // Clamp pan to range [-1.0f, 1.0f]
    pan = max(-1.0f, min(1.0f, pan));

    // Calculate stereo volumes
    float leftVolume = 1.0f;
    float rightVolume = 1.0f;

    if (pan < 0.0f) { { rightVolume = 1.0f + pan; }}        // pan = -1.0f → rightVolume = 0.0f
    else if (pan > 0.0f) { { leftVolume = 1.0f - pan; }}    // pan = 1.0f → leftVolume = 0.0f

    float volumes[2] = { leftVolume, rightVolume };
    HRESULT hr = sourceVoice->SetChannelVolumes(2, volumes); // 2 = number of output channels

    if (FAILED(hr)) { MOSS_ERROR("Failed to set channel volumes."); return; }
}

void AudioStream::setVolume(float volume) {
    this->volume = volume;
    if (sourceVoice) {
        float volumes[MOSS_MAXCHANNELS] = {};
        for (int i = 0; i < MOSS_MAXCHANNELS; ++i) { volumes[i] = volume; }
        sourceVoice->SetVolume(volume);
    }
}

void AudioStream::setPitch(float pitch) {
    this->pitch = pitch;
    if (sourceVoice) { sourceVoice->SetFrequencyRatio(pitch); }
}

void AudioStream::setLooping(bool loop) { this->loop = loop; }

// Used for 2D and 3D pan
void BuildPanMatrix(UINT32 srcCh, UINT32 dstCh, float pan, float volume, std::vector<float>& outMatrix) {
    std::vector<float> gains = AudioPan(dstCh, pan);
    outMatrix.resize(srcCh * dstCh, 0.f);

    for (UINT32 s = 0; s < srcCh; ++s) {
        for (UINT32 d = 0; d < dstCh; ++d) {
            outMatrix[s * dstCh + d] = gains[d] * volume;
        }
    }
}


// AudioStream3D implementation
// ---------------------------------------------------------------------------
// helpers -------------------------------------------------------------------
static bool ClampAndLogHR(HRESULT hr, const char* call) {
    if (FAILED(hr))
    {
        MOSS_ERROR("%s failed 0x%08X", call, hr);
        return false;
    }
    return true;
}

// Build a srcCh × dstCh matrix that applies volume and pan (constant‑power).
static void CalcStereoMatrix3D(UINT32 srcCh, UINT32 dstCh, float pan, float vol, std::vector<float>& mtx) {
    mtx.assign(srcCh * dstCh, 0.0f);

    float l = (1.0f - pan) * 0.5f * vol;   // pan −1 → l=1,r=0
    float r = (1.0f + pan) * 0.5f * vol;   // pan +1 → l=0,r=1

    for (UINT32 s = 0; s < srcCh; ++s)
    {
        mtx[s * dstCh + 0] = l;
        if (dstCh > 1) mtx[s * dstCh + 1] = r;
    }
}
// ---------------------------------------------------------------------------
/*
e.g.
AudioResult r = Compute2DAudioWithReflections(source, listener, walls);

// XAudio2 stereo matrix: source L/R -> output L/R
float matrix[2][2];

// Constant-power mapping
matrix[0][0] = r.left;   // source L -> output L
matrix[0][1] = r.right;  // source L -> output R
matrix[1][0] = r.left;   // source R -> output L
matrix[1][1] = r.right;  // source R -> output R

srcVoice->SetOutputMatrix(masteringVoice, 2, 2, &matrix[0][0]);


// Keeps the audio smooth avoid clicks
static float prevLeft = 0.0f, prevRight = 0.0f;
float smoothL = prevLeft * 0.9f + r.left * 0.1f;
float smoothR = prevRight * 0.9f + r.right * 0.1f;

matrix[0][0] = smoothL;
matrix[0][1] = smoothR;
matrix[1][0] = smoothL;
matrix[1][1] = smoothR;

prevLeft = smoothL;
prevRight = smoothR;
*/

/*
float leftBuffer[1024];
float rightBuffer[1024];
float leftGain, rightGain;

ProcessSoundDirect(
    physics,
    source,
    listener,
    srcVel,
    listenerVel,
    monoInput,
    1024,
    leftBuffer,
    rightBuffer,
    leftGain,
    rightGain
);

// Submit matrix directly
float matrix[2] = { leftGain, rightGain };
voice->SetOutputMatrix(masteringVoice, 2, 2, matrix);
*/
// ====================================================================================

static MicrophoneCallback g_microphoneCallback = nullptr;
static AudioStreamCallback g_streamCallback = nullptr;

void Moss_AudioUpdate(float) {
    for (auto* s : g_activeStreams) {
        if (!s || !s->sourceVoice) continue;

        XAUDIO2_VOICE_STATE state{};
        s->sourceVoice->GetState(&state);

        if (s->playing && state.BuffersQueued == 0 && !s->loop) {
            s->playing = false;
        }
    }
}


Moss_AudioSource* Moss_AudioLoadWav() { }

Moss_AudioSource* Moss_AudioLoadOgg(const char* filename, AudioLoadType type) { }

Moss_AudioSource* Moss_AudioLoadMP3(const char* filename) { }

Moss_AudioSource* Moss_AudioCaptureMicrophone(Microphone* mic) { }

// Effects

AudioEffect Moss_AudioCreateEffect(AudioEffectType type) { }

void Moss_AudioCreateEffect(AudioEffect* effect, const char* paramName, float value) { }

void Moss_AudioRemoveEffect(AudioEffect* effect) { }

// Channel

ChannelID Moss_AudioCreateChannel(ChannelID channel) { }
void Audio_RemoveChannel(ChannelID channel) { }

ChannelID Moss_AudioGetMasterChannel() { }

void Moss_AudioSetChannelVolume(ChannelID channel, float volume) { }

void Moss_AudioSetChannelMute(ChannelID channel, bool mute) { }

void Moss_AudioAddChannelEffect(ChannelID channel, AudioEffect* effect) { }

void Moss_AudioRemoveChannelEffect(ChannelID channel, AudioEffect* effect) { }

void Moss_AudioRemoveAllChannelEffects(ChannelID channel) { }


AudioStream* Moss_AudioStreamCreate() {
    auto* stream = new AudioStream{};
    g_activeStreams.push_back(stream);
    return stream;
}


void Moss_AudioStreamPlay(AudioStream* audiostream) {
    if (!s || !audiostream->sourceVoice || audiostream->playing) return;
    audiostream->sourceVoice->SetVolume(audiostream->volume);
    audiostream->sourceVoice->SetFrequencyRatio(audiostream->pitch);
    audiostream->sourceVoice->Start();
    audiostream->playing = true;
}

void Moss_AudioStreamStop(AudioStream* audiostream) {
    if (!audiostream || !audiostream->sourceVoice) return;

    audiostream->sourceVoice->Stop();
    audiostream->sourceVoice->FlushSourceBuffers();
    audiostream->playing = false;

    if (audiostream->wav) {
        XAUDIO2_BUFFER buf{};
        buf.AudioBytes = s->wav->dataChunkSize;
        buf.pAudioData = (BYTE*)s->wav->dataBegin;
        buf.Flags = XAUDIO2_END_OF_STREAM;
        audiostream->sourceVoice->SubmitSourceBuffer(&buf);
    }
}


void Moss_AudioStreamSetVolume(AudioStream* audiostream, float volume) { 
    if (!s) return;
    audiostream->volume = volume;
    if (audiostream->sourceVoice) audiostream->sourceVoice->SetVolume(volume);
}

void Moss_AudioStreamSetPitch(AudioStream* audiostream, float pitch) {
    if (!s) return;
    audiostream->pitch = pitch;
    if (s->sourceVoice) s->sourceVoice->SetFrequencyRatio(pitch);
 }

void Moss_AudioStreamSetLoop(AudioStream* audiostream, bool loop) { if (audiostream) audiostream->loop = loop; }

void Moss_AudioStreamRemove(AudioStream* audiostream) {
    if (!audiostream) return;

    Moss_AudioStreamStop(audiostream);

    if (audiostream->sourceVoice) {
        audiostream->sourceVoice->DestroyVoice();
        audiostream->sourceVoice = nullptr;
    }

    g_activeStreams.erase(std::remove(g_activeStreams.begin(), g_activeStreams.end(), s),g_activeStreams.end());

    delete audiostream;
}


AudioStream2D* Moss_AudioStream2DCreate() { }

void Moss_AudioStream2DPlay(AudioStream2D* audiostream) { 
    if (!g_activeListener2D || !audiostream->stream.sourceVoice) return;
    Float2 diff;
    diff.x = position.x - (*g_activeListener2D).x;
    diff.y = position.y - (*g_activeListener2D).y;
    float  distance = sqrtf(diff.x*diff.x + diff.y*diff.y);
    float  volume   = std::clamp(1.0f - distance / maxDistance, 0.f, 1.f);
    float  pan      = std::clamp(diff.x / maxDistance, -1.f, 1.f);

    UINT32 srcCh = audiostream->stream.wav.numChannels;
    UINT32 dstCh = g_outputSpec.channels;

    std::vector<float> matrix;
    BuildPanMatrix(srcCh, dstCh, pan, volume, matrix);

    HRESULT hr = audiostream->stream.sourceVoice->SetOutputMatrix( g_masteringVoice, srcCh, dstCh, matrix.data());
    if (FAILED(hr)) { MOSS_ERROR("SetOutputMatrix failed %08X", hr); return; }

    hr = audiostream->stream.sourceVoice->SetVolume(1.0f);
    if (FAILED(hr)) { MOSS_ERROR("SetVolume failed %08X", hr); return; }
}

void Moss_AudioStream2DStop(AudioStream2D* audiostream) { Moss_AudioStreamStop(audiostream->stream) }


void Moss_AudioStream2DSetVolume(AudioStream2D* audiostream, float volume) { }

void Moss_AudioStream2DSetPitch(AudioStream2D* audiostream, float pitch) { }

void Moss_AudioStream2DSetLoop(AudioStream2D* audiostream, bool loop) { }

void Moss_AudioStream3DSetPosition(AudioStream2D* audiostream) { }

void Moss_AudioStream3DSetVelocity(AudioStream2D* audiostream) { }

void Moss_AudioStream3DSetMaxDistance(AudioStream2D* audiostream) { }

void Moss_AudioStream2DRemove(AudioStream2D* audiostream) { }


AudioStream3D* Moss_AudioStream3DCreate() { }

void Moss_AudioStream3DPlay(AudioStream3D* audiostream) {
    if (!g_activeListener2D || !audiostream->stream.sourceVoice) return;

    // Positioning
    Float2 diff;
    diff.x = position.x - (*g_activeListener2D).x;
    diff.y = position.y - (*g_activeListener2D).y;
    float  distance = sqrtf(diff.x * diff.x + diff.y * diff.y);
    float  volume = std::clamp(1.0f - distance / maxDistance, 0.f, 1.f);
    float  pan = std::clamp(diff.x / maxDistance, -1.f, 1.f);

    // --- Doppler pitch shift ---
    Vec3 listenerPos = Vec3(g_activeListener2D->x, 0.f, g_activeListener2D->y);
    Vec3 listenerVel = Vec3::sZero(); // or get from system
    Vec3 sourcePos   = Vec3(position.x, 0.f, position.y);
    Vec3 sourceVel   = Vec3(g_activeListenerVelocity2D->x, 0.f, g_activeListenerVelocity2D->y);

    float doppler = DopplerPitch(listenerPos, listenerVel, sourcePos, sourceVel);

    // --- Set Doppler pitch ratio ---
    HRESULT hr = audiostream->stream.sourceVoice->SetFrequencyRatio(doppler);
    if (FAILED(hr)) { MOSS_ERROR("SetFrequencyRatio failed %08X", hr); return; }

    // --- Set panning matrix ---
    UINT32 srcCh = audiostream->stream.wav.numChannels;
    UINT32 dstCh = g_outputSpec.channels;

    std::vector<float> matrix;
    BuildPanMatrix(srcCh, dstCh, pan, volume, matrix); // still needed
    hr = audiostream->stream.sourceVoice->SetOutputMatrix( g_masteringVoice, srcCh, dstCh, matrix.data());
    if (FAILED(hr)) { MOSS_ERROR("SetOutputMatrix failed %08X", hr); return; }

    // Set final gain
    hr = audiostream->stream.sourceVoice->SetVolume(1.0f); // global volume here
    if (FAILED(hr)) { MOSS_ERROR("SetVolume failed %08X", hr); return; }

    audiostream->stream.play();
}

void Moss_AudioStream3DStop(AudioStream3D* audiostream) { audiostream->stream.stop() }

void Moss_AudioStream3DSetVolume(AudioStream3D* audiostream, float volume) { audiostream->stream. }

void Moss_AudioStream3DSetPitch(AudioStream3D* audiostream, float pitch) { }

void Moss_AudioStream3DSetLoop(AudioStream3D* audiostream, bool loop) { }

void Moss_AudioStream3DSetPosition(AudioStream3D* audiostream) { }

void Moss_AudioStream3DSetVelocity(AudioStream3D* audiostream) { }

void Moss_AudioStream3DSetMaxDistance(AudioStream3D* audiostream) { }

void Moss_AudioStream3DSetDistanceModel(AudioStream3D* stream, DistanceModel model) { }

void Moss_AudioStream3DRemove(AudioStream3D* audiostream) { }

// Listeners

AudioListener2D* Moss_AudioCreateAudioListener2D() { }

AudioListener3D* Moss_AudioCreateAudioListener3D() { }

RayAudioListener2D* Moss_AudioCreateRayAudioListener2D() { }

RayAudioListener3D* Moss_AudioCreateRayAudioListener3D() { }


void Moss_AudioRemoveAudioListener2D(AudioListener2D* listener) { }

void Moss_AudioRemoveAudioListener3D(AudioListener3D* listener) { }

void Moss_AudioRemoveRayAudioListener2D(RayAudioListener2D* listener) { }

void Moss_AudioRemoveRayAudioListener3D(RayAudioListener3D* listener) { }


void Moss_AudioActivateAudioListener2D(AudioListener2D* listener, bool activate) { }

void Moss_AudioActivateAudioListener3D(AudioListener3D* listener, bool activate) { }

void Moss_AudioActivateRayAudioListener2D(RayAudioListener2D* listener, bool activate) { }

void Moss_AudioActivateRayAudioListener3D(RayAudioListener3D* listener, bool activate) { }

void Moss_AudioListenerSetOrientation(AudioListener3D* listener, const Vec3& forward, const Vec3& up) { }


void Moss_AudioStreamSetCallback(AudioStream* stream, AudioStreamCallback callback, void* userData) { }
