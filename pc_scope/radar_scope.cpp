/**
 * radar_scope.cpp — Oscilloscope temps-réel + Analyseur spectral pour Radar Doppler
 * ===================================================================================
 * Application Windows native en C++ avec :
 *   - Lecture série Win32 API (protocole binaire STM32)
 *   - FFT haute résolution via KissFFT (domaine public)
 *   - Interface graphique GPU-accélérée Dear ImGui + OpenGL3 + GLFW
 *
 * 4 panneaux d'affichage :
 *   1. Oscilloscope (forme d'onde temporelle) avec trigger
 *   2. Spectre FFT (magnitude en dB)
 *   3. Spectrogramme (waterfall / cascade)
 *   4. Affichage numérique vitesse / fréquence / SNR
 *
 * Compilation (MSVC) :
 *   Voir CMakeLists.txt ou le script build.bat fourni.
 *
 * Compilation rapide sans CMake (si libs installées) :
 *   cl /O2 /EHsc radar_scope.cpp /I imgui /I glfw/include
 *      imgui/*.cpp imgui/backends/imgui_impl_glfw.cpp imgui/backends/imgui_impl_opengl3.cpp
 *      glfw3.lib opengl32.lib /Fe:radar_scope.exe
 *
 * Usage :
 *   radar_scope.exe              (COM5 par défaut)
 *   radar_scope.exe COM3         (port spécifique)
 */

// ═══════════════════════════════════════════════════════════════════════════════
// Fichier unique : on inclut tout ici pour faciliter la compilation.
// En production, on séparerait en modules (serial.h, fft.h, gui.h, etc.)
// ═══════════════════════════════════════════════════════════════════════════════

#define _CRT_SECURE_NO_WARNINGS
#define _USE_MATH_DEFINES

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cmath>
#include <cstdint>
#include <vector>
#include <string>
#include <algorithm>
#include <mutex>
#include <thread>
#include <atomic>
#include <chrono>

// ── Windows serial API ──
#define WIN32_LEAN_AND_MEAN
#include <windows.h>

// ── OpenGL + GLFW + Dear ImGui ──
// Ces headers sont fournis par les bibliothèques externes.
// Voir CMakeLists.txt pour le téléchargement automatique.
#include "imgui.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl3.h"
#include <GLFW/glfw3.h>

// ═══════════════════════════════════════════════════════════════════════════════
// KissFFT — Embedded (domaine public, Unlicense)
// ═══════════════════════════════════════════════════════════════════════════════
// On embarque KissFFT directement dans ce fichier pour zéro dépendance externe.
// KissFFT est une bibliothèque FFT légère et portable en C pur.
// Source : https://github.com/mborgerding/kissfft

#define KISS_FFT_IMPLEMENTATION
// ── Début de l'implémentation KissFFT inline ──
// (Normalement on inclurait kiss_fft.h, mais on l'embarque pour simplicité)

namespace kissfft {

struct cpx { float r, i; };

struct kiss_fft_state {
    int nfft;
    int inverse;
    std::vector<cpx> twiddles;
    std::vector<int> factors;
};

static void kf_bfly2(cpx *Fout, int fstride, const kiss_fft_state &st, int m) {
    cpx *Fout2 = Fout + m;
    const cpx *tw = st.twiddles.data();
    for (int i = 0; i < m; i++) {
        cpx t;
        t.r = Fout2[i].r * tw[i * fstride].r - Fout2[i].i * tw[i * fstride].i;
        t.i = Fout2[i].r * tw[i * fstride].i + Fout2[i].i * tw[i * fstride].r;
        Fout2[i].r = Fout[i].r - t.r;
        Fout2[i].i = Fout[i].i - t.i;
        Fout[i].r += t.r;
        Fout[i].i += t.i;
    }
}

static void kf_bfly4(cpx *Fout, int fstride, const kiss_fft_state &st, int m) {
    const cpx *tw1 = st.twiddles.data();
    const cpx *tw2 = tw1;
    const cpx *tw3 = tw1;
    cpx scratch[6];
    int k = m;
    int m2 = 2 * m, m3 = 3 * m;
    for (int i = 0; i < k; i++) {
        scratch[0].r = Fout[i + m].r * tw1[i * fstride].r - Fout[i + m].i * tw1[i * fstride].i;
        scratch[0].i = Fout[i + m].r * tw1[i * fstride].i + Fout[i + m].i * tw1[i * fstride].r;
        scratch[1].r = Fout[i + m2].r * tw2[i * fstride * 2].r - Fout[i + m2].i * tw2[i * fstride * 2].i;
        scratch[1].i = Fout[i + m2].r * tw2[i * fstride * 2].i + Fout[i + m2].i * tw2[i * fstride * 2].r;
        scratch[2].r = Fout[i + m3].r * tw3[i * fstride * 3].r - Fout[i + m3].i * tw3[i * fstride * 3].i;
        scratch[2].i = Fout[i + m3].r * tw3[i * fstride * 3].i + Fout[i + m3].i * tw3[i * fstride * 3].r;

        scratch[5].r = Fout[i].r - scratch[1].r;
        scratch[5].i = Fout[i].i - scratch[1].i;
        Fout[i].r += scratch[1].r;
        Fout[i].i += scratch[1].i;
        scratch[3].r = scratch[0].r + scratch[2].r;
        scratch[3].i = scratch[0].i + scratch[2].i;
        scratch[4].r = scratch[0].r - scratch[2].r;
        scratch[4].i = scratch[0].i - scratch[2].i;
        Fout[i + m2].r = Fout[i].r - scratch[3].r;
        Fout[i + m2].i = Fout[i].i - scratch[3].i;
        Fout[i].r += scratch[3].r;
        Fout[i].i += scratch[3].i;
        if (st.inverse) {
            Fout[i + m].r = scratch[5].r - scratch[4].i;
            Fout[i + m].i = scratch[5].i + scratch[4].r;
            Fout[i + m3].r = scratch[5].r + scratch[4].i;
            Fout[i + m3].i = scratch[5].i - scratch[4].r;
        } else {
            Fout[i + m].r = scratch[5].r + scratch[4].i;
            Fout[i + m].i = scratch[5].i - scratch[4].r;
            Fout[i + m3].r = scratch[5].r - scratch[4].i;
            Fout[i + m3].i = scratch[5].i + scratch[4].r;
        }
    }
}

static void kf_bfly_generic(cpx *Fout, int fstride, const kiss_fft_state &st, int m, int p) {
    std::vector<cpx> scratch(p);
    for (int u = 0; u < m; u++) {
        for (int q = 0; q < p; q++) scratch[q] = Fout[u + q * m];
        for (int q = 0; q < p; q++) {
            int twidx = 0;
            Fout[u + q * m] = scratch[0];
            for (int k = 1; k < p; k++) {
                twidx += fstride * q * k;
                if (twidx >= st.nfft) twidx -= st.nfft * (twidx / st.nfft);
                cpx t;
                t.r = scratch[k].r * st.twiddles[twidx].r - scratch[k].i * st.twiddles[twidx].i;
                t.i = scratch[k].r * st.twiddles[twidx].i + scratch[k].i * st.twiddles[twidx].r;
                Fout[u + q * m].r += t.r;
                Fout[u + q * m].i += t.i;
            }
        }
    }
}

static void kf_work(cpx *Fout, const cpx *f, int fstride, int in_stride,
                     const int *factors, const kiss_fft_state &st) {
    int p = *factors++;
    int m = *factors++;
    if (m == 1) {
        for (int i = 0; i < p * m; i++) {
            Fout[i] = f[i * fstride * in_stride];
        }
    } else {
        for (int i = 0; i < p; i++) {
            kf_work(Fout + i * m, f + i * fstride * in_stride, fstride * p, in_stride, factors, st);
        }
    }
    switch (p) {
        case 2: kf_bfly2(Fout, fstride, st, m); break;
        case 4: kf_bfly4(Fout, fstride, st, m); break;
        default: kf_bfly_generic(Fout, fstride, st, m, p); break;
    }
}

static void factorize(int n, std::vector<int> &factors) {
    int p = 4;
    int floor_sqrt = (int)sqrt((double)n);
    while (n > 1) {
        while (n % p) {
            switch (p) {
                case 4: p = 2; break;
                case 2: p = 3; break;
                default: p += 2; break;
            }
            if (p > floor_sqrt) p = n;
        }
        n /= p;
        factors.push_back(p);
        factors.push_back(n);
    }
}

static kiss_fft_state make_state(int nfft, int inverse) {
    kiss_fft_state st;
    st.nfft = nfft;
    st.inverse = inverse;
    st.twiddles.resize(nfft);
    for (int i = 0; i < nfft; i++) {
        double phase = -2.0 * M_PI * i / nfft;
        if (inverse) phase = -phase;
        st.twiddles[i].r = (float)cos(phase);
        st.twiddles[i].i = (float)sin(phase);
    }
    factorize(nfft, st.factors);
    return st;
}

static void fft(const kiss_fft_state &st, const cpx *fin, cpx *fout) {
    kf_work(fout, fin, 1, 1, st.factors.data(), st);
}

// Fonction utilitaire : FFT réelle (entrée float, sortie N/2+1 complexes)
static void rfft(const kiss_fft_state &st, const float *input, cpx *output) {
    int N = st.nfft;
    std::vector<cpx> tmp_in(N), tmp_out(N);
    for (int i = 0; i < N; i++) { tmp_in[i].r = input[i]; tmp_in[i].i = 0; }
    fft(st, tmp_in.data(), tmp_out.data());
    for (int i = 0; i <= N / 2; i++) output[i] = tmp_out[i];
}

} // namespace kissfft

// ═══════════════════════════════════════════════════════════════════════════════
// Configuration
// ═══════════════════════════════════════════════════════════════════════════════

static const int    BAUD_RATE       = 921600;
static const float  FS              = 50000.0f;     // fréquence d'échantillonnage originale
static const float  FS_STREAM       = 25000.0f;     // flux décimé 2:1
static const int    N_STREAM        = 1024;          // échantillons par trame
static const float  DOPPLER_SCALE   = 161.0f;        // Hz/(m/s) pour 24.125 GHz
static const int    SCOPE_WINDOW    = 512;            // échantillons affichés en scope
static const int    FFT_DISPLAY_BINS = N_STREAM / 2;  // 512 bins
static const int    SPECTRO_ROWS    = 100;            // profondeur du spectrogramme
static const float  MAX_DISPLAY_FREQ = 8000.0f;       // Hz max affiché

struct ScopeProcessingConfig {
    float max_display_freq_hz;
    float mains_freq_hz;
    int mains_harmonics;
    float mains_notch_width_hz;
    bool adaptive_noise_enable;
    float noise_rise_alpha;
    float noise_fall_alpha;
    float noise_offset_db;
    bool notch_enable;
    float notch_center_hz;
    float notch_width_hz;
    float notch_depth_db;
};

static std::mutex g_proc_mtx;
static ScopeProcessingConfig g_proc = {
    MAX_DISPLAY_FREQ,
    50.0f,
    8,
    8.0f,
    true,
    0.01f,
    0.20f,
    6.0f,
    false,
    575.0f,
    90.0f,
    20.0f
};

static ScopeProcessingConfig get_processing_config() {
    std::lock_guard<std::mutex> lk(g_proc_mtx);
    return g_proc;
}

static void set_processing_config(const ScopeProcessingConfig &cfg) {
    std::lock_guard<std::mutex> lk(g_proc_mtx);
    g_proc = cfg;
}

// Couleurs de l'interface (format ImVec4 = R, G, B, A)
static const ImVec4 COL_GREEN   = ImVec4(0.0f, 1.0f, 0.53f, 1.0f);
static const ImVec4 COL_BLUE    = ImVec4(0.27f, 0.67f, 1.0f, 1.0f);
static const ImVec4 COL_ORANGE  = ImVec4(1.0f, 0.4f, 0.0f, 1.0f);
static const ImVec4 COL_RED     = ImVec4(1.0f, 0.27f, 0.27f, 1.0f);
static const ImVec4 COL_GREY    = ImVec4(0.4f, 0.4f, 0.4f, 1.0f);
static const ImVec4 COL_WHITE   = ImVec4(1.0f, 1.0f, 1.0f, 1.0f);

// ═══════════════════════════════════════════════════════════════════════════════
// Données partagées (thread série ↔ thread GUI)
// ═══════════════════════════════════════════════════════════════════════════════

struct SharedData {
    std::mutex mtx;

    // Échantillons ADC (après DC removal)
    float adc_samples[N_STREAM] = {};

    // Spectre FFT en dB (512 bins)
    float fft_db[FFT_DISPLAY_BINS] = {};

    // Résultats envoyés par le STM32
    float freq       = 0.0f;
    float speed_kmh  = 0.0f;
    float speed_ms   = 0.0f;
    float snr        = 0.0f;
    bool  detected   = false;

    // Spectrogramme : matrice SPECTRO_ROWS × FFT_DISPLAY_BINS
    // Stocké linéairement, chaque ligne = un spectre FFT
    float spectrogram[SPECTRO_ROWS * FFT_DISPLAY_BINS] = {};
    int   spectro_write_idx = 0;  // index circulaire de la prochaine ligne

    // Statistiques
    float fps       = 0.0f;
    bool  connected = false;
    int   frame_count = 0;
    float noise_floor_db = -80.0f;
};

static SharedData g_shared;
static std::atomic<bool> g_running(true);
static std::string g_port = "COM5";

// ═══════════════════════════════════════════════════════════════════════════════
// Fenêtre de Hann pré-calculée
// ═══════════════════════════════════════════════════════════════════════════════

static float g_hann[N_STREAM];

static void init_hann_window() {
    for (int i = 0; i < N_STREAM; i++) {
        g_hann[i] = 0.5f * (1.0f - cosf(2.0f * (float)M_PI * i / (N_STREAM - 1)));
    }
}

static bool is_near_mains(float f, float mains_f, int harmonics, float width_hz) {
    for (int h = 1; h <= harmonics; h++) {
        if (fabsf(f - mains_f * h) <= width_hz) return true;
    }
    return false;
}

// ═══════════════════════════════════════════════════════════════════════════════
// Algorithme de trigger (stabilisation oscilloscope)
// ═══════════════════════════════════════════════════════════════════════════════

/**
 * @brief  Calcul automatique du seuil de trigger (mode AUTO).
 * Retourne le point médian entre le min et le max du signal.
 */
static float compute_trigger_level(const float *data, int len) {
    float vmin = data[0], vmax = data[0];
    for (int i = 1; i < len; i++) {
        if (data[i] < vmin) vmin = data[i];
        if (data[i] > vmax) vmax = data[i];
    }
    float amp = vmax - vmin;
    if (amp < 20.0f) return (vmin + vmax) * 0.5f;
    return vmin + amp * 0.5f;
}

/**
 * @brief  Recherche du point de trigger (front montant).
 * Retourne l'index du premier croisement du seuil, ou 0 si non trouvé.
 */
static int find_trigger_point(const float *data, int len, float level, int holdoff = 50) {
    int search_end = len - SCOPE_WINDOW - 10;
    if (search_end < holdoff) return 0;
    for (int i = holdoff; i < search_end; i++) {
        if (data[i - 1] < level && data[i] >= level)
            return i;
    }
    return 0;
}

// ═══════════════════════════════════════════════════════════════════════════════
// Lecture série Win32 + décodage du protocole binaire (thread dédié)
// ═══════════════════════════════════════════════════════════════════════════════

/**
 * @brief  Thread de lecture série.
 *
 * Ouvre le port COM en Win32 API, configure le baud rate,
 * lit en continu et parse les trames binaires du STM32.
 */
static void serial_thread_func() {
    // État KissFFT pour le calcul FFT côté PC
    kissfft::kiss_fft_state fft_state = kissfft::make_state(N_STREAM, 0);
    std::vector<kissfft::cpx> fft_out(N_STREAM / 2 + 1);

    // Compteur FPS
    int fps_counter = 0;
    auto fps_timer = std::chrono::steady_clock::now();
    float adaptive_noise_floor_db = -80.0f;

    while (g_running) {
        // ── Ouverture du port série Win32 ──
        std::string port_path = "\\\\.\\" + g_port;
        HANDLE hSerial = CreateFileA(
            port_path.c_str(),
            GENERIC_READ | GENERIC_WRITE,
            0, NULL,
            OPEN_EXISTING,
            0, NULL
        );

        if (hSerial == INVALID_HANDLE_VALUE) {
            fprintf(stderr, "[!] Cannot open %s (error %lu). Retrying...\n",
                    g_port.c_str(), GetLastError());
            Sleep(2000);
            continue;
        }

        // ── Configuration du port série ──
        DCB dcb = {};
        dcb.DCBlength = sizeof(dcb);
        GetCommState(hSerial, &dcb);
        dcb.BaudRate = BAUD_RATE;
        dcb.ByteSize = 8;
        dcb.StopBits = ONESTOPBIT;
        dcb.Parity   = NOPARITY;
        dcb.fBinary  = TRUE;
        dcb.fDtrControl = DTR_CONTROL_ENABLE;
        SetCommState(hSerial, &dcb);

        // Timeouts : on veut des lectures non-bloquantes avec un léger délai
        COMMTIMEOUTS timeouts = {};
        timeouts.ReadIntervalTimeout         = 10;
        timeouts.ReadTotalTimeoutMultiplier  = 0;
        timeouts.ReadTotalTimeoutConstant    = 50;
        SetCommTimeouts(hSerial, &timeouts);

        // Vider le buffer de réception
        PurgeComm(hSerial, PURGE_RXCLEAR | PURGE_TXCLEAR);

        printf("[*] Connected to %s @ %d baud\n", g_port.c_str(), BAUD_RATE);
        {
            std::lock_guard<std::mutex> lk(g_shared.mtx);
            g_shared.connected = true;
        }

        // ── Buffer d'accumulation ──
        std::vector<uint8_t> buf;
        buf.reserve(32768);

        uint8_t chunk[8192];
        bool serial_ok = true;

        while (g_running && serial_ok) {
            // Lecture des données disponibles
            DWORD bytes_read = 0;
            if (!ReadFile(hSerial, chunk, sizeof(chunk), &bytes_read, NULL)) {
                serial_ok = false;
                break;
            }
            if (bytes_read == 0) continue;

            buf.insert(buf.end(), chunk, chunk + bytes_read);

            // ── Boucle de parsing des trames ──
            while (buf.size() >= 4) {
                // Recherche du sync 0xAA 0x55
                size_t idx = std::string::npos;
                for (size_t i = 0; i + 1 < buf.size(); i++) {
                    if (buf[i] == 0xAA && buf[i + 1] == 0x55) {
                        idx = i;
                        break;
                    }
                }

                if (idx == std::string::npos) {
                    // Pas de sync trouvé, garder le dernier octet
                    if (buf.size() > 1)
                        buf.erase(buf.begin(), buf.end() - 1);
                    break;
                }

                // Jeter les octets avant le sync
                if (idx > 0)
                    buf.erase(buf.begin(), buf.begin() + idx);

                // Vérifier qu'on a assez pour le compteur
                if (buf.size() < 4) break;

                uint16_t n_samples = buf[2] | (buf[3] << 8);
                if (n_samples == 0 || n_samples > 8192) {
                    buf.erase(buf.begin(), buf.begin() + 2);
                    continue;
                }

                // Taille totale de la trame
                size_t frame_size = 4 + (size_t)n_samples * 2 + 2 + 17;
                if (buf.size() < frame_size) break;

                // Vérifier l'en-tête FFT
                size_t fft_hdr_off = 4 + (size_t)n_samples * 2;
                if (buf[fft_hdr_off] != 0xBB || buf[fft_hdr_off + 1] != 0x66) {
                    buf.erase(buf.begin(), buf.begin() + 2);
                    continue;
                }

                // ── Décodage des échantillons ADC ──
                float samples_f[N_STREAM];
                int actual_n = (n_samples > N_STREAM) ? N_STREAM : n_samples;
                double sum = 0.0;
                for (int i = 0; i < actual_n; i++) {
                    uint16_t val = buf[4 + i * 2] | (buf[4 + i * 2 + 1] << 8);
                    samples_f[i] = (float)val;
                    sum += val;
                }
                // DC removal
                float mean = (float)(sum / actual_n);
                for (int i = 0; i < actual_n; i++)
                    samples_f[i] -= mean;

                // ── Décodage des résultats FFT du STM32 ──
                size_t r = fft_hdr_off + 2;
                float freq_val, spd_kmh, spd_ms, snr_val;
                memcpy(&freq_val, &buf[r],      4);
                memcpy(&spd_kmh,  &buf[r + 4],  4);
                memcpy(&spd_ms,   &buf[r + 8],  4);
                memcpy(&snr_val,  &buf[r + 12], 4);
                uint8_t det_val = buf[r + 16];

                // ── Calcul FFT côté PC (KissFFT) ──
                float windowed[N_STREAM];
                for (int i = 0; i < actual_n; i++)
                    windowed[i] = samples_f[i] * g_hann[i];
                for (int i = actual_n; i < N_STREAM; i++)
                    windowed[i] = 0.0f;

                kissfft::rfft(fft_state, windowed, fft_out.data());

                float fft_db_local[FFT_DISPLAY_BINS];
                for (int i = 0; i < FFT_DISPLAY_BINS; i++) {
                    float mag = sqrtf(fft_out[i].r * fft_out[i].r +
                                      fft_out[i].i * fft_out[i].i);
                    fft_db_local[i] = 20.0f * log10f(mag + 1e-12f);
                }

                ScopeProcessingConfig cfg = get_processing_config();
                int max_bin = (int)(cfg.max_display_freq_hz / (FS_STREAM / N_STREAM));
                if (max_bin > FFT_DISPLAY_BINS) max_bin = FFT_DISPLAY_BINS;
                if (max_bin < 2) max_bin = FFT_DISPLAY_BINS;
                float freq_res = FS_STREAM / N_STREAM;

                for (int i = 1; i < max_bin; i++) {
                    float fi = i * freq_res;
                    if (is_near_mains(fi, cfg.mains_freq_hz, cfg.mains_harmonics, cfg.mains_notch_width_hz))
                        fft_db_local[i] -= 8.0f;
                    if (cfg.notch_enable && fabsf(fi - cfg.notch_center_hz) <= cfg.notch_width_hz * 0.5f)
                        fft_db_local[i] -= cfg.notch_depth_db;
                }

                std::vector<float> noise_bins;
                noise_bins.reserve(max_bin);
                for (int i = 1; i < max_bin; i++) {
                    float fi = i * freq_res;
                    if (cfg.notch_enable && fabsf(fi - cfg.notch_center_hz) <= cfg.notch_width_hz * 0.5f) continue;
                    noise_bins.push_back(fft_db_local[i]);
                }
                if (!noise_bins.empty()) {
                    size_t m = noise_bins.size() / 2;
                    std::nth_element(noise_bins.begin(), noise_bins.begin() + m, noise_bins.end());
                    float inst = noise_bins[m];
                    if (cfg.adaptive_noise_enable) {
                        float a = (inst > adaptive_noise_floor_db) ? cfg.noise_rise_alpha : cfg.noise_fall_alpha;
                        a = std::clamp(a, 0.001f, 1.0f);
                        adaptive_noise_floor_db += a * (inst - adaptive_noise_floor_db);
                    } else {
                        adaptive_noise_floor_db = inst;
                    }
                }

                // ── Mise à jour des données partagées ──
                {
                    std::lock_guard<std::mutex> lk(g_shared.mtx);
                    memcpy(g_shared.adc_samples, samples_f, actual_n * sizeof(float));
                    memcpy(g_shared.fft_db, fft_db_local, sizeof(fft_db_local));
                    g_shared.freq      = freq_val;
                    g_shared.speed_kmh = spd_kmh;
                    g_shared.speed_ms  = spd_ms;
                    g_shared.snr       = snr_val;
                    g_shared.detected  = (det_val == 1);
                    g_shared.noise_floor_db = adaptive_noise_floor_db;
                    g_shared.frame_count++;

                    // Spectrogramme circulaire
                    int row = g_shared.spectro_write_idx % SPECTRO_ROWS;
                    memcpy(&g_shared.spectrogram[row * FFT_DISPLAY_BINS],
                           fft_db_local, sizeof(fft_db_local));
                    g_shared.spectro_write_idx++;
                }

                // FPS
                fps_counter++;
                auto now = std::chrono::steady_clock::now();
                double elapsed = std::chrono::duration<double>(now - fps_timer).count();
                if (elapsed >= 1.0) {
                    std::lock_guard<std::mutex> lk(g_shared.mtx);
                    g_shared.fps = (float)(fps_counter / elapsed);
                    fps_counter = 0;
                    fps_timer = now;
                }

                // Consommer la trame
                buf.erase(buf.begin(), buf.begin() + frame_size);
            }

            // Limiter la taille du buffer pour éviter une croissance infinie
            if (buf.size() > 65536) {
                buf.erase(buf.begin(), buf.end() - 4096);
            }
        }

        // Déconnexion
        CloseHandle(hSerial);
        {
            std::lock_guard<std::mutex> lk(g_shared.mtx);
            g_shared.connected = false;
        }
        fprintf(stderr, "[!] Serial disconnected. Retrying in 2s...\n");
        Sleep(2000);
    }
}

// ═══════════════════════════════════════════════════════════════════════════════
// Utilitaire : dessin d'un graphe en ligne dans ImGui
// ═══════════════════════════════════════════════════════════════════════════════

/**
 * @brief  Dessine un graphe de données avec ImGui DrawList (plus flexible que PlotLines).
 * @param  label     Titre du graphe
 * @param  data      Tableau de valeurs Y
 * @param  count     Nombre de points
 * @param  x_min, x_max  Plage de l'axe X (unités réelles)
 * @param  y_min, y_max  Plage de l'axe Y
 * @param  size      Taille du widget
 * @param  color     Couleur de la courbe
 */
static void DrawGraph(const char *label, const float *data, int count,
                      float x_min, float x_max, float y_min, float y_max,
                      ImVec2 size, ImU32 color,
                      float vline_x = -1.0f, ImU32 vline_color = 0) {
    ImGui::BeginChild(label, size, true, ImGuiWindowFlags_NoScrollbar);

    ImDrawList *dl = ImGui::GetWindowDrawList();
    ImVec2 p = ImGui::GetCursorScreenPos();
    ImVec2 region = ImGui::GetContentRegionAvail();
    float w = region.x, h = region.y;

    // Fond sombre
    dl->AddRectFilled(p, ImVec2(p.x + w, p.y + h), IM_COL32(15, 15, 20, 255));

    // Grille
    for (int i = 1; i < 4; i++) {
        float yy = p.y + h * i / 4.0f;
        dl->AddLine(ImVec2(p.x, yy), ImVec2(p.x + w, yy), IM_COL32(50, 50, 50, 100));
    }
    for (int i = 1; i < 8; i++) {
        float xx = p.x + w * i / 8.0f;
        dl->AddLine(ImVec2(xx, p.y), ImVec2(xx, p.y + h), IM_COL32(50, 50, 50, 100));
    }

    // Courbe
    if (count > 1) {
        for (int i = 0; i < count - 1; i++) {
            float x0 = p.x + (float)i / (count - 1) * w;
            float x1 = p.x + (float)(i + 1) / (count - 1) * w;
            float v0 = (data[i] - y_min) / (y_max - y_min);
            float v1 = (data[i + 1] - y_min) / (y_max - y_min);
            v0 = 1.0f - std::clamp(v0, 0.0f, 1.0f);
            v1 = 1.0f - std::clamp(v1, 0.0f, 1.0f);
            dl->AddLine(ImVec2(x0, p.y + v0 * h),
                        ImVec2(x1, p.y + v1 * h), color, 1.5f);
        }
    }

    // Ligne verticale optionnelle (marqueur de pic)
    if (vline_x >= x_min && vline_x <= x_max && vline_color != 0) {
        float xp = p.x + (vline_x - x_min) / (x_max - x_min) * w;
        dl->AddLine(ImVec2(xp, p.y), ImVec2(xp, p.y + h), vline_color, 1.0f);
    }

    // Label
    dl->AddText(ImVec2(p.x + 4, p.y + 2), IM_COL32(200, 200, 200, 200), label);

    ImGui::EndChild();
}

// ═══════════════════════════════════════════════════════════════════════════════
// Dessin du spectrogramme (image 2D en texture OpenGL)
// ═══════════════════════════════════════════════════════════════════════════════

static GLuint g_spectro_tex = 0;

/**
 * @brief  Convertit une valeur dB en couleur RGB (palette inferno-like).
 */
static void db_to_rgb(float db, float vmin, float vmax, uint8_t *r, uint8_t *g, uint8_t *b) {
    float t = (db - vmin) / (vmax - vmin);
    t = std::clamp(t, 0.0f, 1.0f);
    // Palette simplifiée inspirée de "inferno" :
    //   noir → violet → rouge → orange → jaune
    if (t < 0.25f) {
        float s = t / 0.25f;
        *r = (uint8_t)(s * 80);
        *g = 0;
        *b = (uint8_t)(s * 120);
    } else if (t < 0.5f) {
        float s = (t - 0.25f) / 0.25f;
        *r = (uint8_t)(80 + s * 150);
        *g = (uint8_t)(s * 30);
        *b = (uint8_t)(120 - s * 80);
    } else if (t < 0.75f) {
        float s = (t - 0.5f) / 0.25f;
        *r = (uint8_t)(230 + s * 25);
        *g = (uint8_t)(30 + s * 120);
        *b = (uint8_t)(40 - s * 40);
    } else {
        float s = (t - 0.75f) / 0.25f;
        *r = 255;
        *g = (uint8_t)(150 + s * 105);
        *b = (uint8_t)(s * 80);
    }
}

/**
 * @brief  Met à jour la texture OpenGL du spectrogramme.
 */
static void update_spectro_texture(const float *spectrogram, int write_idx,
                                    int rows, int cols, float vmin, float vmax) {
    static std::vector<uint8_t> pixels;
    // On limite le nombre de colonnes affichées à la zone 0-8kHz
    int display_cols = (int)(MAX_DISPLAY_FREQ / (FS_STREAM / 2.0f) * cols);
    if (display_cols > cols) display_cols = cols;

    pixels.resize(rows * display_cols * 3);

    for (int r = 0; r < rows; r++) {
        // L'index de lecture part de write_idx (la ligne la plus ancienne)
        int src_row = (write_idx + r) % rows;
        for (int c = 0; c < display_cols; c++) {
            int pi = (r * display_cols + c) * 3;
            db_to_rgb(spectrogram[src_row * cols + c], vmin, vmax,
                      &pixels[pi], &pixels[pi + 1], &pixels[pi + 2]);
        }
    }

    if (g_spectro_tex == 0) {
        glGenTextures(1, &g_spectro_tex);
        glBindTexture(GL_TEXTURE_2D, g_spectro_tex);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    } else {
        glBindTexture(GL_TEXTURE_2D, g_spectro_tex);
    }
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, display_cols, rows,
                 0, GL_RGB, GL_UNSIGNED_BYTE, pixels.data());
}

// ═══════════════════════════════════════════════════════════════════════════════
// Boucle principale : fenêtre GLFW + rendu ImGui
// ═══════════════════════════════════════════════════════════════════════════════

int main(int argc, char **argv) {
    // Argument optionnel : nom du port COM
    if (argc > 1) g_port = argv[1];

    printf("==============================================\n");
    printf("  RADAR DOPPLER 24 GHz - Live Scope (C++)\n");
    printf("  Port: %s  Baud: %d\n", g_port.c_str(), BAUD_RATE);
    printf("==============================================\n");

    // Initialisation de la fenêtre de Hann
    init_hann_window();

    // Initialisation du spectrogramme à -80 dB
    for (int i = 0; i < SPECTRO_ROWS * FFT_DISPLAY_BINS; i++)
        g_shared.spectrogram[i] = -80.0f;

    // ── Initialisation GLFW ──
    if (!glfwInit()) {
        fprintf(stderr, "Failed to initialize GLFW\n");
        return 1;
    }
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

    GLFWwindow *window = glfwCreateWindow(1400, 900,
        "Radar Doppler 24.125 GHz - Live Scope", NULL, NULL);
    if (!window) {
        fprintf(stderr, "Failed to create GLFW window\n");
        glfwTerminate();
        return 1;
    }
    glfwMakeContextCurrent(window);
    glfwSwapInterval(1);  // V-Sync ON (60 fps max, pas besoin de plus)

    // ── Initialisation Dear ImGui ──
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGuiIO &io = ImGui::GetIO();
    io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;

    // Style sombre personnalisé
    ImGui::StyleColorsDark();
    ImGuiStyle &style = ImGui::GetStyle();
    style.WindowRounding = 4.0f;
    style.FrameRounding  = 3.0f;
    style.ItemSpacing    = ImVec2(8, 4);
    style.WindowPadding  = ImVec2(8, 8);

    ImGui_ImplGlfw_InitForOpenGL(window, true);
    ImGui_ImplOpenGL3_Init("#version 330");

    // Police plus grande pour la lisibilité
    io.Fonts->AddFontDefault();

    // ── Lancement du thread série ──
    std::thread serial_thread(serial_thread_func);

    // ── Buffers locaux pour le rendu (évite de tenir le lock pendant le draw) ──
    float local_adc[N_STREAM]          = {};
    float local_fft[FFT_DISPLAY_BINS]  = {};
    float local_spectro[SPECTRO_ROWS * FFT_DISPLAY_BINS] = {};
    int   local_spectro_idx = 0;
    float local_freq = 0, local_spd_kmh = 0, local_spd_ms = 0, local_snr = 0;
    bool  local_detected = false;
    float local_fps = 0;
    bool  local_connected = false;
    float local_noise_floor_db = -80.0f;
    float scope_display[SCOPE_WINDOW] = {};
    enum LayoutMode { LAYOUT_DEFAULT = 0, LAYOUT_FFT_ONLY, LAYOUT_SCOPE_ONLY, LAYOUT_SPECTRO_ONLY };
    static int layout_mode = LAYOUT_DEFAULT;
    static float top_ratio = 0.30f;
    static float mid_ratio = 0.35f;
    static bool show_help_window = false;

    // ══════════════════════════════════════════════════════════════
    // Boucle de rendu principale
    // ══════════════════════════════════════════════════════════════
    while (!glfwWindowShouldClose(window) && g_running) {
        glfwPollEvents();

        // ── Copie atomique des données partagées ──
        {
            std::lock_guard<std::mutex> lk(g_shared.mtx);
            memcpy(local_adc, g_shared.adc_samples, sizeof(local_adc));
            memcpy(local_fft, g_shared.fft_db, sizeof(local_fft));
            memcpy(local_spectro, g_shared.spectrogram, sizeof(local_spectro));
            local_spectro_idx = g_shared.spectro_write_idx;
            local_freq      = g_shared.freq;
            local_spd_kmh   = g_shared.speed_kmh;
            local_spd_ms    = g_shared.speed_ms;
            local_snr       = g_shared.snr;
            local_detected  = g_shared.detected;
            local_fps       = g_shared.fps;
            local_connected = g_shared.connected;
            local_noise_floor_db = g_shared.noise_floor_db;
        }

        // ── Trigger pour l'oscilloscope ──
        float trig_level = compute_trigger_level(local_adc, N_STREAM);
        int trig_idx = find_trigger_point(local_adc, N_STREAM, trig_level);
        int end_idx = trig_idx + SCOPE_WINDOW;
        if (end_idx > N_STREAM) {
            end_idx = N_STREAM;
            trig_idx = std::max(0, end_idx - SCOPE_WINDOW);
        }
        int copy_len = std::min(SCOPE_WINDOW, N_STREAM - trig_idx);
        memcpy(scope_display, &local_adc[trig_idx], copy_len * sizeof(float));
        if (copy_len < SCOPE_WINDOW)
            memset(&scope_display[copy_len], 0, (SCOPE_WINDOW - copy_len) * sizeof(float));

        // Auto-scale Y pour le scope
        float scope_max = 100.0f;
        for (int i = 0; i < SCOPE_WINDOW; i++) {
            float a = fabsf(scope_display[i]);
            if (a > scope_max) scope_max = a;
        }
        scope_max *= 1.2f;

        // ── Nouveau frame ImGui ──
        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();

        // Fenêtre plein écran
        ImGui::SetNextWindowPos(ImVec2(0, 0));
        ImGui::SetNextWindowSize(io.DisplaySize);
        ImGui::Begin("Radar", NULL,
            ImGuiWindowFlags_MenuBar |
            ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoResize |
            ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoCollapse |
            ImGuiWindowFlags_NoBringToFrontOnFocus);

        ScopeProcessingConfig ui_cfg = get_processing_config();
        bool proc_changed = false;
        if (ImGui::BeginMenuBar()) {
            if (ImGui::BeginMenu("Live Controls")) {
                if (ImGui::BeginMenu("Layout")) {
                    if (ImGui::MenuItem("Default", NULL, layout_mode == LAYOUT_DEFAULT)) layout_mode = LAYOUT_DEFAULT;
                    if (ImGui::MenuItem("FFT only", NULL, layout_mode == LAYOUT_FFT_ONLY)) layout_mode = LAYOUT_FFT_ONLY;
                    if (ImGui::MenuItem("Scope only", NULL, layout_mode == LAYOUT_SCOPE_ONLY)) layout_mode = LAYOUT_SCOPE_ONLY;
                    if (ImGui::MenuItem("Spectrogram only", NULL, layout_mode == LAYOUT_SPECTRO_ONLY)) layout_mode = LAYOUT_SPECTRO_ONLY;
                    ImGui::SliderFloat("Top row ratio", &top_ratio, 0.15f, 0.70f, "%.2f");
                    ImGui::SliderFloat("Middle row ratio", &mid_ratio, 0.10f, 0.70f, "%.2f");
                    ImGui::EndMenu();
                }
                if (ImGui::BeginMenu("Noise / Mains / Notch")) {
                    proc_changed |= ImGui::SliderFloat("Fmax FFT (Hz)", &ui_cfg.max_display_freq_hz, 1000.0f, FS_STREAM * 0.5f, "%.0f");
                    proc_changed |= ImGui::SliderFloat("Mains freq (Hz)", &ui_cfg.mains_freq_hz, 45.0f, 65.0f, "%.1f");
                    proc_changed |= ImGui::SliderInt("Mains harmonics", &ui_cfg.mains_harmonics, 1, 20);
                    proc_changed |= ImGui::SliderFloat("Mains notch width (Hz)", &ui_cfg.mains_notch_width_hz, 1.0f, 30.0f, "%.1f");
                    proc_changed |= ImGui::Checkbox("Adaptive noise floor", &ui_cfg.adaptive_noise_enable);
                    proc_changed |= ImGui::SliderFloat("Noise rise alpha", &ui_cfg.noise_rise_alpha, 0.001f, 1.0f, "%.3f");
                    proc_changed |= ImGui::SliderFloat("Noise fall alpha", &ui_cfg.noise_fall_alpha, 0.001f, 1.0f, "%.3f");
                    proc_changed |= ImGui::SliderFloat("Noise floor offset (dB)", &ui_cfg.noise_offset_db, 0.0f, 30.0f, "%.1f");
                    proc_changed |= ImGui::Checkbox("Enable notch", &ui_cfg.notch_enable);
                    proc_changed |= ImGui::SliderFloat("Notch center (Hz)", &ui_cfg.notch_center_hz, 10.0f, FS_STREAM * 0.5f, "%.1f");
                    proc_changed |= ImGui::SliderFloat("Notch width (Hz)", &ui_cfg.notch_width_hz, 1.0f, 1000.0f, "%.1f");
                    proc_changed |= ImGui::SliderFloat("Notch depth (dB)", &ui_cfg.notch_depth_db, 0.0f, 80.0f, "%.1f");
                    ImGui::EndMenu();
                }
                ImGui::EndMenu();
            }
            if (ImGui::BeginMenu("Help")) {
                if (ImGui::MenuItem("Open full help")) show_help_window = true;
                ImGui::EndMenu();
            }
            ImGui::EndMenuBar();
        }
        if (proc_changed) {
            ui_cfg.max_display_freq_hz = std::clamp(ui_cfg.max_display_freq_hz, 1000.0f, FS_STREAM * 0.5f);
            ui_cfg.mains_harmonics = std::max(1, ui_cfg.mains_harmonics);
            ui_cfg.notch_center_hz = std::clamp(ui_cfg.notch_center_hz, 10.0f, FS_STREAM * 0.5f);
            ui_cfg.notch_width_hz = std::clamp(ui_cfg.notch_width_hz, 1.0f, 1000.0f);
            ui_cfg.notch_depth_db = std::clamp(ui_cfg.notch_depth_db, 0.0f, 80.0f);
            ui_cfg.noise_rise_alpha = std::clamp(ui_cfg.noise_rise_alpha, 0.001f, 1.0f);
            ui_cfg.noise_fall_alpha = std::clamp(ui_cfg.noise_fall_alpha, 0.001f, 1.0f);
            set_processing_config(ui_cfg);
        }
        if (show_help_window) {
            ImGui::Begin("Scope Help", &show_help_window, ImGuiWindowFlags_AlwaysAutoResize);
            ImGui::TextWrapped("Live Controls -> Layout lets you focus on a single window (FFT/scope/spectrogram).");
            ImGui::BulletText("Noise / Mains / Notch controls 50Hz filtering and interference suppression.");
            ImGui::BulletText("Use Adaptive noise floor in unstable noise environments.");
            ImGui::End();
        }

        // ── Titre ──
        ImGui::TextColored(COL_WHITE, "RADAR DOPPLER 24.125 GHz - Live Analysis");
        ImGui::SameLine(ImGui::GetWindowWidth() - 250);
        if (local_connected) {
            ImGui::TextColored(COL_GREEN, "CONNECTED");
            ImGui::SameLine();
            ImGui::TextColored(COL_GREY, "%.1f fps | %s", local_fps, g_port.c_str());
        } else {
            ImGui::TextColored(COL_RED, "DISCONNECTED");
        }
        ImGui::Separator();

        float panel_w = ImGui::GetContentRegionAvail().x;
        float panel_h = ImGui::GetContentRegionAvail().y;
        float scope_h = 0.0f;
        float mid_h = 0.0f;
        float half_w = 0.0f;
        float info_h = 0.0f;

        top_ratio = std::clamp(top_ratio, 0.15f, 0.70f);
        mid_ratio = std::clamp(mid_ratio, 0.10f, 0.70f);
        if (top_ratio + mid_ratio > 0.90f) mid_ratio = 0.90f - top_ratio;

        if (layout_mode == LAYOUT_SCOPE_ONLY) {
            DrawGraph("Oscilloscope (FULL)", scope_display, SCOPE_WINDOW,
                      0, SCOPE_WINDOW / FS_STREAM * 1000.0f, -scope_max, scope_max,
                      ImVec2(panel_w, panel_h - 8), IM_COL32(0, 255, 136, 255));
            ImGui::End();
            goto render_scope;
        }
        if (layout_mode == LAYOUT_FFT_ONLY) {
            DrawGraph("FFT Spectrum (FULL)", local_fft, FFT_DISPLAY_BINS,
                      0, FS_STREAM / 2.0f, -20, 80,
                      ImVec2(panel_w, panel_h - 8), IM_COL32(68, 170, 255, 255),
                      local_detected ? local_freq : -1.0f, IM_COL32(255, 68, 68, 200));
            ImGui::End();
            goto render_scope;
        }
        if (layout_mode == LAYOUT_SPECTRO_ONLY) {
            ImGui::BeginChild("SpectrogramOnly", ImVec2(panel_w, panel_h - 8), true, ImGuiWindowFlags_NoScrollbar);
            ImDrawList *dl = ImGui::GetWindowDrawList();
            ImVec2 p = ImGui::GetCursorScreenPos();
            ImVec2 region = ImGui::GetContentRegionAvail();
            update_spectro_texture(local_spectro, local_spectro_idx, SPECTRO_ROWS, FFT_DISPLAY_BINS, -20, 70);
            if (g_spectro_tex) {
                dl->AddImage((ImTextureID)(intptr_t)g_spectro_tex, p, ImVec2(p.x + region.x, p.y + region.y), ImVec2(0, 0), ImVec2(1, 1));
            }
            ImGui::EndChild();
            ImGui::End();
            goto render_scope;
        }

        // ── Panneau 1 : Oscilloscope (pleine largeur, ratio hauteur) ──
        scope_h = panel_h * top_ratio;
        char scope_label[64];
        snprintf(scope_label, sizeof(scope_label),
                 "Oscilloscope (ADC)  |  Trigger: %.0f  |  Window: %.2f ms",
                 trig_level, SCOPE_WINDOW / FS_STREAM * 1000.0f);
        DrawGraph(scope_label, scope_display, SCOPE_WINDOW,
                  0, SCOPE_WINDOW / FS_STREAM * 1000.0f,
                  -scope_max, scope_max,
                  ImVec2(panel_w, scope_h),
                  IM_COL32(0, 255, 136, 255));

        // ── Ligne du milieu : FFT (50%) + Spectrogramme (50%) ──
        mid_h = panel_h * mid_ratio;
        half_w = panel_w * 0.5f - 4;

        ImGui::BeginChild("mid_row", ImVec2(panel_w, mid_h), false);

        // FFT Spectrum
        {
            char fft_label[128];
            if (local_detected)
                snprintf(fft_label, sizeof(fft_label),
                         "FFT Spectrum  |  Peak: %.1f Hz (%.1f dB)",
                         local_freq, local_snr);
            else
                snprintf(fft_label, sizeof(fft_label),
                         "FFT Spectrum  |  No target");

            DrawGraph(fft_label, local_fft, FFT_DISPLAY_BINS,
                      0, FS_STREAM / 2.0f, -20, 80,
                      ImVec2(half_w, mid_h - 4),
                      IM_COL32(68, 170, 255, 255),
                      local_detected ? local_freq : -1.0f,
                      IM_COL32(255, 68, 68, 200));
        }

        ImGui::SameLine();

        // Spectrogramme
        {
            ImGui::BeginChild("Spectrogram", ImVec2(half_w, mid_h - 4), true,
                              ImGuiWindowFlags_NoScrollbar);
            ImDrawList *dl = ImGui::GetWindowDrawList();
            ImVec2 p = ImGui::GetCursorScreenPos();
            ImVec2 region = ImGui::GetContentRegionAvail();

            update_spectro_texture(local_spectro, local_spectro_idx,
                                    SPECTRO_ROWS, FFT_DISPLAY_BINS, -20, 70);

            if (g_spectro_tex) {
                dl->AddImage((ImTextureID)(intptr_t)g_spectro_tex,
                             p, ImVec2(p.x + region.x, p.y + region.y),
                             ImVec2(0, 0), ImVec2(1, 1));
            }
            dl->AddText(ImVec2(p.x + 4, p.y + 2),
                        IM_COL32(255, 170, 0, 200), "Spectrogram (0-8 kHz)");

            ImGui::EndChild();
        }

        ImGui::EndChild(); // mid_row

        // ── Panneau info : affichage numérique (reste) ──
        info_h = panel_h - scope_h - mid_h - 20.0f;
        ImGui::BeginChild("InfoPanel", ImVec2(panel_w, info_h), true);

        if (local_detected) {
            // Grande vitesse en vert
            ImGui::PushFont(io.Fonts->Fonts[0]);
            ImGui::SetWindowFontScale(3.5f);
            ImGui::TextColored(COL_GREEN, "  %6.1f km/h", local_spd_kmh);
            ImGui::SetWindowFontScale(1.0f);
            ImGui::PopFont();

            ImGui::Spacing();

            ImGui::TextColored(COL_WHITE,
                "  %.1f m/s   |   f = %.1f Hz   |   SNR = %.1f dB",
                local_spd_ms, local_freq, local_snr);

            ImGui::Spacing();
            ImGui::Separator();
            ImGui::Spacing();

            // Barre SNR visuelle
            float snr_frac = std::clamp(local_snr / 60.0f, 0.0f, 1.0f);
            ImGui::Text("  SNR:");
            ImGui::SameLine();
            ImGui::PushStyleColor(ImGuiCol_PlotHistogram,
                snr_frac > 0.5f ? (ImVec4)COL_GREEN : (ImVec4)COL_ORANGE);
            ImGui::ProgressBar(snr_frac, ImVec2(300, 20), "");
            ImGui::PopStyleColor();
            ImGui::SameLine();
            ImGui::Text("%.1f dB", local_snr);

            ImGui::SameLine(panel_w - 200);
            ImGui::TextColored(COL_GREEN, "TARGET DETECTED");
        } else {
            ImGui::PushFont(io.Fonts->Fonts[0]);
            ImGui::SetWindowFontScale(3.5f);
            ImGui::TextColored(COL_GREY, "   --.- km/h");
            ImGui::SetWindowFontScale(1.0f);
            ImGui::PopFont();

            ImGui::Spacing();
            ImGui::TextColored(COL_GREY,
                "  SNR = %.1f dB   |   Noise floor = %.1f dB   |   Waiting for target...",
                local_snr, local_noise_floor_db);

            ImGui::SameLine(panel_w - 200);
            ImGui::TextColored(COL_ORANGE, "NO TARGET");
        }

        ImGui::EndChild();

        ImGui::End(); // Fenêtre principale

render_scope:
        // ── Rendu ──
        ImGui::Render();
        int display_w, display_h;
        glfwGetFramebufferSize(window, &display_w, &display_h);
        glViewport(0, 0, display_w, display_h);
        glClearColor(0.06f, 0.06f, 0.08f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT);
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
        glfwSwapBuffers(window);
    }

    // ── Nettoyage ──
    g_running = false;
    if (serial_thread.joinable()) serial_thread.join();

    if (g_spectro_tex) glDeleteTextures(1, &g_spectro_tex);

    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImGui::DestroyContext();
    glfwDestroyWindow(window);
    glfwTerminate();

    return 0;
}
