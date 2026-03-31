/**
 * radar_analyzer.cpp — Analyseur radar Doppler haute résolution (PC)
 * ====================================================================
 * Application Windows C++ qui reçoit les échantillons ADC bruts du STM32
 * via UDP Ethernet et effectue un traitement du signal avancé :
 *
 *   - Réception UDP fragmentée par hops (1024 échantillons × uint16)
 *   - Fenêtre glissante 4096 échantillons reconstruite côté PC
 *   - FFT haute résolution (8192 ou 16384 points) avec fenêtre de Hann
 *   - Détecteur CFAR (Constant False Alarm Rate) Cell-Averaging
 *   - Filtre de Kalman pour le suivi de vitesse
 *   - Affichage temps réel Dear ImGui + OpenGL3 + GLFW
 *
 * Réseau : écoute sur UDP port 5555 (broadcast depuis le STM32).
 * Le PC doit avoir une IP dans 192.168.1.x (ex: 192.168.1.100).
 *
 * Compilation : voir CMakeLists.txt (CMake + MinGW ou MSVC).
 *
 * Usage :
 *   radar_analyzer.exe              (FFT 8192 par défaut)
 *   radar_analyzer.exe 16384        (FFT 16384 points)
 */

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
#include <deque>
#include <array>

/* === API réseau Windows (Winsock2) === */
#include <winsock2.h>
#include <ws2tcpip.h>
#ifdef _MSC_VER
#pragma comment(lib, "ws2_32.lib")
#endif

/* === OpenGL + GLFW + Dear ImGui === */
#include "imgui.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl3.h"
#include <GLFW/glfw3.h>

/* ═══════════════════════════════════════════════════════════════════════════
 * KissFFT embarqué (domaine public)
 * ═══════════════════════════════════════════════════════════════════════════
 * Bibliothèque FFT légère et portable, embarquée directement dans ce fichier
 * pour éviter toute dépendance externe. Source : github.com/mborgerding/kissfft
 */

namespace kissfft {

struct cpx { float r, i; };

struct kiss_fft_state {
  int nfft;
  int inverse;
  std::vector<cpx> twiddles;
  std::vector<int> factors;
};

/* Papillon radix-2 */
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

/* Papillon radix-4 */
static void kf_bfly4(cpx *Fout, int fstride, const kiss_fft_state &st, int m) {
  const cpx *tw1 = st.twiddles.data();
  const cpx *tw2 = tw1, *tw3 = tw1;
  cpx scratch[6];
  int m2 = 2 * m, m3 = 3 * m;
  for (int i = 0; i < m; i++) {
    scratch[0].r = Fout[i+m].r*tw1[i*fstride].r - Fout[i+m].i*tw1[i*fstride].i;
    scratch[0].i = Fout[i+m].r*tw1[i*fstride].i + Fout[i+m].i*tw1[i*fstride].r;
    scratch[1].r = Fout[i+m2].r*tw2[i*fstride*2].r - Fout[i+m2].i*tw2[i*fstride*2].i;
    scratch[1].i = Fout[i+m2].r*tw2[i*fstride*2].i + Fout[i+m2].i*tw2[i*fstride*2].r;
    scratch[2].r = Fout[i+m3].r*tw3[i*fstride*3].r - Fout[i+m3].i*tw3[i*fstride*3].i;
    scratch[2].i = Fout[i+m3].r*tw3[i*fstride*3].i + Fout[i+m3].i*tw3[i*fstride*3].r;
    scratch[5].r = Fout[i].r - scratch[1].r;
    scratch[5].i = Fout[i].i - scratch[1].i;
    Fout[i].r += scratch[1].r; Fout[i].i += scratch[1].i;
    scratch[3].r = scratch[0].r + scratch[2].r;
    scratch[3].i = scratch[0].i + scratch[2].i;
    scratch[4].r = scratch[0].r - scratch[2].r;
    scratch[4].i = scratch[0].i - scratch[2].i;
    Fout[i+m2].r = Fout[i].r - scratch[3].r;
    Fout[i+m2].i = Fout[i].i - scratch[3].i;
    Fout[i].r += scratch[3].r; Fout[i].i += scratch[3].i;
    if (st.inverse) {
      Fout[i+m].r = scratch[5].r - scratch[4].i;
      Fout[i+m].i = scratch[5].i + scratch[4].r;
      Fout[i+m3].r = scratch[5].r + scratch[4].i;
      Fout[i+m3].i = scratch[5].i - scratch[4].r;
    } else {
      Fout[i+m].r = scratch[5].r + scratch[4].i;
      Fout[i+m].i = scratch[5].i - scratch[4].r;
      Fout[i+m3].r = scratch[5].r - scratch[4].i;
      Fout[i+m3].i = scratch[5].i + scratch[4].r;
    }
  }
}

/* Papillon générique (pour facteurs premiers > 4) */
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

/* Travail récursif de la FFT (décomposition Cooley-Tukey) */
static void kf_work(cpx *Fout, const cpx *f, int fstride, int in_stride,
                     const int *factors, const kiss_fft_state &st) {
  int p = *factors++;
  int m = *factors++;
  if (m == 1) {
    for (int i = 0; i < p * m; i++) Fout[i] = f[i * fstride * in_stride];
  } else {
    for (int i = 0; i < p; i++)
      kf_work(Fout + i * m, f + i * fstride * in_stride, fstride * p, in_stride, factors, st);
  }
  switch (p) {
    case 2: kf_bfly2(Fout, fstride, st, m); break;
    case 4: kf_bfly4(Fout, fstride, st, m); break;
    default: kf_bfly_generic(Fout, fstride, st, m, p); break;
  }
}

/* Factorisation de N pour l'algorithme FFT mixte radix */
static void factorize(int n, std::vector<int> &factors) {
  int p = 4;
  int floor_sqrt = (int)sqrt((double)n);
  while (n > 1) {
    while (n % p) {
      switch (p) { case 4: p = 2; break; case 2: p = 3; break; default: p += 2; break; }
      if (p > floor_sqrt) p = n;
    }
    n /= p;
    factors.push_back(p);
    factors.push_back(n);
  }
}

/* Création de l'état FFT (twiddle factors + factorisation) */
static kiss_fft_state make_state(int nfft, int inverse) {
  kiss_fft_state st;
  st.nfft = nfft; st.inverse = inverse;
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

/* FFT directe (complexe → complexe) */
static void fft(const kiss_fft_state &st, const cpx *fin, cpx *fout) {
  kf_work(fout, fin, 1, 1, st.factors.data(), st);
}

/* FFT réelle : entrée float N points → sortie N/2+1 complexes */
static void rfft(const kiss_fft_state &st, const float *input, cpx *output) {
  int N = st.nfft;
  std::vector<cpx> tmp_in(N), tmp_out(N);
  for (int i = 0; i < N; i++) { tmp_in[i].r = input[i]; tmp_in[i].i = 0; }
  fft(st, tmp_in.data(), tmp_out.data());
  for (int i = 0; i <= N / 2; i++) output[i] = tmp_out[i];
}

} // namespace kissfft

/* ═══════════════════════════════════════════════════════════════════════════
 * Configuration globale
 * ═══════════════════════════════════════════════════════════════════════════ */

/* Fréquence d'échantillonnage originale du STM32 (100 kHz) */
static const float FS = 100000.0f;

/* Taille de la FFT PC (par défaut 8192, modifiable en argument) */
static int g_fft_size = 8192;

/* Taille de la fenêtre d'analyse côté PC (comme la FFT STM32) */
static const int STM32_FRAME_SIZE = 4096;
/* Taille d'un hop UDP envoyé par le STM32 (DMA HT/TC) */
static const int STM32_HOP_SIZE = 1024;

/* Facteur Doppler : fd = 2·v·f0/c ≈ 161 Hz/(m/s) pour 24.125 GHz */
static const float DOPPLER_SCALE = 161.0f;

/* Port UDP d'écoute */
static const int UDP_PORT = 5555;

/* Profondeur du spectrogramme (nombre de lignes dans le waterfall) */
static const int SPECTRO_ROWS = 150;

/* Fréquence max affichée (Hz) — limitée par le filtre AFE à 6 kHz */
static const float MAX_DISPLAY_FREQ = 8000.0f;

/* Couleurs de l'interface */
static const ImVec4 COL_GREEN  = ImVec4(0.0f, 1.0f, 0.53f, 1.0f);
static const ImVec4 COL_BLUE   = ImVec4(0.27f, 0.67f, 1.0f, 1.0f);
static const ImVec4 COL_ORANGE = ImVec4(1.0f, 0.4f, 0.0f, 1.0f);
static const ImVec4 COL_RED    = ImVec4(1.0f, 0.27f, 0.27f, 1.0f);
static const ImVec4 COL_GREY   = ImVec4(0.4f, 0.4f, 0.4f, 1.0f);
static const ImVec4 COL_WHITE  = ImVec4(1.0f, 1.0f, 1.0f, 1.0f);
static const ImVec4 COL_YELLOW = ImVec4(1.0f, 0.9f, 0.0f, 1.0f);
static const ImVec4 COL_CYAN   = ImVec4(0.0f, 0.9f, 0.9f, 1.0f);

/* ═══════════════════════════════════════════════════════════════════════════
 * Détecteur CFAR (Constant False Alarm Rate) — Cell-Averaging
 * ═══════════════════════════════════════════════════════════════════════════
 *
 * Le CFAR est le détecteur standard en traitement radar. Contrairement à un
 * seuil fixe (qui rate les cibles faibles et détecte du bruit quand le
 * plancher de bruit varie), le CFAR adapte son seuil localement.
 *
 * Principe du CA-CFAR (Cell-Averaging CFAR) :
 *   Pour chaque bin "sous test" (CUT), on estime le bruit local en moyennant
 *   les bins voisins (cellules de référence), en excluant quelques bins de
 *   garde autour du CUT pour ne pas contaminer l'estimation avec l'énergie
 *   du signal lui-même.
 *
 *   Seuil = alpha × moyenne(cellules de référence)
 *   Si CUT > seuil → détection
 *
 * Paramètres :
 *   - guard_cells : bins de garde de chaque côté (typiquement 3-5)
 *   - ref_cells   : bins de référence de chaque côté (typiquement 10-20)
 *   - alpha       : facteur multiplicatif (dépend du Pfa souhaité)
 *
 * Pour N cellules de référence et une probabilité de fausse alarme Pfa :
 *   alpha = N × (Pfa^(-1/N) - 1)
 * Avec Pfa = 1e-4 et N = 32 : alpha ≈ 4.6 (en linéaire), soit ~13 dB.
 */

struct CfarConfig {
  int guard_cells;   /* nombre de bins de garde de chaque côté du CUT */
  int ref_cells;     /* nombre de bins de référence de chaque côté */
  float alpha_db;    /* seuil en dB au-dessus de la moyenne locale */
};

struct ProcessingConfig {
  CfarConfig cfar;
  float max_display_freq_hz;
  float mains_freq_hz;
  int mains_harmonics;
  float mains_notch_width_hz;
  float mains_cfar_extra_db;
  float spectrum_avg_alpha;
  int peak_median_len;
  float kalman_q_accel;
  float kalman_r_meas;
  float kalman_miss_decay;
  float kalman_speed_floor_ms;
  int kalman_miss_reset_count;
  bool adaptive_noise_enable;
  float noise_floor_rise_alpha;
  float noise_floor_fall_alpha;
  float noise_floor_offset_db;
  bool notch_enable;
  float notch_center_hz;
  float notch_width_hz;
  float notch_depth_db;
  int top_n_tracks;
  float track_gate_hz;
  bool calib_enable_noise_profile;
  float calib_target_rms;
  float calib_noise_offset_db;
};

static std::mutex g_proc_mtx;
static ProcessingConfig g_proc = {
  {4, 16, 12.0f}, /* CFAR */
  MAX_DISPLAY_FREQ,
  50.0f,
  8,
  8.0f,
  8.0f,
  0.25f,
  5,
  12.0f,
  0.25f,
  0.90f,
  0.05f,
  10,
  true,
  0.01f,
  0.20f,
  8.0f,
  false,
  575.0f,
  80.0f,
  18.0f,
  3,
  80.0f,
  true,
  600.0f,
  6.0f
};

static const int MAX_TRACKS = 5;

struct TrackInfo {
  float freq_hz = 0.0f;
  float snr_db = 0.0f;
  int age = 0;
  float jitter_hz = 0.0f;
  float dropout_ratio = 1.0f;
  float snr_stability = 0.0f;
};

struct AutoCalibState {
  bool active = false;
  bool done = false;
  int frames_done = 0;
  int frames_target = 80;
  float dc_mean_acc = 0.0f;
  float rms_acc = 0.0f;
  int accum_count = 0;
  float dc_offset = 0.0f;
  float gain_scale = 1.0f;
  std::vector<float> noise_profile_db;
};

static ProcessingConfig get_processing_config()
{
  std::lock_guard<std::mutex> lk(g_proc_mtx);
  return g_proc;
}

static void set_processing_config(const ProcessingConfig &cfg)
{
  std::lock_guard<std::mutex> lk(g_proc_mtx);
  g_proc = cfg;
}

static bool is_near_mains_bin(int bin, float freq_res_hz,
                              float mains_freq_hz,
                              int mains_harmonics,
                              float mains_notch_width_hz)
{
  if (freq_res_hz <= 0.0f) return false;
  float f = bin * freq_res_hz;
  for (int h = 1; h <= mains_harmonics; h++) {
    float fm = mains_freq_hz * h;
    if (fabsf(f - fm) <= mains_notch_width_hz) return true;
  }
  return false;
}

static int median_bin(const std::deque<int> &hist)
{
  if (hist.empty()) return -1;
  std::vector<int> tmp(hist.begin(), hist.end());
  std::sort(tmp.begin(), tmp.end());
  return tmp[tmp.size() / 2];
}

/**
 * @brief  Applique le détecteur CA-CFAR sur un spectre FFT en dB.
 *
 * Pour chaque bin dans la plage [start_bin, end_bin), calcule le seuil
 * CFAR local et stocke les résultats dans les tableaux de sortie.
 *
 * @param  spectrum_db   Spectre FFT en dB (N/2 bins)
 * @param  n_bins        Nombre de bins dans le spectre
 * @param  cfg           Configuration CFAR (guard, ref, alpha)
 * @param  threshold_out Tableau de sortie pour le seuil CFAR (N/2 valeurs)
 * @param  detections    Tableau de sortie booléen (1=détection, 0=rien)
 * @param  start_bin     Premier bin à analyser
 * @param  end_bin       Dernier bin + 1
 */
static void cfar_detect(const float *spectrum_db, int n_bins,
                         const CfarConfig &cfg,
                         float *threshold_out, uint8_t *detections,
                         int start_bin, int end_bin)
{
  if (n_bins <= 0) return;

  /* Initialiser les sorties à zéro */
  for (int i = 0; i < n_bins; i++) {
    threshold_out[i] = 0.0f;
    detections[i] = 0;
  }

  int total_side = cfg.guard_cells + cfg.ref_cells;

  for (int cut = start_bin; cut < end_bin; cut++)
  {
    /* Accumuler les cellules de référence des deux côtés */
    float sum = 0.0f;
    int count = 0;

    /* Côté gauche (bins < CUT, en dehors de la zone de garde) */
    for (int j = cut - total_side; j < cut - cfg.guard_cells; j++)
    {
      if (j >= 0 && j < n_bins)
      {
        sum += spectrum_db[j];
        count++;
      }
    }

    /* Côté droit (bins > CUT, en dehors de la zone de garde) */
    for (int j = cut + cfg.guard_cells + 1; j <= cut + total_side; j++)
    {
      if (j >= 0 && j < n_bins)
      {
        sum += spectrum_db[j];
        count++;
      }
    }

    /* Seuil CFAR = moyenne locale + alpha (en dB) */
    float noise_avg = (count > 0) ? sum / count : -80.0f;
    float threshold = noise_avg + cfg.alpha_db;
    threshold_out[cut] = threshold;

    /* Décision de détection */
    if (spectrum_db[cut] > threshold)
      detections[cut] = 1;
  }
}

static inline bool is_in_notch_band(int bin, float freq_res_hz,
                                    float center_hz, float width_hz)
{
  if (freq_res_hz <= 0.0f || width_hz <= 0.0f) return false;
  float f = bin * freq_res_hz;
  float half_w = width_hz * 0.5f;
  return (f >= (center_hz - half_w) && f <= (center_hz + half_w));
}

/* ═══════════════════════════════════════════════════════════════════════════
 * Filtre de Kalman 1D pour le suivi de vitesse
 * ═══════════════════════════════════════════════════════════════════════════
 *
 * Le filtre de Kalman est un estimateur optimal récursif qui combine :
 *   1. Un modèle de prédiction (le mouvement de la cible)
 *   2. Une mesure bruitée (la fréquence Doppler FFT)
 *
 * Modèle d'état (1D simplifié) :
 *   État x = [vitesse, accélération]
 *   Transition : x(k+1) = F × x(k) + bruit_processus
 *   Mesure : z(k) = H × x(k) + bruit_mesure
 *
 * Avec :
 *   F = [[1, dt], [0, 1]]  (cinématique linéaire)
 *   H = [1, 0]             (on mesure la vitesse directement)
 *   dt = 10.24 ms          (période hop STM32)
 *
 * Le filtre lisse les mesures bruitées et fournit une estimation
 * stable même quand des mesures sont manquantes (perte de cible).
 */

struct KalmanState {
  /* État estimé : [vitesse (m/s), accélération (m/s²)] */
  float x[2];

  /* Matrice de covariance de l'erreur d'estimation (2×2, stockée plate) */
  float P[4];  /* P[0]=P00, P[1]=P01, P[2]=P10, P[3]=P11 */

  /* Variance du bruit de processus (accélération aléatoire).
   * Plus Q est grand, plus le filtre fait confiance aux mesures.
   * Q petit → filtre lisse fortement (lent à réagir). */
  float Q_accel;

  /* Variance du bruit de mesure (incertitude sur la vitesse FFT).
   * Dépend de la résolution FFT et du SNR. */
  float R_meas;

  /* Période entre les mises à jour (secondes) */
  float dt;

  /* Nombre de cycles sans détection (pour la logique de "perte de piste") */
  int miss_count;

  /* Flag : le filtre a-t-il été initialisé avec une première mesure ? */
  int initialized;
};

/**
 * @brief  Initialise le filtre de Kalman avec des paramètres par défaut.
 *
 * @param  kf  Pointeur vers la structure Kalman
 * @param  dt  Période entre les mises à jour (en secondes)
 */
static void kalman_init(KalmanState *kf, float dt)
{
  ProcessingConfig cfg = get_processing_config();
  kf->x[0] = 0.0f;   /* vitesse initiale = 0 */
  kf->x[1] = 0.0f;   /* accélération initiale = 0 */
  kf->P[0] = 100.0f;  /* covariance initiale élevée (on ne sait rien) */
  kf->P[1] = 0.0f;
  kf->P[2] = 0.0f;
  kf->P[3] = 100.0f;
  kf->Q_accel = cfg.kalman_q_accel;  /* moins de lissage, plus réactif */
  kf->R_meas = cfg.kalman_r_meas;
  kf->dt = dt;
  kf->miss_count = 0;
  kf->initialized = 0;
}

/**
 * @brief  Étape de prédiction du filtre de Kalman.
 *
 * Propage l'état et la covariance selon le modèle cinématique :
 *   x_pred = F × x
 *   P_pred = F × P × F^T + Q
 *
 * @param  kf  Pointeur vers la structure Kalman
 */
static void kalman_predict(KalmanState *kf)
{
  float dt = kf->dt;

  /* Prédiction de l'état : vitesse += accélération × dt */
  float x0_pred = kf->x[0] + kf->x[1] * dt;
  float x1_pred = kf->x[1];  /* accélération supposée constante */

  /* Prédiction de la covariance : P_pred = F·P·F^T + Q
   * F = [[1, dt], [0, 1]]
   * Q = [[dt^4/4, dt^3/2], [dt^3/2, dt^2]] × Q_accel
   *     (modèle de bruit d'accélération constante par morceaux) */
  float p00 = kf->P[0] + dt * (kf->P[2] + kf->P[1]) + dt * dt * kf->P[3];
  float p01 = kf->P[1] + dt * kf->P[3];
  float p10 = kf->P[2] + dt * kf->P[3];
  float p11 = kf->P[3];

  /* Ajout du bruit de processus Q */
  float dt2 = dt * dt;
  float dt3 = dt2 * dt;
  float dt4 = dt3 * dt;
  float q = kf->Q_accel;
  p00 += dt4 / 4.0f * q;
  p01 += dt3 / 2.0f * q;
  p10 += dt3 / 2.0f * q;
  p11 += dt2 * q;

  kf->x[0] = x0_pred;
  kf->x[1] = x1_pred;
  if (kf->x[0] < 0.0f) kf->x[0] = 0.0f;
  kf->P[0] = p00;
  kf->P[1] = p01;
  kf->P[2] = p10;
  kf->P[3] = p11;
}

/**
 * @brief  Étape de mise à jour (correction) du filtre de Kalman.
 *
 * Corrige l'état prédit en utilisant la mesure :
 *   K = P_pred × H^T × (H × P_pred × H^T + R)^{-1}
 *   x = x_pred + K × (z - H × x_pred)
 *   P = (I - K × H) × P_pred
 *
 * @param  kf           Pointeur vers la structure Kalman
 * @param  measurement  Vitesse mesurée (m/s)
 */
static void kalman_update(KalmanState *kf, float measurement)
{
  if (!kf->initialized)
  {
    /* Première mesure : initialiser directement l'état */
    kf->x[0] = measurement;
    kf->x[1] = 0.0f;
    kf->P[0] = kf->R_meas;  /* incertitude = celle de la mesure */
    kf->P[1] = 0.0f;
    kf->P[2] = 0.0f;
    kf->P[3] = 10.0f;
    kf->initialized = 1;
    kf->miss_count = 0;
    return;
  }

  /* Innovation (résidu) : différence mesure - prédiction.
   * H = [1, 0] donc z_pred = x[0]. */
  float y = measurement - kf->x[0];

  /* Covariance de l'innovation : S = H × P × H^T + R = P[0] + R */
  float S = kf->P[0] + kf->R_meas;

  /* Gain de Kalman : K = P × H^T / S = [P[0]/S, P[2]/S] */
  float K0 = kf->P[0] / S;
  float K1 = kf->P[2] / S;

  /* Correction de l'état */
  kf->x[0] += K0 * y;
  kf->x[1] += K1 * y;
  if (kf->x[0] < 0.0f) kf->x[0] = 0.0f;

  /* Correction de la covariance : P = (I - K×H) × P */
  float p00 = (1.0f - K0) * kf->P[0];
  float p01 = (1.0f - K0) * kf->P[1];
  float p10 = -K1 * kf->P[0] + kf->P[2];
  float p11 = -K1 * kf->P[1] + kf->P[3];

  kf->P[0] = p00; kf->P[1] = p01;
  kf->P[2] = p10; kf->P[3] = p11;

  kf->miss_count = 0;
}

/* ═══════════════════════════════════════════════════════════════════════════
 * Données partagées entre le thread UDP et le thread GUI
 * ═══════════════════════════════════════════════════════════════════════════ */

struct SharedData {
  std::mutex mtx;

  /* Échantillons ADC bruts de la fenêtre glissante (4096 × uint16) */
  std::vector<uint16_t> adc_raw;

  /* Spectre FFT en dB (g_fft_size / 2 bins) */
  std::vector<float> fft_db;

  /* Seuil CFAR en dB (même taille que fft_db) */
  std::vector<float> cfar_threshold;

  /* Détections CFAR (1 = détection, 0 = rien) */
  std::vector<uint8_t> cfar_detections;

  /* Résultats de détection */
  float peak_freq     = 0.0f;   /* fréquence du pic CFAR principal (Hz) */
  float peak_snr      = 0.0f;   /* SNR du pic (dB) */
  float raw_speed_ms  = 0.0f;   /* vitesse brute (m/s) */
  float raw_speed_kmh = 0.0f;   /* vitesse brute (km/h) */
  float kal_speed_ms  = 0.0f;   /* vitesse filtrée Kalman (m/s) */
  float kal_speed_kmh = 0.0f;   /* vitesse filtrée Kalman (km/h) */
  float kal_accel     = 0.0f;   /* accélération estimée (m/s²) */
  bool  detected      = false;  /* cible CFAR détectée ? */
  int   cfar_n_det    = 0;      /* nombre de bins CFAR détectés */
  float noise_floor_db = -80.0f;
  std::array<TrackInfo, MAX_TRACKS> tracks{};
  int track_count = 0;
  float calib_dc_offset = 0.0f;
  float calib_gain = 1.0f;
  bool calib_active = false;
  bool calib_done = false;

  /* Spectrogramme : matrice SPECTRO_ROWS × n_display_bins */
  std::vector<float> spectrogram;
  int spectro_write_idx = 0;
  int n_display_bins = 0;

  /* Statistiques */
  float fps = 0.0f;
  bool  connected = false;
  int   frame_count = 0;   /* nombre de hops traités */
  uint32_t last_seq = 0;
  int   missed_frames = 0; /* nombre de hops incomplets/perdus */
};

static SharedData g_shared;
static std::atomic<bool> g_running(true);
static GLuint g_spectro_tex = 0;

/* Signal pour changement dynamique de taille FFT (GUI → thread UDP) */
static std::atomic<int> g_requested_fft_size(0);
static std::atomic<bool> g_requested_proc_refresh(false);
static std::atomic<bool> g_record_enable(false);
static std::atomic<bool> g_replay_enable(false);
static std::atomic<bool> g_replay_reload(false);
static std::atomic<bool> g_calib_start(false);
static const char *CAPTURE_FILE_PATH = "radar_capture_hops.bin";

/* Tailles FFT disponibles (puissances de 2) */
static const int FFT_SIZES[] = { 1024, 2048, 4096, 8192, 16384, 32768, 65536 };
static const int NUM_FFT_SIZES = sizeof(FFT_SIZES) / sizeof(FFT_SIZES[0]);

/* ═══════════════════════════════════════════════════════════════════════════
 * Thread de réception UDP + traitement du signal
 * ═══════════════════════════════════════════════════════════════════════════ */

static void udp_thread_func()
{
  /* === Initialisation Winsock === */
  WSADATA wsa;
  WSAStartup(MAKEWORD(2, 2), &wsa);

  /* Créer un socket UDP */
  SOCKET sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
  if (sock == INVALID_SOCKET)
  {
    fprintf(stderr, "[!] Échec création socket UDP\n");
    return;
  }

  /* Permettre le broadcast et la réutilisation du port */
  BOOL opt = TRUE;
  setsockopt(sock, SOL_SOCKET, SO_BROADCAST, (char *)&opt, sizeof(opt));
  setsockopt(sock, SOL_SOCKET, SO_REUSEADDR, (char *)&opt, sizeof(opt));

  /* Agrandir le buffer de réception (256 ko) pour éviter les pertes */
  int rcv_buf_size = 256 * 1024;
  setsockopt(sock, SOL_SOCKET, SO_RCVBUF, (char *)&rcv_buf_size, sizeof(rcv_buf_size));

  /* Timeout de réception (500 ms) pour pouvoir vérifier g_running */
  DWORD timeout_ms = 500;
  setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, (char *)&timeout_ms, sizeof(timeout_ms));

  /* Bind sur le port UDP 5000, toutes les interfaces */
  struct sockaddr_in addr = {};
  addr.sin_family = AF_INET;
  addr.sin_port = htons(UDP_PORT);
  addr.sin_addr.s_addr = INADDR_ANY;

  if (bind(sock, (struct sockaddr *)&addr, sizeof(addr)) == SOCKET_ERROR)
  {
    fprintf(stderr, "[!] Échec bind UDP port %d (erreur %d)\n",
            UDP_PORT, WSAGetLastError());
    closesocket(sock);
    WSACleanup();
    return;
  }

  printf("[*] Écoute UDP sur port %d...\n", UDP_PORT);

  /* === Initialisation FFT (KissFFT) === */
  kissfft::kiss_fft_state fft_state = kissfft::make_state(g_fft_size, 0);
  int half_fft = g_fft_size / 2;
  std::vector<kissfft::cpx> fft_out(half_fft + 1);
  /* Fenêtre de Hann taillée pour la fenêtre glissante STM32 (4096 pts),
   * PAS pour la FFT complète (8192 pts). Sinon on applique seulement
   * la première moitié de la fenêtre (rampe 0→1 au lieu de cloche 0→1→0)
   * ce qui cause une fuite spectrale massive. */
  std::vector<float> hann(STM32_FRAME_SIZE);
  std::vector<float> fft_input(g_fft_size, 0.0f);
  std::vector<float> fft_db_local(half_fft);
  std::vector<float> fft_db_smooth(half_fft, -120.0f);
  std::vector<float> cfar_thresh(half_fft);
  std::vector<uint8_t> cfar_det(half_fft);
  std::deque<int> peak_bin_hist;
  float adaptive_noise_floor_db = -80.0f;

  /* Pré-calcul de la fenêtre de Hann sur la fenêtre glissante (4096 points) */
  for (int i = 0; i < STM32_FRAME_SIZE; i++)
    hann[i] = 0.5f * (1.0f - cosf(2.0f * (float)M_PI * i / (STM32_FRAME_SIZE - 1)));

  /* Buffer de réassemblage des fragments UDP (un hop) */
  std::vector<uint16_t> frame_buf(STM32_HOP_SIZE);
  /* Fenêtre glissante pour la FFT locale (4096 échantillons) */
  std::vector<uint16_t> rolling_adc(STM32_FRAME_SIZE, 0);
  uint32_t current_seq = 0xFFFFFFFF;
  int expected_fragments = 0;

  /* Bitmap de fragments reçus (max 16 fragments, suffisant pour 2 en hop).
   * On utilise un bitmap au lieu d'un simple compteur pour détecter les doublons :
   * si un fragment arrive 2 fois, on ne le compte qu'une seule fois. */
  uint32_t frag_bitmap = 0;
  int unique_fragments = 0;

  /* Compteurs de debug (affichés toutes les 2 secondes sur stdout) */
  int dbg_frames_ok = 0;
  int dbg_frames_incomplete = 0;
  int dbg_duplicates = 0;
  auto dbg_timer = std::chrono::steady_clock::now();

  /* Compteur FPS */
  int fps_counter = 0;
  auto fps_timer = std::chrono::steady_clock::now();

  /* Filtre de Kalman */
  float dt = (float)STM32_HOP_SIZE / FS;  /* ~10.24 ms */
  KalmanState kalman;
  kalman_init(&kalman, dt);

  /* Initialiser les données partagées */
  {
    ProcessingConfig cfg = get_processing_config();
    int n_disp = (int)(cfg.max_display_freq_hz / (FS / 2.0f) * half_fft);
    if (n_disp > half_fft) n_disp = half_fft;

    std::lock_guard<std::mutex> lk(g_shared.mtx);
    g_shared.adc_raw.resize(STM32_FRAME_SIZE, 0);
    g_shared.fft_db.resize(half_fft, -80.0f);
    g_shared.cfar_threshold.resize(half_fft, -80.0f);
    g_shared.cfar_detections.resize(half_fft, 0);
    g_shared.n_display_bins = n_disp;
    g_shared.spectrogram.resize(SPECTRO_ROWS * n_disp, -80.0f);
  }

  /* === Boucle de réception === */
  uint8_t recv_buf[2048];
  uint32_t timeout_streak = 0;
  bool first_packet_logged = false;
  sockaddr_in src_addr{};
  int src_addr_len = sizeof(src_addr);

  while (g_running)
  {
    int len = recvfrom(sock, (char *)recv_buf, sizeof(recv_buf), 0,
                      (sockaddr *)&src_addr, &src_addr_len);
    if (len <= 0) {
      timeout_streak++;
      if (timeout_streak >= 10)  /* ~5s sans paquet (timeout=500ms) */
      {
        printf("[UDP DEBUG] aucun paquet recu depuis ~%u s (port=%d)\n",
               (unsigned)(timeout_streak / 2), UDP_PORT);
        fflush(stdout);
        timeout_streak = 0;
      }
      continue;  /* timeout ou erreur */
    }
    timeout_streak = 0;

    /* Le paquet doit avoir au moins 12 octets (en-tête de fragment) */
    if (len < 12) continue;

    /* Décoder l'en-tête de fragment (12 octets, little-endian) */
    uint32_t seq = recv_buf[0] | (recv_buf[1] << 8) |
                   (recv_buf[2] << 16) | (recv_buf[3] << 24);
    uint16_t frag_id = recv_buf[4] | (recv_buf[5] << 8);
    uint16_t frag_count = recv_buf[6] | (recv_buf[7] << 8);
    uint16_t offset = recv_buf[8] | (recv_buf[9] << 8);
    uint16_t payload_len = recv_buf[10] | (recv_buf[11] << 8);

    if (!first_packet_logged) {
      printf("[UDP DEBUG] premier paquet recu: src=%s seq=%u frag=%u/%u len=%u\n",
             inet_ntoa(src_addr.sin_addr), seq, (unsigned)frag_id, (unsigned)frag_count, (unsigned)payload_len);
      fflush(stdout);
      first_packet_logged = true;
    }

    /* Vérifications de cohérence */
    if (frag_count == 0 || frag_id >= frag_count) continue;
    if (offset + payload_len > STM32_HOP_SIZE * 2) continue;
    if (12 + payload_len > len) continue;

    /* Nouveau numéro de séquence → nouveau hop */
    if (seq != current_seq)
    {
      /* Si on avait un hop précédent incomplet, le compter comme perdu */
      if (current_seq != 0xFFFFFFFF && unique_fragments < expected_fragments)
      {
        dbg_frames_incomplete++;
        std::lock_guard<std::mutex> lk(g_shared.mtx);
        g_shared.missed_frames++;
      }
      current_seq = seq;
      frag_bitmap = 0;
      unique_fragments = 0;
      expected_fragments = frag_count;

      /* Remettre le buffer hop à zéro pour éviter les données résiduelles */
      memset(frame_buf.data(), 0, STM32_HOP_SIZE * sizeof(uint16_t));
    }

    /* Vérifier si ce fragment a déjà été reçu (doublon) */
    if (frag_id < 32 && (frag_bitmap & (1u << frag_id)))
    {
      dbg_duplicates++;
      continue;  /* ignorer le doublon */
    }

    /* Marquer ce fragment comme reçu dans le bitmap */
    if (frag_id < 32)
    {
      frag_bitmap |= (1u << frag_id);
      unique_fragments++;
    }

    /* Copier les données du fragment dans le buffer de réassemblage */
    memcpy(((uint8_t *)frame_buf.data()) + offset, recv_buf + 12, payload_len);

    /* Marquer la connexion comme active */
    {
      std::lock_guard<std::mutex> lk(g_shared.mtx);
      g_shared.connected = true;
      g_shared.last_seq = seq;
    }

    /* === Debug : afficher les stats de réception toutes les 2 secondes === */
    {
      auto now = std::chrono::steady_clock::now();
      float elapsed = std::chrono::duration<float>(now - dbg_timer).count();
      if (elapsed >= 2.0f)
      {
        printf("[UDP DEBUG] ok=%d incomplete=%d duplicates=%d | "
               "last_seq=%u frag_bitmap=0x%X (%d/%d)\n",
               dbg_frames_ok, dbg_frames_incomplete, dbg_duplicates,
               current_seq, frag_bitmap, unique_fragments, expected_fragments);
        fflush(stdout);
        dbg_frames_ok = 0;
        dbg_frames_incomplete = 0;
        dbg_duplicates = 0;
        dbg_timer = now;
      }
    }

    /* Tous les fragments UNIQUES reçus → hop complet, lancer le traitement */
    if (unique_fragments >= expected_fragments)
    {
      dbg_frames_ok++;
      ProcessingConfig cfg = get_processing_config();
      kalman.Q_accel = cfg.kalman_q_accel;
      kalman.R_meas = cfg.kalman_r_meas;

      /* === Vérification debug : compter les échantillons à zéro du hop === */
      {
        int zero_count = 0;
        for (int i = 0; i < STM32_HOP_SIZE; i++)
          if (frame_buf[i] == 0) zero_count++;
        if (zero_count > STM32_HOP_SIZE / 10)  /* > 10% de zéros = suspect */
        {
          printf("[UDP DEBUG] ATTENTION: %d/%d echantillons a zero dans hop seq=%u bitmap=0x%X\n",
                 zero_count, STM32_HOP_SIZE, current_seq, frag_bitmap);
          fflush(stdout);
        }
      }

      /* === Mettre à jour la fenêtre glissante 4096 === */
      memmove(rolling_adc.data(),
              rolling_adc.data() + STM32_HOP_SIZE,
              (STM32_FRAME_SIZE - STM32_HOP_SIZE) * sizeof(uint16_t));
      memcpy(rolling_adc.data() + (STM32_FRAME_SIZE - STM32_HOP_SIZE),
             frame_buf.data(),
             STM32_HOP_SIZE * sizeof(uint16_t));

      /* === Préparer les données pour la FFT ===
       * Les échantillons sont en uint16 bruts (0-4095 pour 12 bits ADC),
       * envoyés par le STM32 AVANT DC removal et fenêtrage, par hops.
       * On reconstruit une fenêtre glissante de 4096 puis on traite ici :
       * DC removal + Hann + FFT (taille g_fft_size configurable). */

      int fft_input_samples = std::min(STM32_FRAME_SIZE, g_fft_size);

      /* Conversion uint16 → float + DC removal */
      double sum = 0.0;
      for (int i = 0; i < fft_input_samples; i++)
        sum += rolling_adc[i];
      float mean = (float)(sum / std::max(1, fft_input_samples));

      /* Remplir le buffer FFT depuis rolling_adc, puis zero-padding */
      for (int i = 0; i < fft_input_samples; i++)
        fft_input[i] = ((float)rolling_adc[i] - mean) * hann[i];
      for (int i = fft_input_samples; i < g_fft_size; i++)
        fft_input[i] = 0.0f;

      /* FFT */
      kissfft::rfft(fft_state, fft_input.data(), fft_out.data());

      /* Magnitude en dB */
      for (int i = 0; i < half_fft; i++)
      {
        float mag = sqrtf(fft_out[i].r * fft_out[i].r +
                          fft_out[i].i * fft_out[i].i);
        fft_db_local[i] = 20.0f * log10f(mag + 1e-12f);
        if (i < (int)fft_db_smooth.size()) {
          if (fft_db_smooth[i] < -100.0f)
            fft_db_smooth[i] = fft_db_local[i];
          else
            fft_db_smooth[i] = (1.0f - cfg.spectrum_avg_alpha) * fft_db_smooth[i]
                             + cfg.spectrum_avg_alpha * fft_db_local[i];
        }
      }

      float freq_res = FS / g_fft_size;

      /* === CFAR === */
      int min_bin = 2;  /* ignorer DC */
      int max_bin = (int)(cfg.max_display_freq_hz / (FS / g_fft_size));
      if (max_bin > half_fft) max_bin = half_fft;

      if (cfg.notch_enable) {
        for (int i = min_bin; i < max_bin; i++) {
          if (is_in_notch_band(i, freq_res, cfg.notch_center_hz, cfg.notch_width_hz)) {
            fft_db_local[i] -= cfg.notch_depth_db;
            fft_db_smooth[i] -= cfg.notch_depth_db;
          }
        }
      }

      cfar_detect(fft_db_smooth.data(), half_fft, cfg.cfar,
                  cfar_thresh.data(), cfar_det.data(),
                  min_bin, max_bin);

      float noise_inst = -80.0f;
      {
        std::vector<float> noise_bins;
        noise_bins.reserve(std::max(0, max_bin - min_bin));
        for (int i = min_bin; i < max_bin; i++) {
          if (cfar_det[i]) continue;
          if (cfg.notch_enable && is_in_notch_band(i, freq_res, cfg.notch_center_hz, cfg.notch_width_hz)) continue;
          noise_bins.push_back(fft_db_smooth[i]);
        }
        if (!noise_bins.empty()) {
          size_t mid = noise_bins.size() / 2;
          std::nth_element(noise_bins.begin(), noise_bins.begin() + mid, noise_bins.end());
          noise_inst = noise_bins[mid];
        }
      }

      if (cfg.adaptive_noise_enable) {
        float alpha = (noise_inst > adaptive_noise_floor_db)
                        ? cfg.noise_floor_rise_alpha
                        : cfg.noise_floor_fall_alpha;
        alpha = std::clamp(alpha, 0.001f, 1.0f);
        adaptive_noise_floor_db += alpha * (noise_inst - adaptive_noise_floor_db);
      } else {
        adaptive_noise_floor_db = noise_inst;
      }

      for (int i = min_bin; i < max_bin; i++) {
        if (is_near_mains_bin(i, freq_res,
                              cfg.mains_freq_hz,
                              cfg.mains_harmonics,
                              cfg.mains_notch_width_hz)) {
          cfar_det[i] = 0;
          cfar_thresh[i] += cfg.mains_cfar_extra_db;
        }
        if (cfg.adaptive_noise_enable) {
          float floor_thr = adaptive_noise_floor_db + cfg.noise_floor_offset_db;
          if (cfar_thresh[i] < floor_thr) cfar_thresh[i] = floor_thr;
          if (fft_db_smooth[i] <= cfar_thresh[i]) cfar_det[i] = 0;
        }
      }

      /* Trouver le pic CFAR principal (bin détecté avec la magnitude max) */
      float best_mag = -1000.0f;
      int best_bin = -1;
      int n_det = 0;
      for (int i = min_bin; i < max_bin; i++)
      {
        if (cfar_det[i])
        {
          n_det++;
          if (fft_db_smooth[i] > best_mag)
          {
            best_mag = fft_db_smooth[i];
            best_bin = i;
          }
        }
      }

      if (best_bin >= 0) {
        peak_bin_hist.push_back(best_bin);
        if ((int)peak_bin_hist.size() > cfg.peak_median_len) peak_bin_hist.pop_front();
        best_bin = median_bin(peak_bin_hist);
      } else {
        peak_bin_hist.clear();
      }

      /* Résolution fréquentielle : Δf = FS / g_fft_size */
      float peak_freq = (best_bin >= 0) ? best_bin * freq_res : 0.0f;
      float raw_speed = peak_freq / DOPPLER_SCALE;
      bool detected = (best_bin >= 0);
      if (raw_speed < 0.0f) raw_speed = 0.0f;

      /* SNR du pic par rapport au seuil CFAR */
      float peak_snr = (best_bin >= 0)
        ? fft_db_smooth[best_bin] - cfar_thresh[best_bin]
        : 0.0f;

      /* === Filtre de Kalman === */
      kalman_predict(&kalman);
      if (detected)
      {
        kalman_update(&kalman, raw_speed);
      }
      else
      {
        kalman.miss_count++;
        kalman.x[0] *= cfg.kalman_miss_decay;
        if (kalman.x[0] < cfg.kalman_speed_floor_ms) kalman.x[0] = 0.0f;
        /* Après N cycles sans détection, réinitialiser le filtre */
        if (kalman.miss_count > cfg.kalman_miss_reset_count)
        {
          kalman.initialized = 0;
          kalman.x[0] = 0.0f;
          kalman.x[1] = 0.0f;
        }
      }

      /* === Mise à jour des données partagées === */
      {
        std::lock_guard<std::mutex> lk(g_shared.mtx);
        memcpy(g_shared.adc_raw.data(), rolling_adc.data(),
               STM32_FRAME_SIZE * sizeof(uint16_t));
        g_shared.fft_db = fft_db_local;
        g_shared.cfar_threshold = cfar_thresh;
        g_shared.cfar_detections = cfar_det;

        g_shared.peak_freq     = peak_freq;
        g_shared.peak_snr      = peak_snr;
        g_shared.raw_speed_ms  = raw_speed;
        g_shared.raw_speed_kmh = raw_speed * 3.6f;
        g_shared.kal_speed_ms  = kalman.x[0];
        g_shared.kal_speed_kmh = kalman.x[0] * 3.6f;
        g_shared.kal_accel     = kalman.x[1];
        g_shared.detected      = detected;
        g_shared.cfar_n_det    = n_det;
        g_shared.noise_floor_db = adaptive_noise_floor_db;
        g_shared.frame_count++;

        /* Spectrogramme circulaire */
        int n_disp = g_shared.n_display_bins;
        int row = g_shared.spectro_write_idx % SPECTRO_ROWS;
        for (int i = 0; i < n_disp && i < half_fft; i++)
          g_shared.spectrogram[row * n_disp + i] = fft_db_local[i];
        g_shared.spectro_write_idx++;
      }

      /* FPS */
      fps_counter++;
      auto now = std::chrono::steady_clock::now();
      double elapsed = std::chrono::duration<double>(now - fps_timer).count();
      if (elapsed >= 1.0)
      {
        std::lock_guard<std::mutex> lk(g_shared.mtx);
        g_shared.fps = (float)(fps_counter / elapsed);
        fps_counter = 0;
        fps_timer = now;
      }

      frag_bitmap = 0;
      unique_fragments = 0;
    }

    /* === Changement dynamique de taille FFT (demandé par le GUI) === */
    int req = g_requested_fft_size.exchange(0);
    bool proc_refresh = g_requested_proc_refresh.exchange(false);
    if ((req > 0 && req != g_fft_size) || proc_refresh)
    {
      if (req > 0 && req != g_fft_size) {
        printf("[FFT] Changement de taille : %d -> %d\n", g_fft_size, req);
        fflush(stdout);
      }

      if (req > 0 && req != g_fft_size) g_fft_size = req;
      half_fft = g_fft_size / 2;

      /* Recréer l'état KissFFT */
      fft_state = kissfft::make_state(g_fft_size, 0);

      /* Redimensionner les buffers */
      fft_out.resize(half_fft + 1);
      fft_input.assign(g_fft_size, 0.0f);
      fft_db_local.assign(half_fft, -80.0f);
      fft_db_smooth.assign(half_fft, -120.0f);
      cfar_thresh.assign(half_fft, -80.0f);
      cfar_det.assign(half_fft, 0);
      peak_bin_hist.clear();

      /* Réinitialiser le Kalman (taille FFT ou paramètres modifiés) */
      kalman_init(&kalman, dt);

      /* Mettre à jour les données partagées */
      {
        ProcessingConfig cfg2 = get_processing_config();
        int n_disp = (int)(cfg2.max_display_freq_hz / (FS / 2.0f) * half_fft);
        if (n_disp > half_fft) n_disp = half_fft;

        std::lock_guard<std::mutex> lk(g_shared.mtx);
        g_shared.fft_db.assign(half_fft, -80.0f);
        g_shared.cfar_threshold.assign(half_fft, -80.0f);
        g_shared.cfar_detections.assign(half_fft, 0);
        g_shared.n_display_bins = n_disp;
        g_shared.spectrogram.assign(SPECTRO_ROWS * n_disp, -80.0f);
        g_shared.spectro_write_idx = 0;
      }
    }
  }

  closesocket(sock);
  WSACleanup();
}

/* ═══════════════════════════════════════════════════════════════════════════
 * Utilitaires graphiques ImGui
 * ═══════════════════════════════════════════════════════════════════════════ */

/**
 * @brief  Dessine un graphe avec ImGui DrawList.
 */
static void DrawGraph(const char *label, const float *data, int count,
                      float x_min, float x_max, float y_min, float y_max,
                      ImVec2 size, ImU32 color,
                      const float *overlay = nullptr, ImU32 overlay_color = 0,
                      float vline_x = -1.0f, ImU32 vline_color = 0,
                      bool show_freq_axis = false,
                      bool show_cursor_readout = false)
{
  ImGui::BeginChild(label, size, true, ImGuiWindowFlags_NoScrollbar);
  ImDrawList *dl = ImGui::GetWindowDrawList();
  ImVec2 p = ImGui::GetCursorScreenPos();
  ImVec2 region = ImGui::GetContentRegionAvail();
  float w = region.x, h = region.y;

  /* Fond sombre */
  dl->AddRectFilled(p, ImVec2(p.x + w, p.y + h), IM_COL32(15, 15, 20, 255));

  /* Grille */
  for (int i = 1; i < 4; i++) {
    float yy = p.y + h * i / 4.0f;
    dl->AddLine(ImVec2(p.x, yy), ImVec2(p.x + w, yy), IM_COL32(50, 50, 50, 100));
  }
  for (int i = 1; i < 8; i++) {
    float xx = p.x + w * i / 8.0f;
    dl->AddLine(ImVec2(xx, p.y), ImVec2(xx, p.y + h), IM_COL32(50, 50, 50, 100));
  }

  /* Courbe principale */
  if (count > 1) {
    for (int i = 0; i < count - 1; i++) {
      float x0 = p.x + (float)i / (count - 1) * w;
      float x1 = p.x + (float)(i + 1) / (count - 1) * w;
      float v0 = 1.0f - std::clamp((data[i] - y_min) / (y_max - y_min), 0.0f, 1.0f);
      float v1 = 1.0f - std::clamp((data[i+1] - y_min) / (y_max - y_min), 0.0f, 1.0f);
      dl->AddLine(ImVec2(x0, p.y + v0 * h), ImVec2(x1, p.y + v1 * h), color, 1.5f);
    }
  }

  /* Courbe overlay (seuil CFAR) */
  if (overlay && count > 1 && overlay_color) {
    for (int i = 0; i < count - 1; i++) {
      float x0 = p.x + (float)i / (count - 1) * w;
      float x1 = p.x + (float)(i + 1) / (count - 1) * w;
      float v0 = 1.0f - std::clamp((overlay[i] - y_min) / (y_max - y_min), 0.0f, 1.0f);
      float v1 = 1.0f - std::clamp((overlay[i+1] - y_min) / (y_max - y_min), 0.0f, 1.0f);
      dl->AddLine(ImVec2(x0, p.y + v0 * h), ImVec2(x1, p.y + v1 * h), overlay_color, 1.0f);
    }
  }

  /* Ligne verticale (marqueur de pic) */
  if (vline_x >= x_min && vline_x <= x_max && vline_color) {
    float xp = p.x + (vline_x - x_min) / (x_max - x_min) * w;
    dl->AddLine(ImVec2(xp, p.y), ImVec2(xp, p.y + h), vline_color, 1.0f);
  }

  /* Label */
  dl->AddText(ImVec2(p.x + 4, p.y + 2), IM_COL32(200, 200, 200, 200), label);

  if (show_freq_axis) {
    ImGui::SetWindowFontScale(1.25f);
    char b0[24], b1[24], b2[24];
    snprintf(b0, sizeof(b0), "%.0f Hz", x_min);
    snprintf(b1, sizeof(b1), "%.0f Hz", (x_min + x_max) * 0.5f);
    snprintf(b2, sizeof(b2), "%.0f Hz", x_max);
    dl->AddText(ImVec2(p.x + 4, p.y + h - 18), IM_COL32(180, 180, 180, 200), b0);
    dl->AddText(ImVec2(p.x + w * 0.5f - 24, p.y + h - 18), IM_COL32(180, 180, 180, 200), b1);
    dl->AddText(ImVec2(p.x + w - 56, p.y + h - 18), IM_COL32(180, 180, 180, 200), b2);
    ImGui::SetWindowFontScale(1.0f);
  }

  if (show_cursor_readout && count > 1 && ImGui::IsWindowHovered()) {
    ImVec2 m = ImGui::GetIO().MousePos;
    float mx = std::clamp(m.x, p.x, p.x + w);
    float my = std::clamp(m.y, p.y, p.y + h);
    float t = (mx - p.x) / (w > 1.0f ? w : 1.0f);
    int idx = (int)std::clamp(t * (count - 1), 0.0f, (float)(count - 1));
    float xv = x_min + t * (x_max - x_min);
    float yv = data[idx];

    dl->AddLine(ImVec2(mx, p.y), ImVec2(mx, p.y + h), IM_COL32(255, 255, 255, 90), 1.0f);
    dl->AddLine(ImVec2(p.x, my), ImVec2(p.x + w, my), IM_COL32(255, 255, 255, 60), 1.0f);

    ImGui::BeginTooltip();
    ImGui::Text("f = %.1f Hz", xv);
    ImGui::Text("A = %.1f dB", yv);
    ImGui::EndTooltip();
  }

  ImGui::EndChild();
}

/**
 * @brief  Conversion dB → couleur RGB (palette inferno).
 */
static void db_to_rgb(float db, float vmin, float vmax, uint8_t *r, uint8_t *g, uint8_t *b)
{
  float t = std::clamp((db - vmin) / (vmax - vmin), 0.0f, 1.0f);
  if (t < 0.25f) {
    float s = t / 0.25f;
    *r = (uint8_t)(s * 80); *g = 0; *b = (uint8_t)(s * 120);
  } else if (t < 0.5f) {
    float s = (t - 0.25f) / 0.25f;
    *r = (uint8_t)(80 + s * 150); *g = (uint8_t)(s * 30); *b = (uint8_t)(120 - s * 80);
  } else if (t < 0.75f) {
    float s = (t - 0.5f) / 0.25f;
    *r = (uint8_t)(230 + s * 25); *g = (uint8_t)(30 + s * 120); *b = (uint8_t)(40 - s * 40);
  } else {
    float s = (t - 0.75f) / 0.25f;
    *r = 255; *g = (uint8_t)(150 + s * 105); *b = (uint8_t)(s * 80);
  }
}

/**
 * @brief  Mise à jour de la texture OpenGL du spectrogramme.
 */
static void update_spectro_texture(const float *spectrogram, int write_idx,
                                    int rows, int cols, float vmin, float vmax)
{
  static std::vector<uint8_t> pixels;
  pixels.resize(rows * cols * 3);

  for (int r = 0; r < rows; r++) {
    /* Waterfall: oldest en haut, plus récent en bas */
    int src_row = (write_idx + r) % rows;
    for (int c = 0; c < cols; c++) {
      int pi = (r * cols + c) * 3;
      db_to_rgb(spectrogram[src_row * cols + c], vmin, vmax,
                &pixels[pi], &pixels[pi+1], &pixels[pi+2]);
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
  glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, cols, rows, 0, GL_RGB, GL_UNSIGNED_BYTE, pixels.data());
}

/* ═══════════════════════════════════════════════════════════════════════════
 * Point d'entrée principal
 * ═══════════════════════════════════════════════════════════════════════════ */

int main(int argc, char **argv)
{
  /* Argument optionnel : taille de la FFT */
  if (argc > 1)
  {
    int fft_arg = atoi(argv[1]);
    /* Vérifier que c'est une puissance de 2 entre 4096 et 65536 */
    if (fft_arg >= 4096 && fft_arg <= 65536 && (fft_arg & (fft_arg - 1)) == 0)
      g_fft_size = fft_arg;
    else
      fprintf(stderr, "[!] FFT size invalide (%d), utilisation de %d\n", fft_arg, g_fft_size);
  }

  float freq_res = FS / g_fft_size;
  float speed_res = freq_res / DOPPLER_SCALE;
  ProcessingConfig boot_cfg = get_processing_config();

  printf("==============================================\n");
  printf("  RADAR DOPPLER 24 GHz - Analyseur PC (C++)\n");
  printf("  UDP port: %d  FFT: %d points\n", UDP_PORT, g_fft_size);
  printf("  Resolution: %.2f Hz = %.3f m/s = %.2f km/h\n",
         freq_res, speed_res, speed_res * 3.6f);
  printf("  CFAR: guard=%d ref=%d alpha=%.1f dB\n",
         boot_cfg.cfar.guard_cells, boot_cfg.cfar.ref_cells, boot_cfg.cfar.alpha_db);
  printf("==============================================\n");

  /* === Initialisation GLFW === */
  if (!glfwInit()) {
    fprintf(stderr, "Échec initialisation GLFW\n");
    return 1;
  }
  glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
  glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
  glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

  GLFWwindow *window = glfwCreateWindow(1600, 1000,
    "Radar Doppler 24.125 GHz - Analyseur PC [CFAR + Kalman]", NULL, NULL);
  if (!window) {
    fprintf(stderr, "Échec création fenêtre GLFW\n");
    glfwTerminate();
    return 1;
  }
  glfwMakeContextCurrent(window);
  glfwSwapInterval(1);  /* V-Sync ON */

  /* === Initialisation Dear ImGui === */
  IMGUI_CHECKVERSION();
  ImGui::CreateContext();
  ImGuiIO &io = ImGui::GetIO();
  io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;

  ImGui::StyleColorsDark();
  ImGuiStyle &style = ImGui::GetStyle();
  style.WindowRounding = 4.0f;
  style.FrameRounding  = 3.0f;
  style.ItemSpacing    = ImVec2(8, 4);
  style.WindowPadding  = ImVec2(8, 8);

  ImGui_ImplGlfw_InitForOpenGL(window, true);
  ImGui_ImplOpenGL3_Init("#version 330");

  /* === Lancement du thread UDP === */
  std::thread udp_thread(udp_thread_func);

  /* === Buffers locaux pour le rendu === */
  int half_fft = g_fft_size / 2;
  std::vector<float> local_fft(half_fft, -80.0f);
  std::vector<float> local_cfar(half_fft, -80.0f);
  std::vector<uint8_t> local_det(half_fft, 0);
  std::vector<uint16_t> local_adc(STM32_FRAME_SIZE, 0);
  std::vector<float> local_spectro;
  int local_spectro_idx = 0;
  int local_n_disp = 0;

  float local_peak_freq = 0, local_peak_snr = 0;
  float local_raw_kmh = 0, local_kal_kmh = 0, local_kal_accel = 0;
  float local_noise_floor_db = -80.0f;
  bool local_detected = false;
  int local_cfar_n = 0;
  int local_frame_count = 0;
  float local_fps = 0;
  bool local_connected = false;

  /* Historique des vitesses pour le graphe temporel */
  std::deque<float> speed_history;
  std::deque<float> speed_raw_history;
  std::deque<float> doppler_trend_hz;
  const int HISTORY_LEN = 300;  /* ~3.1 secondes à ~97.6 hops/s */
  static int trend_seconds = 30;
  static int scope_time_div_ms = 10;
  static bool scope_auto_amplitude = true;
  static float scope_amp_per_div = 100.0f;
  static bool scope_freeze = false;
  static bool show_help_window = false;
  enum LayoutMode { LAYOUT_DEFAULT = 0, LAYOUT_FFT_ONLY, LAYOUT_SCOPE_ONLY, LAYOUT_SPECTRO_ONLY };
  static int layout_mode = LAYOUT_DEFAULT;
  static float top_ratio = 0.30f;
  static float mid_ratio = 0.30f;

  /* === Boucle de rendu principale === */
  while (!glfwWindowShouldClose(window) && g_running)
  {
    glfwPollEvents();

    /* Copie atomique des données partagées */
    {
      std::lock_guard<std::mutex> lk(g_shared.mtx);
      if (!g_shared.fft_db.empty()) local_fft = g_shared.fft_db;
      if (!g_shared.cfar_threshold.empty()) local_cfar = g_shared.cfar_threshold;
      if (!g_shared.cfar_detections.empty()) local_det = g_shared.cfar_detections;
      if (!g_shared.adc_raw.empty()) local_adc = g_shared.adc_raw;
      if (!g_shared.spectrogram.empty()) {
        local_spectro = g_shared.spectrogram;
        local_spectro_idx = g_shared.spectro_write_idx;
        local_n_disp = g_shared.n_display_bins;
      }
      local_peak_freq = g_shared.peak_freq;
      local_peak_snr  = g_shared.peak_snr;
      local_raw_kmh   = g_shared.raw_speed_kmh;
      local_kal_kmh   = g_shared.kal_speed_kmh;
      local_kal_accel = g_shared.kal_accel;
      local_noise_floor_db = g_shared.noise_floor_db;
      local_detected  = g_shared.detected;
      local_cfar_n    = g_shared.cfar_n_det;
      local_frame_count = g_shared.frame_count;
      local_fps       = g_shared.fps;
      local_connected = g_shared.connected;
    }

    /* Redimensionner les buffers locaux si la taille FFT a changé */
    if ((int)local_fft.size() != g_fft_size / 2)
    {
      half_fft = g_fft_size / 2;
      local_fft.assign(half_fft, -80.0f);
      local_cfar.assign(half_fft, -80.0f);
      local_det.assign(half_fft, 0);
    }

    /* Historique des vitesses */
    speed_history.push_back(local_kal_kmh);
    speed_raw_history.push_back(local_raw_kmh);
    doppler_trend_hz.push_back(local_peak_freq);
    if ((int)speed_history.size() > HISTORY_LEN) speed_history.pop_front();
    if ((int)speed_raw_history.size() > HISTORY_LEN) speed_raw_history.pop_front();
    int trend_max_points = (int)std::round(std::max(1.0f, local_fps) * trend_seconds);
    if (trend_max_points < 200) trend_max_points = 200;
    while ((int)doppler_trend_hz.size() > trend_max_points) doppler_trend_hz.pop_front();

    /* Oscilloscope : auto-scale */
    float scope_min = 4095, scope_max = 0;
    for (int i = 0; i < STM32_FRAME_SIZE; i++) {
      if (local_adc[i] < scope_min) scope_min = (float)local_adc[i];
      if (local_adc[i] > scope_max) scope_max = (float)local_adc[i];
    }
    float scope_center = (scope_min + scope_max) / 2.0f;
    float scope_amp_auto = (scope_max - scope_min) / 2.0f;
    if (scope_amp_auto < 50) scope_amp_auto = 50;
    scope_amp_auto *= 1.2f;

    static std::vector<float> scope_history((int)FS, 0.0f);
    static int scope_hist_write = 0;
    static bool scope_hist_filled = false;
    static int last_scope_frame_count = -1;
    static std::vector<float> scope_data((int)FS, 0.0f);
    const int hist_size = (int)scope_history.size();
    const int hops_in_rolling = STM32_FRAME_SIZE / STM32_HOP_SIZE;

    if (!scope_freeze) {
      if (last_scope_frame_count < 0) last_scope_frame_count = local_frame_count;
      int frame_delta = local_frame_count - last_scope_frame_count;
      if (frame_delta < 0) frame_delta = 0;
      int hops_to_append = std::clamp(frame_delta, 1, hops_in_rolling);
      int first_hop = hops_in_rolling - hops_to_append;

      for (int h = first_hop; h < hops_in_rolling; ++h) {
        int base = h * STM32_HOP_SIZE;
        for (int i = 0; i < STM32_HOP_SIZE; ++i) {
          float v = (float)local_adc[base + i] - scope_center;
          scope_history[scope_hist_write] = v;
          scope_hist_write++;
          if (scope_hist_write >= hist_size) {
            scope_hist_write = 0;
            scope_hist_filled = true;
          }
        }
      }
      last_scope_frame_count = local_frame_count;
    }

    int n_scope = (int)(FS * ((float)scope_time_div_ms / 1000.0f));
    if (n_scope < 32) n_scope = 32;
    if (n_scope > hist_size) n_scope = hist_size;
    if (n_scope > STM32_FRAME_SIZE && !scope_hist_filled) n_scope = STM32_FRAME_SIZE;

    for (int i = 0; i < n_scope; ++i) {
      int idx = scope_hist_write - n_scope + i;
      while (idx < 0) idx += hist_size;
      idx %= hist_size;
      scope_data[i] = scope_history[idx];
    }

    const float scope_half_divs = 4.0f;
    float scope_amp = scope_auto_amplitude ? scope_amp_auto : (scope_amp_per_div * scope_half_divs);
    if (scope_amp < 10.0f) scope_amp = 10.0f;

    /* === Nouveau frame ImGui === */
    ImGui_ImplOpenGL3_NewFrame();
    ImGui_ImplGlfw_NewFrame();
    ImGui::NewFrame();

    /* Fenêtre plein écran */
    ImGui::SetNextWindowPos(ImVec2(0, 0));
    ImGui::SetNextWindowSize(io.DisplaySize);
    ImGui::Begin("Radar", NULL,
      ImGuiWindowFlags_MenuBar |
      ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoResize |
      ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoCollapse |
      ImGuiWindowFlags_NoBringToFrontOnFocus);

    ProcessingConfig ui_cfg = get_processing_config();
    bool proc_changed = false;
    if (ImGui::BeginMenuBar()) {
      if (ImGui::BeginMenu("Live Controls")) {
        if (ImGui::BeginMenu("Layout")) {
          if (ImGui::MenuItem("Default", NULL, layout_mode == LAYOUT_DEFAULT)) layout_mode = LAYOUT_DEFAULT;
          if (ImGui::MenuItem("FFT only", NULL, layout_mode == LAYOUT_FFT_ONLY)) layout_mode = LAYOUT_FFT_ONLY;
          if (ImGui::MenuItem("Scope only", NULL, layout_mode == LAYOUT_SCOPE_ONLY)) layout_mode = LAYOUT_SCOPE_ONLY;
          if (ImGui::MenuItem("Spectrogram only", NULL, layout_mode == LAYOUT_SPECTRO_ONLY)) layout_mode = LAYOUT_SPECTRO_ONLY;
          ImGui::Separator();
          ImGui::SliderFloat("Top row ratio", &top_ratio, 0.15f, 0.70f, "%.2f");
          ImGui::SliderFloat("Middle row ratio", &mid_ratio, 0.10f, 0.70f, "%.2f");
          ImGui::EndMenu();
        }
        if (ImGui::BeginMenu("Acquisition & Display")) {
          if (ImGui::BeginMenu("FFT Size")) {
            for (int i = 0; i < NUM_FFT_SIZES; i++) {
              bool selected = (FFT_SIZES[i] == g_fft_size);
              char lbl[32];
              snprintf(lbl, sizeof(lbl), "%d pts", FFT_SIZES[i]);
              if (ImGui::MenuItem(lbl, NULL, selected) && !selected)
                g_requested_fft_size.store(FFT_SIZES[i]);
            }
            ImGui::EndMenu();
          }
          static const int scope_div_options[] = {1, 2, 5, 10, 20, 40, 50, 100, 200, 500, 1000};
          if (ImGui::BeginCombo("Scope ms/div", std::to_string(scope_time_div_ms).c_str())) {
            for (int v : scope_div_options) {
              bool sel = (v == scope_time_div_ms);
              if (ImGui::Selectable(std::to_string(v).c_str(), sel)) scope_time_div_ms = v;
            }
            ImGui::EndCombo();
          }
          ImGui::Checkbox("Scope auto amplitude", &scope_auto_amplitude);
          if (!scope_auto_amplitude) {
            ImGui::SliderFloat("Scope amplitude/div (ADC)", &scope_amp_per_div, 5.0f, 800.0f, "%.1f");
          }
          ImGui::Checkbox("Freeze scope", &scope_freeze);
          proc_changed |= ImGui::SliderFloat("Fmax FFT (Hz)", &ui_cfg.max_display_freq_hz, 1000.0f, FS * 0.5f, "%.0f");
          ImGui::SliderInt("Doppler trend (s)", &trend_seconds, 5, 120);
          ImGui::EndMenu();
        }
        if (ImGui::BeginMenu("CFAR Detection")) {
          proc_changed |= ImGui::SliderInt("CFAR guard", &ui_cfg.cfar.guard_cells, 1, 32);
          proc_changed |= ImGui::SliderInt("CFAR ref", &ui_cfg.cfar.ref_cells, 2, 64);
          proc_changed |= ImGui::SliderFloat("CFAR alpha (dB)", &ui_cfg.cfar.alpha_db, 2.0f, 30.0f, "%.1f");
          ImGui::EndMenu();
        }
        if (ImGui::BeginMenu("Noise / Mains / Notch")) {
          proc_changed |= ImGui::SliderFloat("Mains freq (Hz)", &ui_cfg.mains_freq_hz, 45.0f, 65.0f, "%.1f");
          proc_changed |= ImGui::SliderInt("Mains harmonics", &ui_cfg.mains_harmonics, 1, 20);
          proc_changed |= ImGui::SliderFloat("Mains notch width (Hz)", &ui_cfg.mains_notch_width_hz, 1.0f, 30.0f, "%.1f");
          proc_changed |= ImGui::SliderFloat("Mains CFAR extra (dB)", &ui_cfg.mains_cfar_extra_db, 0.0f, 30.0f, "%.1f");
          proc_changed |= ImGui::Checkbox("Adaptive noise floor", &ui_cfg.adaptive_noise_enable);
          proc_changed |= ImGui::SliderFloat("Noise rise alpha", &ui_cfg.noise_floor_rise_alpha, 0.001f, 1.0f, "%.3f");
          proc_changed |= ImGui::SliderFloat("Noise fall alpha", &ui_cfg.noise_floor_fall_alpha, 0.001f, 1.0f, "%.3f");
          proc_changed |= ImGui::SliderFloat("Noise floor offset (dB)", &ui_cfg.noise_floor_offset_db, 0.0f, 30.0f, "%.1f");
          proc_changed |= ImGui::Checkbox("Enable notch", &ui_cfg.notch_enable);
          proc_changed |= ImGui::SliderFloat("Notch center (Hz)", &ui_cfg.notch_center_hz, 10.0f, FS * 0.5f, "%.1f");
          proc_changed |= ImGui::SliderFloat("Notch width (Hz)", &ui_cfg.notch_width_hz, 1.0f, 1000.0f, "%.1f");
          proc_changed |= ImGui::SliderFloat("Notch depth (dB)", &ui_cfg.notch_depth_db, 0.0f, 80.0f, "%.1f");
          ImGui::EndMenu();
        }
        if (ImGui::BeginMenu("Kalman / Tracking / Calibration")) {
          proc_changed |= ImGui::SliderFloat("Spectrum avg alpha", &ui_cfg.spectrum_avg_alpha, 0.01f, 1.00f, "%.2f");
          proc_changed |= ImGui::SliderInt("Peak median len", &ui_cfg.peak_median_len, 1, 15);
          proc_changed |= ImGui::SliderFloat("Kalman Q accel", &ui_cfg.kalman_q_accel, 0.1f, 50.0f, "%.2f");
          proc_changed |= ImGui::SliderFloat("Kalman R meas", &ui_cfg.kalman_r_meas, 0.01f, 10.0f, "%.2f");
          proc_changed |= ImGui::SliderFloat("Kalman miss decay", &ui_cfg.kalman_miss_decay, 0.50f, 1.00f, "%.2f");
          proc_changed |= ImGui::SliderFloat("Kalman floor (m/s)", &ui_cfg.kalman_speed_floor_ms, 0.0f, 1.0f, "%.2f");
          proc_changed |= ImGui::SliderInt("Kalman reset misses", &ui_cfg.kalman_miss_reset_count, 1, 50);
          proc_changed |= ImGui::SliderInt("Top-N tracks", &ui_cfg.top_n_tracks, 1, MAX_TRACKS);
          proc_changed |= ImGui::SliderFloat("Track gate (Hz)", &ui_cfg.track_gate_hz, 10.0f, 400.0f, "%.1f");
          proc_changed |= ImGui::Checkbox("Use idle noise profile", &ui_cfg.calib_enable_noise_profile);
          proc_changed |= ImGui::SliderFloat("Calibration target RMS", &ui_cfg.calib_target_rms, 100.0f, 2000.0f, "%.0f");
          proc_changed |= ImGui::SliderFloat("Calibration noise offset (dB)", &ui_cfg.calib_noise_offset_db, 0.0f, 20.0f, "%.1f");
          if (ImGui::MenuItem("Start auto-calibration")) g_calib_start.store(true);
          ImGui::EndMenu();
        }
        if (ImGui::BeginMenu("Record / Replay")) {
          bool rec = g_record_enable.load();
          bool rep = g_replay_enable.load();
          if (ImGui::Checkbox("Record UDP hops", &rec)) g_record_enable.store(rec);
          if (ImGui::Checkbox("Replay from capture", &rep)) g_replay_enable.store(rep);
          if (ImGui::MenuItem("Reload replay file")) g_replay_reload.store(true);
          ImGui::TextDisabled("File: %s", CAPTURE_FILE_PATH);
          ImGui::EndMenu();
        }
        ImGui::EndMenu();
      }
      if (ImGui::BeginMenu("Help")) {
        if (ImGui::MenuItem("Open full help")) show_help_window = true;
        ImGui::Separator();
        ImGui::TextWrapped("Quick tips:");
        ImGui::BulletText("Start with CFAR preset = Balanced.");
        ImGui::BulletText("Enable notch preset 550-600Hz if harmonics dominate.");
        ImGui::BulletText("Use Adaptive noise floor in changing environments.");
        ImGui::EndMenu();
      }
      ImGui::EndMenuBar();
    }

    if (show_help_window) {
      ImGui::Begin("Analyzer Help", &show_help_window, ImGuiWindowFlags_AlwaysAutoResize);
      ImGui::TextWrapped("This menu organizes live controls by signal-processing stage.");
      ImGui::Separator();
      ImGui::TextColored(COL_CYAN, "Acquisition & Display");
      ImGui::BulletText("FFT Size: larger gives finer frequency resolution but slower response.");
      ImGui::BulletText("Example: 8192 -> ~12.2 Hz/bin, 16384 -> ~6.1 Hz/bin.");
      ImGui::BulletText("Scope ms/div: controls oscilloscope time span.");
      ImGui::BulletText("Fmax FFT: limits displayed and analyzed frequency range.");
      ImGui::Separator();
      ImGui::TextColored(COL_CYAN, "CFAR Detection");
      ImGui::BulletText("Guard cells protect target energy from contaminating noise estimate.");
      ImGui::BulletText("Ref cells estimate local noise floor.");
      ImGui::BulletText("Alpha raises threshold above local floor.");
      ImGui::BulletText("Example: increase alpha if false alarms appear.");
      ImGui::Separator();
      ImGui::TextColored(COL_CYAN, "Noise / Mains / Notch");
      ImGui::BulletText("Adaptive noise floor tracks environmental drift.");
      ImGui::BulletText("Rise alpha reacts to sudden noise increase; fall alpha controls recovery speed.");
      ImGui::BulletText("Notch filter suppresses known interference bands.");
      ImGui::BulletText("Example: 575 Hz center, 90 Hz width for a 550-600 Hz disturbance.");
      ImGui::Separator();
      ImGui::TextColored(COL_CYAN, "Kalman / Tracking / Calibration");
      ImGui::BulletText("Kalman Q/R tune responsiveness vs smoothing.");
      ImGui::BulletText("Top-N tracks keeps multiple Doppler candidates for crowded scenes.");
      ImGui::BulletText("Auto-calibration estimates DC offset, gain normalization, and optional idle noise profile.");
      ImGui::BulletText("Example workflow: keep radar still -> run auto-calibration -> enable idle noise profile.");
      ImGui::Separator();
      ImGui::TextColored(COL_CYAN, "Record / Replay");
      ImGui::BulletText("Record captures raw hops for offline A/B tuning.");
      ImGui::BulletText("Replay lets you test identical data with different settings.");
      ImGui::BulletText("Example: compare CFAR presets on same capture.");
      ImGui::End();
    }

    /* Titre */
    ImGui::TextColored(COL_WHITE,
      "RADAR DOPPLER 24.125 GHz — Analyseur PC [FFT %d | CFAR | Kalman]", g_fft_size);
    ImGui::SameLine(ImGui::GetWindowWidth() - 300);
    if (local_connected) {
      ImGui::TextColored(COL_GREEN, "ETH CONNECTED");
      ImGui::SameLine();
      ImGui::TextColored(COL_GREY, "%.1f fps | Res: %.2f Hz", local_fps, FS / g_fft_size);
    } else {
      ImGui::TextColored(COL_RED, "WAITING FOR STM32...");
    }
    ImGui::Separator();

    if (proc_changed) {
      ui_cfg.max_display_freq_hz = std::clamp(ui_cfg.max_display_freq_hz, 1000.0f, FS * 0.5f);
      ui_cfg.cfar.guard_cells = std::max(1, ui_cfg.cfar.guard_cells);
      ui_cfg.cfar.ref_cells = std::max(2, ui_cfg.cfar.ref_cells);
      ui_cfg.spectrum_avg_alpha = std::clamp(ui_cfg.spectrum_avg_alpha, 0.01f, 1.00f);
      ui_cfg.peak_median_len = std::max(1, ui_cfg.peak_median_len);
      ui_cfg.mains_harmonics = std::max(1, ui_cfg.mains_harmonics);
      ui_cfg.kalman_miss_reset_count = std::max(1, ui_cfg.kalman_miss_reset_count);
      ui_cfg.noise_floor_rise_alpha = std::clamp(ui_cfg.noise_floor_rise_alpha, 0.001f, 1.0f);
      ui_cfg.noise_floor_fall_alpha = std::clamp(ui_cfg.noise_floor_fall_alpha, 0.001f, 1.0f);
      ui_cfg.notch_center_hz = std::clamp(ui_cfg.notch_center_hz, 10.0f, FS * 0.5f);
      ui_cfg.notch_width_hz = std::clamp(ui_cfg.notch_width_hz, 1.0f, 1000.0f);
      ui_cfg.notch_depth_db = std::clamp(ui_cfg.notch_depth_db, 0.0f, 80.0f);
      ui_cfg.top_n_tracks = std::clamp(ui_cfg.top_n_tracks, 1, MAX_TRACKS);
      ui_cfg.track_gate_hz = std::clamp(ui_cfg.track_gate_hz, 10.0f, 400.0f);
      set_processing_config(ui_cfg);
      g_requested_proc_refresh.store(true);
    }

    float panel_w = ImGui::GetContentRegionAvail().x;
    float panel_h = ImGui::GetContentRegionAvail().y;

    top_ratio = std::clamp(top_ratio, 0.15f, 0.70f);
    mid_ratio = std::clamp(mid_ratio, 0.10f, 0.70f);
    if (top_ratio + mid_ratio > 0.90f) mid_ratio = 0.90f - top_ratio;

    float top_h = panel_h * top_ratio;
    float half_w = panel_w * 0.5f - 4;
    float mid_h = 0.0f;
    float mid_col_w = 0.0f;
    float info_h = 0.0f;

    if (layout_mode == LAYOUT_FFT_ONLY) {
      ImGui::BeginChild("fft_only", ImVec2(panel_w, panel_h), false);
      int display_bins = (int)(ui_cfg.max_display_freq_hz / (FS / 2.0f) * half_fft);
      if (display_bins > half_fft) display_bins = half_fft;
      float fft_y_min = 1e12f, fft_y_max = -1e12f;
      for (int i = 1; i < display_bins; i++) {
        if (local_fft[i] < fft_y_min) fft_y_min = local_fft[i];
        if (local_fft[i] > fft_y_max) fft_y_max = local_fft[i];
        if (local_cfar[i] < fft_y_min) fft_y_min = local_cfar[i];
        if (local_cfar[i] > fft_y_max) fft_y_max = local_cfar[i];
      }
      float fft_y_range = fft_y_max - fft_y_min;
      if (fft_y_range < 10.0f) fft_y_range = 10.0f;
      fft_y_min -= fft_y_range * 0.05f;
      fft_y_max += fft_y_range * 0.10f;
      DrawGraph("FFT + CFAR (FULL)", local_fft.data(), display_bins,
                0, ui_cfg.max_display_freq_hz, fft_y_min, fft_y_max,
                ImVec2(panel_w - 4, panel_h - 4),
                IM_COL32(68, 170, 255, 255),
                local_cfar.data(), IM_COL32(255, 100, 0, 180),
                local_detected ? local_peak_freq : -1.0f,
                IM_COL32(255, 68, 68, 200), true, true);
      ImGui::EndChild();
      ImGui::End();
      goto render_pass;
    }
    if (layout_mode == LAYOUT_SCOPE_ONLY) {
      ImGui::BeginChild("scope_only", ImVec2(panel_w, panel_h), false);
      DrawGraph("Oscilloscope ADC (FULL)", scope_data.data(), n_scope,
                0, (float)n_scope / FS * 1000.0f, -scope_amp, scope_amp,
                ImVec2(panel_w - 4, panel_h - 4), IM_COL32(0, 255, 136, 255));
      ImGui::EndChild();
      ImGui::End();
      goto render_pass;
    }
    if (layout_mode == LAYOUT_SPECTRO_ONLY) {
      ImGui::BeginChild("spectro_only", ImVec2(panel_w, panel_h), true, ImGuiWindowFlags_NoScrollbar);
      ImDrawList *dl = ImGui::GetWindowDrawList();
      ImVec2 p = ImGui::GetCursorScreenPos();
      ImVec2 region = ImGui::GetContentRegionAvail();
      if (local_n_disp > 0 && !local_spectro.empty()) {
        update_spectro_texture(local_spectro.data(), local_spectro_idx, SPECTRO_ROWS, local_n_disp, -20, 70);
        if (g_spectro_tex)
          dl->AddImage((ImTextureID)(intptr_t)g_spectro_tex, p, ImVec2(p.x + region.x, p.y + region.y), ImVec2(0,0), ImVec2(1,1));
      }
      ImGui::EndChild();
      ImGui::End();
      goto render_pass;
    }

    ImGui::BeginChild("top_row", ImVec2(panel_w, top_h), false);

    /* Oscilloscope */
    {
      char lbl[128];
      float y_div = scope_amp / 4.0f;
      snprintf(lbl, sizeof(lbl), "Oscilloscope ADC (%d ech, %d ms/div, %.1f ADC/div)%s",
               n_scope, scope_time_div_ms, y_div, scope_freeze ? " [FREEZE]" : "");
      DrawGraph(lbl, scope_data.data(), n_scope,
                0, (float)n_scope / FS * 1000.0f,
                -scope_amp, scope_amp,
                ImVec2(half_w, top_h - 4),
                IM_COL32(0, 255, 136, 255));
    }

    ImGui::SameLine();

    /* FFT + seuil CFAR */
    {
      int display_bins = (int)(ui_cfg.max_display_freq_hz / (FS / 2.0f) * half_fft);
      if (display_bins > half_fft) display_bins = half_fft;

      /* Auto-scale Y-axis : trouver min/max dans les bins affichés
       * pour que le spectre tienne toujours dans la fenêtre,
       * quelle que soit la taille FFT (la magnitude dB augmente avec N). */
      float fft_y_min = 1e12f, fft_y_max = -1e12f;
      for (int i = 1; i < display_bins; i++) {  /* skip bin 0 (DC) */
        if (local_fft[i] < fft_y_min) fft_y_min = local_fft[i];
        if (local_fft[i] > fft_y_max) fft_y_max = local_fft[i];
        /* Inclure aussi le seuil CFAR dans le range */
        if (local_cfar[i] < fft_y_min) fft_y_min = local_cfar[i];
        if (local_cfar[i] > fft_y_max) fft_y_max = local_cfar[i];
      }
      /* Ajouter une marge de 10% en haut et en bas */
      float fft_y_range = fft_y_max - fft_y_min;
      if (fft_y_range < 10.0f) fft_y_range = 10.0f;  /* range minimum */
      fft_y_min -= fft_y_range * 0.05f;
      fft_y_max += fft_y_range * 0.10f;

      /* Lissage pour éviter les sauts brusques de l'échelle */
      static float smooth_y_min = -20.0f, smooth_y_max = 80.0f;
      smooth_y_min += (fft_y_min - smooth_y_min) * 0.15f;
      smooth_y_max += (fft_y_max - smooth_y_max) * 0.15f;

      char lbl[256];
      if (local_detected)
        snprintf(lbl, sizeof(lbl), "FFT + CFAR  |  f=%.1f Hz  SNR=+%.1f dB  [%d det]  |  Y: %.0f..%.0f dB",
                 local_peak_freq, local_peak_snr, local_cfar_n, smooth_y_min, smooth_y_max);
      else
        snprintf(lbl, sizeof(lbl), "FFT + CFAR  |  Pas de cible  |  Y: %.0f..%.0f dB", smooth_y_min, smooth_y_max);

      DrawGraph(lbl, local_fft.data(), display_bins,
                0, ui_cfg.max_display_freq_hz, smooth_y_min, smooth_y_max,
                ImVec2(half_w, top_h - 4),
                IM_COL32(68, 170, 255, 255),
                local_cfar.data(), IM_COL32(255, 100, 0, 180),
                local_detected ? local_peak_freq : -1.0f,
                IM_COL32(255, 68, 68, 200),
                true,
                true);
    }

    ImGui::EndChild();

    /* === Rangée du milieu : Spectrogramme (50%) + Vitesse temporelle (50%) === */
    mid_h = panel_h * mid_ratio;
    mid_col_w = panel_w / 3.0f - 6.0f;

    ImGui::BeginChild("mid_row", ImVec2(panel_w, mid_h), false);

    /* Spectrogramme */
    {
      ImGui::BeginChild("Spectrogram", ImVec2(mid_col_w, mid_h - 4), true,
                         ImGuiWindowFlags_NoScrollbar);
      ImDrawList *dl = ImGui::GetWindowDrawList();
      ImVec2 p = ImGui::GetCursorScreenPos();
      ImVec2 region = ImGui::GetContentRegionAvail();

      if (local_n_disp > 0 && !local_spectro.empty()) {
        update_spectro_texture(local_spectro.data(), local_spectro_idx,
                               SPECTRO_ROWS, local_n_disp, -20, 70);
        if (g_spectro_tex)
          dl->AddImage((ImTextureID)(intptr_t)g_spectro_tex,
                       p, ImVec2(p.x + region.x, p.y + region.y),
                       ImVec2(0, 0), ImVec2(1, 1));
      }
      dl->AddText(ImVec2(p.x + 4, p.y + 2),
                  IM_COL32(255, 170, 0, 200), "Spectrogramme (0-8 kHz)");
      ImGui::SetWindowFontScale(1.25f);
      dl->AddText(ImVec2(p.x + 6, p.y + region.y - 18), IM_COL32(200, 200, 200, 200), "0 Hz");
      dl->AddText(ImVec2(p.x + region.x * 0.5f - 30, p.y + region.y - 18), IM_COL32(200, 200, 200, 200), "4 kHz");
      dl->AddText(ImVec2(p.x + region.x - 52, p.y + region.y - 18), IM_COL32(200, 200, 200, 200), "8 kHz");
      ImGui::SetWindowFontScale(1.0f);

      ImGui::EndChild();
    }

    ImGui::SameLine();

    /* Long-term Doppler trend */
    {
      ImGui::BeginChild("Doppler Trend", ImVec2(mid_col_w, mid_h - 4), true,
                         ImGuiWindowFlags_NoScrollbar);
      ImDrawList *dl = ImGui::GetWindowDrawList();
      ImVec2 p = ImGui::GetCursorScreenPos();
      ImVec2 region = ImGui::GetContentRegionAvail();
      float w = region.x, h = region.y;
      dl->AddRectFilled(p, ImVec2(p.x + w, p.y + h), IM_COL32(15, 15, 20, 255));

      float trend_max_hz = std::max(200.0f, ui_cfg.max_display_freq_hz);
      for (int i = 1; i < 4; i++) {
        float yy = p.y + h * i / 4.0f;
        dl->AddLine(ImVec2(p.x, yy), ImVec2(p.x + w, yy), IM_COL32(50, 50, 50, 100));
      }

      int n = (int)doppler_trend_hz.size();
      if (n > 1) {
        for (int i = 0; i < n - 1; i++) {
          float x0 = p.x + (float)i / (n - 1) * w;
          float x1 = p.x + (float)(i + 1) / (n - 1) * w;
          float v0 = 1.0f - std::clamp(doppler_trend_hz[i] / trend_max_hz, 0.0f, 1.0f);
          float v1 = 1.0f - std::clamp(doppler_trend_hz[i + 1] / trend_max_hz, 0.0f, 1.0f);
          dl->AddLine(ImVec2(x0, p.y + v0 * h), ImVec2(x1, p.y + v1 * h),
                      IM_COL32(255, 210, 80, 220), 2.0f);
        }
      }

      char trend_lbl[160];
      snprintf(trend_lbl, sizeof(trend_lbl), "Doppler trend: %ds | Noise floor %.1f dB",
               trend_seconds, local_noise_floor_db);
      dl->AddText(ImVec2(p.x + 4, p.y + 2), IM_COL32(220, 220, 220, 220), trend_lbl);
      ImGui::EndChild();
    }

    ImGui::SameLine();

    /* Historique des vitesses (Kalman vs brut) */
    {
      float max_speed = 50.0f;
      for (float v : speed_history) if (fabsf(v) > max_speed) max_speed = fabsf(v) * 1.2f;
      for (float v : speed_raw_history) if (fabsf(v) > max_speed) max_speed = fabsf(v) * 1.2f;

      /* Dessiner manuellement les deux courbes */
      ImGui::BeginChild("Speed History", ImVec2(mid_col_w, mid_h - 4), true,
                         ImGuiWindowFlags_NoScrollbar);
      ImDrawList *dl = ImGui::GetWindowDrawList();
      ImVec2 p = ImGui::GetCursorScreenPos();
      ImVec2 region = ImGui::GetContentRegionAvail();
      float w = region.x, h = region.y;

      dl->AddRectFilled(p, ImVec2(p.x + w, p.y + h), IM_COL32(15, 15, 20, 255));

      /* Grille */
      for (int i = 1; i < 4; i++) {
        float yy = p.y + h * i / 4.0f;
        dl->AddLine(ImVec2(p.x, yy), ImVec2(p.x + w, yy), IM_COL32(50, 50, 50, 100));
      }

      int n = (int)speed_history.size();
      if (n > 1) {
        /* Courbe brute (gris) */
        for (int i = 0; i < n - 1; i++) {
          float x0 = p.x + (float)i / (n - 1) * w;
          float x1 = p.x + (float)(i + 1) / (n - 1) * w;
          float v0 = 1.0f - std::clamp((speed_raw_history[i] + max_speed) / (2 * max_speed), 0.0f, 1.0f);
          float v1 = 1.0f - std::clamp((speed_raw_history[i+1] + max_speed) / (2 * max_speed), 0.0f, 1.0f);
          dl->AddLine(ImVec2(x0, p.y + v0 * h), ImVec2(x1, p.y + v1 * h),
                      IM_COL32(100, 100, 100, 150), 1.0f);
        }
        /* Courbe Kalman (vert) */
        for (int i = 0; i < n - 1; i++) {
          float x0 = p.x + (float)i / (n - 1) * w;
          float x1 = p.x + (float)(i + 1) / (n - 1) * w;
          float v0 = 1.0f - std::clamp((speed_history[i] + max_speed) / (2 * max_speed), 0.0f, 1.0f);
          float v1 = 1.0f - std::clamp((speed_history[i+1] + max_speed) / (2 * max_speed), 0.0f, 1.0f);
          dl->AddLine(ImVec2(x0, p.y + v0 * h), ImVec2(x1, p.y + v1 * h),
                      IM_COL32(0, 255, 136, 255), 2.0f);
        }
      }

      dl->AddText(ImVec2(p.x + 4, p.y + 2),
                  IM_COL32(200, 200, 200, 200), "Vitesse (vert=Kalman, gris=brut)");

      ImGui::EndChild();
    }

    ImGui::EndChild();

    /* === Rangée du bas : Affichage numérique === */
    info_h = panel_h * 0.35f;
    ImGui::BeginChild("InfoPanel", ImVec2(panel_w, info_h), true);

    if (local_detected)
    {
      /* Grande vitesse Kalman en vert */
      ImGui::SetWindowFontScale(2.4f);
      ImGui::TextColored(COL_GREEN, "  Kalman: %6.1f km/h", local_kal_kmh);
      ImGui::TextColored(COL_GREY,  "  Brut  : %6.1f km/h", local_raw_kmh);
      ImGui::SetWindowFontScale(1.0f);

      ImGui::Spacing();

      /* Détails */
      ImGui::TextColored(COL_WHITE,
        "  Kalman: %.1f m/s  |  Brut: %.1f km/h  |  Accel: %.2f m/s²",
        local_kal_kmh / 3.6f, local_raw_kmh, local_kal_accel);

      ImGui::TextColored(COL_BLUE,
        "  f = %.1f Hz  |  CFAR SNR = +%.1f dB  |  %d bins détectés",
        local_peak_freq, local_peak_snr, local_cfar_n);

      ImGui::Spacing();
      ImGui::Separator();
      ImGui::Spacing();

      /* Barres SNR et confiance */
      float snr_frac = std::clamp(local_peak_snr / 30.0f, 0.0f, 1.0f);
      ImGui::Text("  CFAR SNR:");
      ImGui::SameLine();
      ImGui::PushStyleColor(ImGuiCol_PlotHistogram,
        snr_frac > 0.5f ? (ImVec4)COL_GREEN : (ImVec4)COL_ORANGE);
      ImGui::ProgressBar(snr_frac, ImVec2(300, 20), "");
      ImGui::PopStyleColor();
      ImGui::SameLine();
      ImGui::Text("+%.1f dB", local_peak_snr);

      ImGui::SameLine(panel_w - 250);
      ImGui::TextColored(COL_GREEN, "CIBLE DETECTEE [CFAR]");

      ImGui::Spacing();

      /* Infos FFT */
      ImGui::TextColored(COL_GREY,
        "  FFT: %d pts | Res: %.2f Hz = %.3f m/s | CFAR: guard=%d ref=%d alpha=%.0f dB",
        g_fft_size, FS / g_fft_size,
        (FS / g_fft_size) / DOPPLER_SCALE,
        ui_cfg.cfar.guard_cells, ui_cfg.cfar.ref_cells, ui_cfg.cfar.alpha_db);
    }
    else
    {
      ImGui::SetWindowFontScale(3.5f);
      ImGui::TextColored(COL_GREY, "   --.- km/h");
      ImGui::SetWindowFontScale(1.0f);

      ImGui::Spacing();
      ImGui::TextColored(COL_GREY,
        "  En attente de cible...  |  CFAR actif  |  FFT %d pts", g_fft_size);

      ImGui::SameLine(panel_w - 250);
      ImGui::TextColored(COL_ORANGE, "PAS DE CIBLE");
    }

    ImGui::EndChild();

    ImGui::End();

render_pass:
    /* === Rendu OpenGL === */
    ImGui::Render();
    int display_w, display_h;
    glfwGetFramebufferSize(window, &display_w, &display_h);
    glViewport(0, 0, display_w, display_h);
    glClearColor(0.06f, 0.06f, 0.08f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT);
    ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
    glfwSwapBuffers(window);
  }

  /* === Nettoyage === */
  g_running = false;
  if (udp_thread.joinable()) udp_thread.join();

  if (g_spectro_tex) glDeleteTextures(1, &g_spectro_tex);

  ImGui_ImplOpenGL3_Shutdown();
  ImGui_ImplGlfw_Shutdown();
  ImGui::DestroyContext();
  glfwDestroyWindow(window);
  glfwTerminate();

  return 0;
}
