/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Programme principal - Radar Doppler 24.125 GHz
  *                   Version 2 : FS=100kHz, FFT=4096, UDP streaming Ethernet
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "string.h"
#include <math.h>
#include <stdio.h>

/* CMSIS-DSP : bibliothèque de traitement du signal optimisée pour Cortex-M7.
 * Fournit des fonctions FFT, filtrage, statistiques, etc. qui exploitent
 * les instructions DSP matérielles du Cortex-M7 (FPU, SIMD). */
#include "arm_math.h"

/* Pilote de l'écran OLED SSD1306 128x64 connecté en I2C. */
#include "ssd1306.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* === Taille du buffer DMA pour l'ADC ===
 * Double-buffer circulaire de 2048 échantillons à 100 kHz.
 * Interruption "half-complete" après 1024, "complete" après 2048.
 * Pendant que le DMA remplit une moitié, le CPU traite l'autre. */
#define ADC_BUF_SIZE      2048

/* === Taille de la FFT embarquée (STM32) ===
 * 4096 points à 100 kHz → résolution fréquentielle :
 *   Δf = FS / FFT_SIZE = 100000 / 4096 ≈ 24.4 Hz par bin
 * Résolution en vitesse : Δv = Δf / 161 ≈ 0.15 m/s ≈ 0.54 km/h
 * Fréquence max observable (Nyquist) = FS/2 = 50 kHz → vmax ≈ 310 m/s
 * Mais le filtre passe-bas AFE coupe à 6 kHz → vmax utile ≈ 37 m/s */
#define FFT_SIZE          4096

/* Fréquence d'échantillonnage ADC en Hz.
 * Timer 2 cadence à 100 kHz :
 *   fAPB1_timer = 108 MHz, prescaler = 0, ARR = 1079
 *   → ftimer = 108 MHz / (1079+1) = 100 000 Hz */
#define FS                100000.0f

/* Facteur de conversion Doppler : fd = 2·v·f0/c
 * f0 = 24.125 GHz → facteur = 2 × 24.125e9 / 3e8 ≈ 161 Hz/(m/s) */
#define DOPPLER_SCALE     161.0f

/* === Plage de recherche du pic FFT ===
 * MIN_BIN = 2 : évite la fuite DC (bins 0-1)
 * MAX_BIN = 250 : bin 250 × 24.4 Hz ≈ 6100 Hz → vitesse max ≈ 38 m/s ≈ 137 km/h
 * Cohérent avec le filtre passe-bas analogique à 6 kHz */
#define MIN_BIN           2
#define MAX_BIN           250

/* === Seuils de détection ===
 * SNR_THRESHOLD : 15 dB minimum (pic ~31× au-dessus du bruit moyen)
 * ADC_PP_THRESHOLD : amplitude crête-à-crête minimum ~80 mV */
#define SNR_THRESHOLD     15.0f
#define ADC_PP_THRESHOLD  100

/* === Configuration UDP pour le streaming Ethernet ===
 * On envoie les échantillons ADC bruts par hop DMA de 1024 points
 * (1024 × uint16 = 2048 octets) via UDP. À ~97.6 hops/s :
 * 100000 / 1024 = 97.656 Hz → ~195 ko/s.
 * Ethernet 100 Mbps = 12.5 Mo/s → charge réseau ≈ 1.6% (trivial).
 *
 * Adresses IP statiques (pas de DHCP pour la simplicité) :
 *   STM32 : 192.168.1.10
 *   PC    : 192.168.1.100
 * Port UDP : 5555 */
#define UDP_DST_PORT      5555
#define UDP_SRC_PORT      5555

/* Adresse IP du STM32 : 192.168.1.10 */
#define IP_SRC_0  192
#define IP_SRC_1  168
#define IP_SRC_2  1
#define IP_SRC_3  10

/* Adresse IP du PC destinataire : 192.168.1.100 */
#define IP_DST_0  192
#define IP_DST_1  168
#define IP_DST_2  1
#define IP_DST_3  100

/* Adresse MAC du STM32 (identique à celle configurée dans MX_ETH_Init) */
#define MAC_SRC_0  0x00
#define MAC_SRC_1  0x80
#define MAC_SRC_2  0xE1
#define MAC_SRC_3  0x00
#define MAC_SRC_4  0x00
#define MAC_SRC_5  0x00

/* Adresse MAC de broadcast (le PC recevra tout paquet sur le port 5555).
 * On utilise broadcast pour éviter d'avoir à connaître la MAC du PC.
 * Le routeur/switch délivrera le paquet à tous les périphériques du LAN. */
#define MAC_DST_0  0xFF
#define MAC_DST_1  0xFF
#define MAC_DST_2  0xFF
#define MAC_DST_3  0xFF
#define MAC_DST_4  0xFF
#define MAC_DST_5  0xFF

/* Taille max d'un paquet UDP (payload) dans un frame Ethernet standard.
 * MTU = 1500 octets - 20 (IP) - 8 (UDP) = 1472 octets de données utiles.
 * En mode hop, on fragmente 2048 octets ADC en 2 paquets max. */
#define UDP_MAX_PAYLOAD   1400

/* Taille d'un hop UDP : 1024 échantillons (demi-buffer DMA ADC) */
#define UDP_HOP_SAMPLES     (ADC_BUF_SIZE / 2)

/* Nombre d'échantillons ADC envoyés par trame UDP */
#define UDP_STREAM_SAMPLES  UDP_HOP_SAMPLES  /* 1024 échantillons = 2048 octets */

/* Nombre de paquets UDP nécessaires pour envoyer une trame UDP (hop).
 * 2048 / 1400 = 1.46 → 2 paquets (le dernier est plus court). */
#define UDP_FRAGMENT_COUNT  ((UDP_STREAM_SAMPLES * 2 + UDP_MAX_PAYLOAD - 1) / UDP_MAX_PAYLOAD)

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* === Descripteurs DMA Ethernet ===
 * Placés à des adresses fixes en SRAM2 (0x2007C000). */
#if defined ( __ICCARM__ ) /*!< IAR Compiler */
#pragma location=0x2007c000
ETH_DMADescTypeDef  DMARxDscrTab[ETH_RX_DESC_CNT]; /* Ethernet Rx DMA Descriptors */
#pragma location=0x2007c0a0
ETH_DMADescTypeDef  DMATxDscrTab[ETH_TX_DESC_CNT]; /* Ethernet Tx DMA Descriptors */

#elif defined ( __CC_ARM )  /* MDK ARM Compiler */

__attribute__((at(0x2007c000))) ETH_DMADescTypeDef  DMARxDscrTab[ETH_RX_DESC_CNT]; /* Ethernet Rx DMA Descriptors */
__attribute__((at(0x2007c0a0))) ETH_DMADescTypeDef  DMATxDscrTab[ETH_TX_DESC_CNT]; /* Ethernet Tx DMA Descriptors */

#elif defined ( __GNUC__ ) /* GNU Compiler */

ETH_DMADescTypeDef DMARxDscrTab[ETH_RX_DESC_CNT] __attribute__((section(".RxDecripSection"))); /* Ethernet Rx DMA Descriptors */
ETH_DMADescTypeDef DMATxDscrTab[ETH_TX_DESC_CNT] __attribute__((section(".TxDecripSection")));   /* Ethernet Tx DMA Descriptors */
#endif

ETH_TxPacketConfig TxConfig;

/* === Handles des périphériques HAL === */
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

DAC_HandleTypeDef hdac;
DMA_HandleTypeDef hdma_dac1;

ETH_HandleTypeDef heth;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart3;

PCD_HandleTypeDef hpcd_USB_OTG_FS;

I2C_HandleTypeDef hi2c1;

/* USER CODE BEGIN PV */

/* === Variables OLED === */
uint32_t oled_last_update = 0;
#define OLED_UPDATE_MS  200  /* rafraîchir l'OLED toutes les 200 ms (5 Hz) */

/* Dernières valeurs mesurées pour l'affichage OLED */
float last_speed_kmh = 0;
float last_speed_ms = 0;
float last_freq = 0;
float last_snr = 0;
uint8_t last_detected = 0;

/* === Buffer DMA double pour l'ADC (2048 échantillons à 100 kHz) ===
 * Interruption HT après 1024, TC après 2048. */
volatile uint16_t adc_buf[ADC_BUF_SIZE];

/* Drapeaux d'interruption DMA */
volatile uint8_t  adc_half_ready = 0;
volatile uint8_t  adc_full_ready = 0;

/* === Buffers FFT embarquée (4096 points) ===
 * fft_in  : accumulation de 4096 échantillons float32 (16 ko)
 * fft_out : sortie FFT complexe (16 ko)
 * fft_mag : magnitudes (8 ko)
 * hann_window : coefficients de fenêtrage (16 ko) */
float32_t fft_in[FFT_SIZE];
float32_t fft_out[FFT_SIZE];
float32_t fft_mag[FFT_SIZE / 2];
float32_t hann_window[FFT_SIZE];

/* Instance FFT CMSIS-DSP (twiddle factors pré-calculés) */
arm_rfft_fast_instance_f32 fft_instance;

/* === Buffer UDP hop : copie brute des 1024 échantillons ADC uint16 ===
 * Il est rempli à chaque DMA HT/TC puis envoyé immédiatement via UDP.
 * Le PC reconstruit ensuite sa fenêtre glissante de 4096 échantillons. */
uint16_t udp_hop_buf[UDP_HOP_SAMPLES];

/* Pointeur d'accumulation FFT : incrémenté de 1024 à chaque DMA HT/TC.
 * Quand fft_fill >= FFT_SIZE (4096), le buffer est plein → lancer la FFT. */
uint16_t fft_fill = 0;
uint8_t  fft_ready = 0;

/* === Statistiques ADC === */
volatile uint16_t adc_min = 4095;
volatile uint16_t adc_max = 0;
volatile uint32_t adc_sum = 0;
volatile uint32_t adc_count = 0;

/* === Paramètres du sweep DAC (mode test) === */
#define DAC_BUF_SIZE  1024
#define SWEEP_F_START 30.0f
#define SWEEP_F_END   6000.0f
#define SWEEP_TIME_MS 15000

uint16_t dac_buf[DAC_BUF_SIZE];
float sweep_phase = 0.0f;
uint32_t sweep_start_tick = 0;

/* === Buffer UART pour compatibilité avec l'ancien protocole ===
 * On garde le streaming UART décimé en parallèle du streaming UDP.
 * Le UART envoie 1024 échantillons (décimation 4:1 depuis 4096).
 * Le UDP envoie un hop brut de 1024 échantillons à chaque HT/TC DMA. */
#define ADC_STREAM_SAMPLES  1024
uint16_t stream_buf[ADC_STREAM_SAMPLES];

/* === Buffer Ethernet pour construction de trames UDP brutes ===
 * On construit manuellement les trames Ethernet/IP/UDP sans lwIP.
 * En-tête total : 14 (Ethernet) + 20 (IPv4) + 8 (UDP) = 42 octets.
 *
 * Buffer aligné sur 4 octets pour le DMA Ethernet. */
__ALIGN_BEGIN static uint8_t eth_tx_buf[1514] __ALIGN_END;

/* Compteur de séquence UDP pour que le PC détecte les paquets perdus.
 * Incrémenté à chaque hop envoyé (pas à chaque fragment). */
static uint32_t udp_seq_counter = 0;

/* Flag indiquant que le lien Ethernet est UP (PHY connecté). */
static uint8_t eth_link_up = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ETH_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_USB_OTG_FS_PCD_Init(void);
static void MX_ADC1_Init(void);
static void MX_DAC_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM4_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* ═══════════════════════════════════════════════════════════════════════════
 * Construction manuelle de trames UDP brutes (sans lwIP)
 * ═══════════════════════════════════════════════════════════════════════════
 *
 * On fabrique les trames Ethernet/IP/UDP manuellement car ajouter lwIP
 * nécessiterait de regénérer le projet CubeMX. La pile "bare-metal"
 * est suffisante pour du streaming unidirectionnel en broadcast.
 *
 * Structure d'une trame :
 *   [14 octets Ethernet] [20 octets IPv4] [8 octets UDP] [données]
 *
 * Checksum IP : obligatoire (calculé par logiciel).
 * Checksum UDP : optionnel en IPv4, on met 0x0000 pour simplifier.
 */

/**
 * @brief  Calcul du checksum IPv4 (RFC 791).
 *
 * Le checksum IP est le complément à 1 de la somme des mots de 16 bits
 * de l'en-tête. On additionne tous les mots de 16 bits avec carry,
 * puis on inverse le résultat.
 *
 * @param  data  Pointeur vers l'en-tête IPv4 (20 octets)
 * @param  len   Longueur de l'en-tête en octets (toujours 20 ici)
 * @return       Checksum sur 16 bits en ordre réseau (big-endian)
 */
static uint16_t ip_checksum(const uint8_t *data, int len)
{
  uint32_t sum = 0;
  /* Additionner les mots de 16 bits (big-endian) */
  for (int i = 0; i < len; i += 2)
    sum += ((uint32_t)data[i] << 8) | data[i + 1];
  /* Replier les retenues de 32 bits dans les 16 bits bas */
  while (sum >> 16)
    sum = (sum & 0xFFFF) + (sum >> 16);
  /* Complément à 1 */
  return (uint16_t)(~sum);
}

/**
 * @brief  Envoie un paquet UDP brut via HAL Ethernet.
 *
 * Construit l'en-tête Ethernet (14 oct) + IPv4 (20 oct) + UDP (8 oct)
 * puis copie les données payload et transmet via HAL_ETH_Transmit().
 *
 * @param  payload      Données à envoyer
 * @param  payload_len  Taille des données en octets (max ~1472)
 */
static HAL_StatusTypeDef udp_send_raw(const uint8_t *payload, uint16_t payload_len)
{
  /* Taille totale des données IP (en-tête IP + en-tête UDP + payload) */
  uint16_t ip_total_len = 20 + 8 + payload_len;
  /* Taille totale de la trame Ethernet (sans CRC, ajouté par le matériel) */
  uint16_t eth_frame_len = 14 + ip_total_len;

  /* Limiter à la taille du buffer */
  if (eth_frame_len > sizeof(eth_tx_buf)) return HAL_ERROR;

  uint8_t *p = eth_tx_buf;

  /* ── En-tête Ethernet (14 octets) ──
   * [6 oct MAC destination] [6 oct MAC source] [2 oct EtherType] */
  *p++ = MAC_DST_0; *p++ = MAC_DST_1; *p++ = MAC_DST_2;
  *p++ = MAC_DST_3; *p++ = MAC_DST_4; *p++ = MAC_DST_5;
  *p++ = MAC_SRC_0; *p++ = MAC_SRC_1; *p++ = MAC_SRC_2;
  *p++ = MAC_SRC_3; *p++ = MAC_SRC_4; *p++ = MAC_SRC_5;
  *p++ = 0x08; *p++ = 0x00;  /* EtherType = 0x0800 = IPv4 */

  /* ── En-tête IPv4 (20 octets, pas d'options) ── */
  uint8_t *ip_hdr = p;
  *p++ = 0x45;         /* Version 4, IHL = 5 (20 octets, pas d'options) */
  *p++ = 0x00;         /* DSCP = 0, ECN = 0 */
  *p++ = (uint8_t)(ip_total_len >> 8);   /* Longueur totale (MSB) */
  *p++ = (uint8_t)(ip_total_len & 0xFF); /* Longueur totale (LSB) */
  *p++ = 0x00; *p++ = 0x00;  /* Identification = 0 */
  *p++ = 0x40; *p++ = 0x00;  /* Flags: Don't Fragment, Fragment Offset = 0 */
  *p++ = 64;                  /* TTL = 64 (standard) */
  *p++ = 17;                  /* Protocole = 17 = UDP */
  *p++ = 0x00; *p++ = 0x00;  /* Checksum (rempli après calcul) */
  *p++ = IP_SRC_0; *p++ = IP_SRC_1; *p++ = IP_SRC_2; *p++ = IP_SRC_3;
  *p++ = IP_DST_0; *p++ = IP_DST_1; *p++ = IP_DST_2; *p++ = IP_DST_3;

  /* Calcul du checksum IPv4 et insertion dans l'en-tête */
  uint16_t cksum = ip_checksum(ip_hdr, 20);
  ip_hdr[10] = (uint8_t)(cksum >> 8);
  ip_hdr[11] = (uint8_t)(cksum & 0xFF);

  /* ── En-tête UDP (8 octets) ── */
  uint16_t udp_len = 8 + payload_len;
  *p++ = (uint8_t)(UDP_SRC_PORT >> 8); *p++ = (uint8_t)(UDP_SRC_PORT & 0xFF);
  *p++ = (uint8_t)(UDP_DST_PORT >> 8); *p++ = (uint8_t)(UDP_DST_PORT & 0xFF);
  *p++ = (uint8_t)(udp_len >> 8);      *p++ = (uint8_t)(udp_len & 0xFF);
  *p++ = 0x00; *p++ = 0x00;  /* Checksum UDP = 0 (optionnel en IPv4) */

  /* ── Payload ── */
  memcpy(p, payload, payload_len);

  /* ── Transmission via HAL Ethernet ──
   * On utilise HAL_ETH_Transmit() qui configure le descripteur DMA TX
   * et lance l'envoi matériel. Le CRC Ethernet est ajouté automatiquement
   * par le MAC (configuré dans TxConfig.CRCPadCtrl). */
  ETH_BufferTypeDef tx_buffer;
  tx_buffer.buffer = eth_tx_buf;
  tx_buffer.len = eth_frame_len;
  tx_buffer.next = NULL;

  TxConfig.Length = eth_frame_len;
  TxConfig.TxBuffer = &tx_buffer;

  return HAL_ETH_Transmit(&heth, &TxConfig, 10);  /* timeout 10 ms */
}

/**
 * @brief  Envoie une trame d'échantillons ADC via UDP fragmenté.
 *
 * Les échantillons (n_samples × 2 octets) peuvent être trop gros
 * (MTU = 1500 → payload max ≈ 1472 octets). On les envoie en fragments
 * avec un en-tête de 12 octets par fragment :
 *
 *   [4 oct : séquence]  [2 oct : fragment_id]  [2 oct : fragment_count]
 *   [2 oct : offset]    [2 oct : payload_len]
 *   [N oct : données ADC uint16 little-endian]
 *
 * Le PC réassemble les fragments grâce au numéro de séquence et à l'offset.
 *
 * @param  samples      Tableau d'échantillons uint16 (hop ADC brut)
 * @param  n_samples    Nombre d'échantillons (1024 en mode hop)
 * @param  seq          Numéro de séquence de la trame
 */
static void udp_send_adc_frame(const uint16_t *raw_samples, uint16_t n_samples, uint32_t seq)
{
  /* Buffer temporaire pour un fragment : en-tête (12 oct) + données */
  static uint8_t frag_buf[12 + UDP_MAX_PAYLOAD];

  uint16_t total_bytes = n_samples * 2;
  uint16_t frag_count = (total_bytes + UDP_MAX_PAYLOAD - 1) / UDP_MAX_PAYLOAD;
  uint16_t offset = 0;

  for (uint16_t frag_id = 0; frag_id < frag_count; frag_id++)
  {
    uint16_t chunk = total_bytes - offset;
    if (chunk > UDP_MAX_PAYLOAD) chunk = UDP_MAX_PAYLOAD;

    /* En-tête de fragment (12 octets, little-endian) */
    uint8_t *h = frag_buf;
    h[0] = (uint8_t)(seq);       h[1] = (uint8_t)(seq >> 8);
    h[2] = (uint8_t)(seq >> 16); h[3] = (uint8_t)(seq >> 24);
    h[4] = (uint8_t)(frag_id);   h[5] = (uint8_t)(frag_id >> 8);
    h[6] = (uint8_t)(frag_count); h[7] = (uint8_t)(frag_count >> 8);
    h[8] = (uint8_t)(offset);    h[9] = (uint8_t)(offset >> 8);
    h[10] = (uint8_t)(chunk);    h[11] = (uint8_t)(chunk >> 8);

    /* Copie directe des uint16 bruts (pas de conversion float→uint16) */
    memcpy(frag_buf + 12, ((const uint8_t *)raw_samples) + offset, chunk);

    /* Attendre AVANT l'envoi que le DMA TX ait fini de lire eth_tx_buf
     * du fragment précédent. HAL_ETH_Transmit sur STM32F7 peut rendre
     * la main avant que le DMA ait fini physiquement.
     *
     * Calcul : 1412 octets * 8 / 100 Mbps = 113 µs + IFG ≈ 130 µs
     * On attend 200 µs pour avoir de la marge (4300 boucles à 216 MHz). */
    if (frag_id > 0)
    {
      for (volatile int d = 0; d < 4300; d++) {}
    }

    /* Envoi du fragment. On ne break PAS sur erreur : on tente
     * d'envoyer TOUS les fragments quoi qu'il arrive. Le PC ignore
     * les trames incomplètes grâce au bitmap de fragments. */
    udp_send_raw(frag_buf, 12 + chunk);

    offset += chunk;
  }
}

/**
 * @brief  Vérifie l'état du lien Ethernet (PHY LAN8742A).
 *
 * Lit le registre BSR (Basic Status Register) du PHY via MDIO
 * pour vérifier si le lien physique est établi (câble branché,
 * auto-négociation terminée).
 *
 * @return  1 si le lien est UP, 0 sinon
 */
static uint8_t eth_check_link(void)
{
  uint32_t reg_val = 0;
  /* Adresse du PHY LAN8742A sur le bus MDIO : typiquement 0 */
  /* Registre BSR (Basic Status Register) = registre 1 */
  if (HAL_ETH_ReadPHYRegister(&heth, 0, 1, &reg_val) == HAL_OK)
  {
    /* Bit 2 du BSR = "Link Status" : 1 = lien UP */
    return (reg_val & (1 << 2)) ? 1 : 0;
  }
  return 0;
}

/**
 * @brief  Initialise le lien Ethernet (démarrage du MAC + PHY).
 *
 * Appelé après MX_ETH_Init() pour activer le transceiver.
 * Configure le PHY en auto-négociation et démarre le MAC.
 */
static void eth_start_link(void)
{
  /* Démarrer le MAC Ethernet avec diagnostic UART */
  char dbg[128];

  HAL_StatusTypeDef status = HAL_ETH_Start(&heth);
  snprintf(dbg, sizeof(dbg), "[ETH] HAL_ETH_Start = %d (0=OK)\r\n", (int)status);
  HAL_UART_Transmit(&huart3, (uint8_t *)dbg, strlen(dbg), 50);

  eth_link_up = eth_check_link();
  snprintf(dbg, sizeof(dbg), "[ETH] Link = %s\r\n", eth_link_up ? "UP" : "DOWN");
  HAL_UART_Transmit(&huart3, (uint8_t *)dbg, strlen(dbg), 50);
}

/**
 * @brief  Pré-calcul de la fenêtre de Hann pour le fenêtrage FFT.
 *
 * w[n] = 0.5 × (1 - cos(2π·n / (N-1)))
 *
 * La fenêtre atténue le signal à zéro aux bords pour éliminer les
 * discontinuités qui causent des fuites spectrales. Compromis :
 * le lobe principal est 2× plus large mais les lobes secondaires
 * sont atténués de ~31 dB.
 */
void init_hann_window(void)
{
  for (int i = 0; i < FFT_SIZE; i++)
    hann_window[i] = 0.5f * (1.0f - arm_cos_f32(2.0f * PI * i / (FFT_SIZE - 1)));
}

/**
 * @brief  Calcule la fréquence instantanée du sweep logarithmique.
 *
 * f(t) = f_start × (f_end / f_start)^(t / T)
 *
 * @return  Fréquence actuelle du sweep en Hz
 */
float get_sweep_freq(void)
{
  uint32_t elapsed = HAL_GetTick() - sweep_start_tick;
  if (elapsed >= SWEEP_TIME_MS)
  {
    sweep_start_tick = HAL_GetTick();
    sweep_phase = 0.0f;
    elapsed = 0;
  }
  float t_ratio = (float)elapsed / (float)SWEEP_TIME_MS;
  return SWEEP_F_START * powf(SWEEP_F_END / SWEEP_F_START, t_ratio);
}

/**
 * @brief  Remplit le buffer DAC avec la fréquence actuelle du sweep.
 *
 * Sinusoïde centrée sur 2048 (mi-course 12 bits), amplitude ±800 LSB.
 * La phase est continue entre les appels pour éviter les discontinuités.
 * Note : le DAC tourne à FS=100 kHz maintenant (Timer 4 reconfiguré).
 */
void update_dac_sweep(void)
{
  float freq = get_sweep_freq();
  float phase_inc = 2.0f * PI * freq / FS;

  for (int i = 0; i < DAC_BUF_SIZE; i++)
  {
    dac_buf[i] = (uint16_t)(2048.0f + 800.0f * sinf(sweep_phase));
    sweep_phase += phase_inc;
  }

  if (sweep_phase > 2.0f * PI * 1000.0f)
    sweep_phase = fmodf(sweep_phase, 2.0f * PI);
}

/**
 * @brief  Traitement FFT complet et envoi des résultats.
 *
 * Pipeline :
 * 1. DC removal (arm_mean_f32)
 * 2. Fenêtrage de Hann (arm_mult_f32)
 * 3. FFT réelle 4096 points (arm_rfft_fast_f32)
 * 4. Magnitudes (arm_cmplx_mag_f32)
 * 5. Recherche du pic spectral [MIN_BIN..MAX_BIN]
 * 6. Estimation du plancher de bruit
 * 7. Calcul SNR + vitesses Doppler
 * 8. Streaming UART (protocole binaire, compatibilité ancien PC scope)
 * 9. Mise à jour des données pour l'affichage OLED
 *
 * Temps d'exécution estimé : ~4 ms sur Cortex-M7 @ 216 MHz
 * (FFT 4096 points flottante + overhead)
 */
void process_fft(void)
{
  /* === Étape 1 : DC removal === */
  float32_t mean;
  arm_mean_f32(fft_in, FFT_SIZE, &mean);
  for (int i = 0; i < FFT_SIZE; i++)
    fft_in[i] -= mean;

  /* === Étape 2 : Fenêtrage de Hann === */
  arm_mult_f32(fft_in, hann_window, fft_in, FFT_SIZE);

  /* === Étape 3 : FFT réelle 4096 points === */
  arm_rfft_fast_f32(&fft_instance, fft_in, fft_out, 0);

  /* === Étape 4 : Magnitudes === */
  arm_cmplx_mag_f32(fft_out, fft_mag, FFT_SIZE / 2);

  /* === Étape 5 : Recherche du pic spectral === */
  float32_t peak_val;
  uint32_t peak_idx;
  arm_max_f32(&fft_mag[MIN_BIN], MAX_BIN - MIN_BIN, &peak_val, &peak_idx);
  peak_idx += MIN_BIN;

  /* === Étape 6 : Estimation du plancher de bruit === */
  float32_t noise_sum = 0;
  int noise_count = 0;
  for (int i = MIN_BIN; i < MAX_BIN; i++)
  {
    if (i < (int)peak_idx - 5 || i > (int)peak_idx + 5)
    {
      noise_sum += fft_mag[i];
      noise_count++;
    }
  }
  float32_t noise_avg = (noise_count > 0) ? noise_sum / noise_count : 1e-12f;

  /* === Étape 7 : Calcul du SNR et des vitesses === */
  float32_t peak_db = 20.0f * log10f(peak_val + 1e-12f);
  float32_t noise_db = 20.0f * log10f(noise_avg + 1e-12f);
  float32_t snr = peak_db - noise_db;
  float32_t freq = (float32_t)peak_idx * FS / FFT_SIZE;
  float32_t speed_ms = freq / DOPPLER_SCALE;
  float32_t speed_kmh = speed_ms * 3.6f;

  uint16_t adc_pp = adc_max - adc_min;
  adc_min = 4095; adc_max = 0; adc_sum = 0; adc_count = 0;

  /* === Étape 8 : Décision de détection + LEDs === */
  if (snr > SNR_THRESHOLD && adc_pp > ADC_PP_THRESHOLD)
  {
    HAL_GPIO_WritePin(GPIOB, LD1_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOB, LD3_Pin, GPIO_PIN_RESET);
    last_speed_kmh = speed_kmh;
    last_speed_ms  = speed_ms;
    last_freq      = freq;
    last_snr       = snr;
    last_detected  = 1;
  }
  else
  {
    HAL_GPIO_WritePin(GPIOB, LD1_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB, LD3_Pin, GPIO_PIN_SET);
    last_snr      = snr;
    last_detected = 0;
  }

  /* === Étape 9 : Streaming UART (compatibilité ancien scope) ===
   * Protocole identique à la v1 : sync 0xAA55 + 1024 samples décimés
   * + sync 0xBB66 + freq/speed/snr/det. */
  uint8_t sync[2] = {0xAA, 0x55};
  HAL_UART_Transmit(&huart3, sync, 2, 5);

  uint16_t n_samples = ADC_STREAM_SAMPLES;
  HAL_UART_Transmit(&huart3, (uint8_t *)&n_samples, 2, 5);
  HAL_UART_Transmit(&huart3, (uint8_t *)stream_buf, n_samples * 2, 200);

  uint8_t fft_hdr[2] = {0xBB, 0x66};
  HAL_UART_Transmit(&huart3, fft_hdr, 2, 5);
  HAL_UART_Transmit(&huart3, (uint8_t *)&freq, 4, 5);
  HAL_UART_Transmit(&huart3, (uint8_t *)&speed_kmh, 4, 5);
  HAL_UART_Transmit(&huart3, (uint8_t *)&speed_ms, 4, 5);
  HAL_UART_Transmit(&huart3, (uint8_t *)&snr, 4, 5);

  uint8_t det = (snr > SNR_THRESHOLD && adc_pp > ADC_PP_THRESHOLD) ? 1 : 0;
  HAL_UART_Transmit(&huart3, &det, 1, 5);
}

/* USER CODE END 0 */

/**
  * @brief  Point d'entrée de l'application.
  * @retval int (ne retourne jamais car boucle infinie)
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* Activation du cache d'instructions Cortex-M7 (~2× plus rapide) */
  SCB_EnableICache();

  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configuration horloge : HSE 8 MHz → PLL → SYSCLK = 216 MHz */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialisation de tous les périphériques CubeMX */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ETH_Init();
  MX_USART3_UART_Init();
  MX_USB_OTG_FS_PCD_Init();
  MX_ADC1_Init();
  MX_DAC_Init();
  MX_TIM2_Init();
  MX_TIM4_Init();

  /* USER CODE BEGIN 2 */

  /* === Reconfiguration UART à 921600 bauds === */
  huart3.Init.BaudRate = 921600;
  if (HAL_UART_Init(&huart3) != HAL_OK) Error_Handler();

  /* === Reconfiguration Timer 2 pour 100 kHz ===
   * fAPB1_timer = 108 MHz, prescaler = 0
   * ARR = 1079 → 108 MHz / 1080 = 100 000 Hz */
  __HAL_TIM_SET_PRESCALER(&htim2, 0);
  __HAL_TIM_SET_AUTORELOAD(&htim2, 1079);

  /* === Initialisation I2C1 + OLED SSD1306 === */
  {
    GPIO_InitTypeDef gpio = {0};
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_I2C1_CLK_ENABLE();

    gpio.Pin       = GPIO_PIN_8 | GPIO_PIN_9;
    gpio.Mode      = GPIO_MODE_AF_OD;
    gpio.Pull      = GPIO_PULLUP;
    gpio.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
    gpio.Alternate = GPIO_AF4_I2C1;
    HAL_GPIO_Init(GPIOB, &gpio);

    hi2c1.Instance              = I2C1;
    hi2c1.Init.Timing           = 0x00702681;  /* 400 kHz @ APB1=54 MHz */
    hi2c1.Init.OwnAddress1      = 0;
    hi2c1.Init.AddressingMode   = I2C_ADDRESSINGMODE_7BIT;
    hi2c1.Init.DualAddressMode  = I2C_DUALADDRESS_DISABLE;
    hi2c1.Init.GeneralCallMode  = I2C_GENERALCALL_DISABLE;
    hi2c1.Init.NoStretchMode    = I2C_NOSTRETCH_DISABLE;
    if (HAL_I2C_Init(&hi2c1) != HAL_OK) Error_Handler();

    SSD1306_Init(&hi2c1);
    SSD1306_Clear();
    SSD1306_DrawString(16, 8,  "RADAR DOPPLER", 1);
    SSD1306_DrawString(34, 20, "24.125 GHz", 1);
    SSD1306_DrawString(10, 36, "FS=100k FFT=4096", 1);
    SSD1306_DrawString(20, 52, "ETH + UART", 1);
    SSD1306_Update();
    HAL_Delay(1000);
  }

  /* === Initialisation FFT CMSIS-DSP (4096 points) === */
  arm_rfft_fast_init_f32(&fft_instance, FFT_SIZE);
  init_hann_window();

  /* === Démarrage de l'acquisition ADC via DMA + Timer === */
  HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adc_buf, ADC_BUF_SIZE);
  HAL_TIM_Base_Start(&htim2);

  HAL_GPIO_WritePin(GPIOB, LD1_Pin, GPIO_PIN_SET);

  /* === Démarrage du lien Ethernet ===
   * Le MAC est initialisé par MX_ETH_Init(), on démarre le transceiver. */
  eth_start_link();

  /* === Configuration du DAC + DMA === */
  HAL_NVIC_DisableIRQ(TIM4_IRQn);
  sweep_start_tick = HAL_GetTick();
  update_dac_sweep();

  __HAL_RCC_DMA1_CLK_ENABLE();
  hdma_dac1.Instance = DMA1_Stream5;
  hdma_dac1.Init.Channel = DMA_CHANNEL_7;
  hdma_dac1.Init.Direction = DMA_MEMORY_TO_PERIPH;
  hdma_dac1.Init.PeriphInc = DMA_PINC_DISABLE;
  hdma_dac1.Init.MemInc = DMA_MINC_ENABLE;
  hdma_dac1.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
  hdma_dac1.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
  hdma_dac1.Init.Mode = DMA_CIRCULAR;
  hdma_dac1.Init.Priority = DMA_PRIORITY_LOW;
  hdma_dac1.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
  HAL_DMA_Init(&hdma_dac1);
  __HAL_LINKDMA(&hdac, DMA_Handle1, hdma_dac1);

  /* Timer 4 pour le DAC : aussi à 100 kHz maintenant
   * ARR = 1079 → 108 MHz / 1080 = 100 kHz */
  __HAL_TIM_SET_PRESCALER(&htim4, 0);
  __HAL_TIM_SET_AUTORELOAD(&htim4, 1079);
  HAL_TIM_Base_Start(&htim4);

  HAL_DAC_Start_DMA(&hdac, DAC_CHANNEL_1, (uint32_t *)dac_buf, DAC_BUF_SIZE, DAC_ALIGN_12B_R);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  /* === Boucle principale ===
   * Timing à FS = 100 kHz :
   * - Toutes les 10.24 ms : 1024 échantillons ADC prêts (DMA HT ou TC)
   *   et envoyés en hop UDP (~97.6 Hz)
   * - Toutes les 40.96 ms : 4096 échantillons accumulés → process_fft()
   *   (période FFT = FFT_SIZE / FS = 4096 / 100000 = 40.96 ms ≈ 24.4 Hz)
   * - Toutes les 200 ms : rafraîchissement OLED
   * - Vérification périodique du lien Ethernet (toutes les secondes) */
  uint32_t eth_check_tick = 0;

  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

    /* === Traitement de la première moitié du buffer DMA (indices 0..1023) === */
    if (adc_half_ready)
    {
      adc_half_ready = 0;

      for (int i = 0; i < ADC_BUF_SIZE / 2; i++)
      {
        uint16_t val = adc_buf[i];
        fft_in[fft_fill + i] = (float32_t)val;
        udp_hop_buf[i] = val;  /* hop brut UDP (1024 échantillons) */

        if ((fft_fill + i) % 4 == 0)
          stream_buf[(fft_fill + i) / 4] = val;

        if (val < adc_min) adc_min = val;
        if (val > adc_max) adc_max = val;
        adc_sum += val;
        adc_count++;
      }

      fft_fill += ADC_BUF_SIZE / 2;

      if (eth_link_up)
      {
        udp_send_adc_frame(udp_hop_buf, UDP_HOP_SAMPLES, udp_seq_counter++);
      }

      if (fft_fill >= FFT_SIZE)
      {
        fft_fill = 0;
        process_fft();
        update_dac_sweep();
        HAL_GPIO_TogglePin(GPIOB, LD2_Pin);
      }
    }

    /* === Traitement de la seconde moitié (indices 1024..2047) === */
    if (adc_full_ready)
    {
      adc_full_ready = 0;

      for (int i = 0; i < ADC_BUF_SIZE / 2; i++)
      {
        uint16_t val = adc_buf[ADC_BUF_SIZE / 2 + i];
        fft_in[fft_fill + i] = (float32_t)val;
        udp_hop_buf[i] = val;  /* hop brut UDP (1024 échantillons) */

        if ((fft_fill + i) % 4 == 0)
          stream_buf[(fft_fill + i) / 4] = val;

        if (val < adc_min) adc_min = val;
        if (val > adc_max) adc_max = val;
        adc_sum += val;
        adc_count++;
      }

      fft_fill += ADC_BUF_SIZE / 2;

      if (eth_link_up)
      {
        udp_send_adc_frame(udp_hop_buf, UDP_HOP_SAMPLES, udp_seq_counter++);
      }

      if (fft_fill >= FFT_SIZE)
      {
        fft_fill = 0;
        process_fft();
        update_dac_sweep();
        HAL_GPIO_TogglePin(GPIOB, LD2_Pin);
      }
    }

    /* === Rafraîchissement OLED (5 Hz) === */
    if (HAL_GetTick() - oled_last_update >= OLED_UPDATE_MS)
    {
      oled_last_update = HAL_GetTick();
      SSD1306_ShowRadar(last_speed_kmh, last_speed_ms,
                        last_freq, last_snr, last_detected);
    }

    /* === Vérification périodique du lien Ethernet (1 Hz) ===
     * On re-teste le PHY toutes les secondes pour détecter
     * le branchement/débranchement du câble RJ45. */
    if (HAL_GetTick() - eth_check_tick >= 1000)
    {
      eth_check_tick = HAL_GetTick();
      uint8_t new_link = eth_check_link();
      if (new_link && !eth_link_up)
      {
        /* Le câble vient d'être branché → redémarrer le MAC */
        HAL_ETH_Start(&heth);
      }
      eth_link_up = new_link;
    }

    }
  /* USER CODE END 3 */
}

/**
  * @brief  Configuration de l'horloge système.
  *
  * HSE (8 MHz) → PLL → SYSCLK = 216 MHz
  * APB1 = 54 MHz (timers = 108 MHz)
  * APB2 = 108 MHz (timers = 216 MHz)
  *
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 216;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 9;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_7) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief  Initialisation de l'ADC1.
  *
  * ADC1 canal 0 (PA0), 12 bits, trigger Timer 2 TRGO.
  * Temps d'échantillonnage : 3 cycles (le plus rapide possible).
  *   Horloge ADC = APB2/4 = 27 MHz
  *   Temps de conversion = (3 + 12) / 27 MHz = 555 ns
  *   Bien inférieur à la période de 10 µs (100 kHz).
  *
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T2_TRGO;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  /* 3 cycles : le temps d'échantillonnage le plus court.
   * À 100 kHz, chaque conversion dure (3+12)/27MHz = 555 ns << 10 µs.
   * Suffisant car le filtre passe-bas AFE à 6 kHz limite le slew rate. */
  sConfig.SamplingTime = ADC_SAMPLETIME_15CYCLES;  /* 15 cycles (556 ns @ 27 MHz)
    * suffisant pour 100 kHz (10 µs/ech). 3 cycles était trop court pour
    * charger le condensateur S&H avec l'impédance de sortie du radar IF */
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief  Initialisation du DAC.
  * @retval None
  */
static void MX_DAC_Init(void)
{

  /* USER CODE BEGIN DAC_Init 0 */

  /* USER CODE END DAC_Init 0 */

  DAC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN DAC_Init 1 */

  /* USER CODE END DAC_Init 1 */

  /** DAC Initialization
  */
  hdac.Instance = DAC;
  if (HAL_DAC_Init(&hdac) != HAL_OK)
  {
    Error_Handler();
  }

  /** DAC channel OUT1 config
  */
  sConfig.DAC_Trigger = DAC_TRIGGER_T4_TRGO;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  if (HAL_DAC_ConfigChannel(&hdac, &sConfig, DAC_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DAC_Init 2 */

  /* USER CODE END DAC_Init 2 */

}

/**
  * @brief  Initialisation de l'Ethernet.
  *
  * PHY LAN8742A présent sur la Nucleo-144.
  * Maintenant UTILISÉ pour le streaming UDP des données ADC brutes.
  *
  * @retval None
  */
static void MX_ETH_Init(void)
{

  /* USER CODE BEGIN ETH_Init 0 */

  /* USER CODE END ETH_Init 0 */

   static uint8_t MACAddr[6];

  /* USER CODE BEGIN ETH_Init 1 */

  /* USER CODE END ETH_Init 1 */
  heth.Instance = ETH;
  MACAddr[0] = 0x00;
  MACAddr[1] = 0x80;
  MACAddr[2] = 0xE1;
  MACAddr[3] = 0x00;
  MACAddr[4] = 0x00;
  MACAddr[5] = 0x00;
  heth.Init.MACAddr = &MACAddr[0];
  heth.Init.MediaInterface = HAL_ETH_RMII_MODE;
  heth.Init.TxDesc = DMATxDscrTab;
  heth.Init.RxDesc = DMARxDscrTab;
  heth.Init.RxBuffLen = 1524;

  /* USER CODE BEGIN MACADDRESS */

  /* USER CODE END MACADDRESS */

  if (HAL_ETH_Init(&heth) != HAL_OK)
  {
    Error_Handler();
  }

  memset(&TxConfig, 0 , sizeof(ETH_TxPacketConfig));
  TxConfig.Attributes = ETH_TX_PACKETS_FEATURES_CSUM | ETH_TX_PACKETS_FEATURES_CRCPAD;
  TxConfig.ChecksumCtrl = ETH_CHECKSUM_IPHDR_PAYLOAD_INSERT_PHDR_CALC;
  TxConfig.CRCPadCtrl = ETH_CRC_PAD_INSERT;
  /* USER CODE BEGIN ETH_Init 2 */

  /* USER CODE END ETH_Init 2 */

}

/**
  * @brief  Initialisation du Timer 2 (horloge ADC).
  *
  * CubeMX configure ARR = 2159 (50 kHz).
  * On reconfigure à ARR = 1079 (100 kHz) dans USER CODE BEGIN 2.
  *
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 2159;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief  Initialisation du Timer 4 (horloge DAC).
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 65535;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief  Initialisation de l'UART3 (115200 bauds par défaut, reconfiguré à 921600).
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief  Initialisation USB OTG FS (non utilisé).
  * @retval None
  */
static void MX_USB_OTG_FS_PCD_Init(void)
{

  /* USER CODE BEGIN USB_OTG_FS_Init 0 */

  /* USER CODE END USB_OTG_FS_Init 0 */

  /* USER CODE BEGIN USB_OTG_FS_Init 1 */

  /* USER CODE END USB_OTG_FS_Init 1 */
  hpcd_USB_OTG_FS.Instance = USB_OTG_FS;
  hpcd_USB_OTG_FS.Init.dev_endpoints = 6;
  hpcd_USB_OTG_FS.Init.speed = PCD_SPEED_FULL;
  hpcd_USB_OTG_FS.Init.dma_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.phy_itface = PCD_PHY_EMBEDDED;
  hpcd_USB_OTG_FS.Init.Sof_enable = ENABLE;
  hpcd_USB_OTG_FS.Init.low_power_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.lpm_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.vbus_sensing_enable = ENABLE;
  hpcd_USB_OTG_FS.Init.use_dedicated_ep1 = DISABLE;
  if (HAL_PCD_Init(&hpcd_USB_OTG_FS) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USB_OTG_FS_Init 2 */

  /* USER CODE END USB_OTG_FS_Init 2 */

}

/**
  * @brief  Activation horloges DMA.
  * @retval None
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

}

/**
  * @brief  Initialisation GPIO (LEDs, bouton, USB).
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LD1_Pin|LD3_Pin|LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(USB_PowerSwitchOn_GPIO_Port, USB_PowerSwitchOn_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : USER_Btn_Pin */
  GPIO_InitStruct.Pin = USER_Btn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USER_Btn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD1_Pin LD3_Pin LD2_Pin */
  GPIO_InitStruct.Pin = LD1_Pin|LD3_Pin|LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_PowerSwitchOn_Pin */
  GPIO_InitStruct.Pin = USB_PowerSwitchOn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(USB_PowerSwitchOn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_OverCurrent_Pin */
  GPIO_InitStruct.Pin = USB_OverCurrent_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USB_OverCurrent_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/**
 * @brief  Callback DMA "Half Complete" pour l'ADC1.
 *
 * Appelée quand les 1024 premiers échantillons de adc_buf[] sont prêts.
 * Le DMA continue à remplir la seconde moitié pendant le traitement.
 *
 * @param hadc  Pointeur vers le handle ADC
 */
void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef *hadc)
{
  if (hadc->Instance == ADC1) adc_half_ready = 1;
}

/**
 * @brief  Callback DMA "Transfer Complete" pour l'ADC1.
 *
 * Appelée quand les 1024 derniers échantillons sont prêts.
 *
 * @param hadc  Pointeur vers le handle ADC
 */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
  if (hadc->Instance == ADC1) adc_full_ready = 1;
}

/* USER CODE END 4 */

/**
  * @brief  Gestionnaire d'erreur global.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
