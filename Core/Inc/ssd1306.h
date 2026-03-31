/**
 * @file    ssd1306.h
 * @brief   Pilote SSD1306 OLED 128x64 I2C pour STM32 HAL
 * @note    Conçu pour le projet Radar Doppler (L3 EEA)
 *
 * Ce fichier d'en-tête définit l'API publique du pilote SSD1306.
 * Le SSD1306 est un contrôleur d'écran OLED monochrome très répandu,
 * utilisé dans les modules OLED 0.96" 128x64 pixels.
 *
 * Architecture du pilote :
 * - Un framebuffer de 1024 octets en RAM représente l'état de l'écran
 * - Les fonctions de dessin (DrawPixel, DrawString, etc.) modifient le framebuffer
 * - SSD1306_Update() envoie le framebuffer entier à l'écran via I2C
 * - Cette approche "framebuffer + flush" est standard pour les écrans OLED :
 *   elle évite les multiples transactions I2C individuelles et permet de
 *   composer une image complexe avant de l'afficher d'un coup
 *
 * Connexion physique :
 *   STM32 PB8 (SCL) ──── SCL du module OLED
 *   STM32 PB9 (SDA) ──── SDA du module OLED
 *   3.3V ──── VCC du module OLED
 *   GND  ──── GND du module OLED
 *   (Pull-ups 4.7kΩ optionnels sur SCL/SDA, les pull-ups internes suffisent)
 */
#ifndef SSD1306_H
#define SSD1306_H

#include "stm32f7xx_hal.h"
#include <stdint.h>
#include <string.h>

/* ═══════════════════════════════════════════════════════════════════════════ */
/*  Configuration de l'écran                                                  */
/* ═══════════════════════════════════════════════════════════════════════════ */

/* Dimensions de l'écran en pixels.
 * Le SSD1306 supporte des résolutions jusqu'à 128×64.
 * Pour un module 0.96" standard, c'est toujours 128×64. */
#define SSD1306_WIDTH       128
#define SSD1306_HEIGHT      64

/* Nombre de "pages" mémoire.
 * Le SSD1306 organise sa mémoire graphique (GDDRAM) en pages de 8 lignes.
 * Pour 64 lignes de haut → 64/8 = 8 pages (page 0 à page 7).
 * Chaque page contient 128 octets (un par colonne).
 * Chaque octet représente 8 pixels verticaux (bit 0 = pixel du haut). */
#define SSD1306_PAGES       (SSD1306_HEIGHT / 8)

/* Taille du framebuffer en octets : 128 colonnes × 8 pages = 1024 octets.
 * C'est la quantité de données envoyée à l'écran lors de chaque Update(). */
#define SSD1306_BUF_SIZE    (SSD1306_WIDTH * SSD1306_PAGES)

/* ═══════════════════════════════════════════════════════════════════════════ */
/*  Adresse I2C                                                               */
/* ═══════════════════════════════════════════════════════════════════════════ */

/* Adresse I2C du SSD1306.
 * L'adresse 7 bits est 0x3C (défaut quand la broche SA0 est à GND).
 * Certains modules utilisent 0x3D (SA0 = VCC).
 *
 * Le HAL de ST utilise des adresses 8 bits (adresse 7 bits décalée d'1 bit
 * à gauche, le bit 0 étant le bit R/W). Donc 0x3C << 1 = 0x78.
 * Le HAL gère automatiquement le bit R/W selon l'opération (lecture/écriture). */
#define SSD1306_I2C_ADDR    (0x3C << 1)

/* ═══════════════════════════════════════════════════════════════════════════ */
/*  Octets de contrôle I2C (datasheet SSD1306, Figure 8-7)                    */
/* ═══════════════════════════════════════════════════════════════════════════ */

/* Après l'adresse I2C, le SSD1306 attend un "octet de contrôle" qui indique
 * si les octets suivants sont des commandes ou des données d'affichage.
 *
 * L'octet de contrôle a deux bits importants :
 *   Bit 7 (Co) : Continuation. 0 = les octets suivants sont tous du même type
 *   Bit 6 (D/C#) : Data/Command. 0 = commande, 1 = donnée GDDRAM
 *
 * 0x00 = Co=0, D/C#=0 → les octets suivants sont des commandes
 * 0x40 = Co=0, D/C#=1 → les octets suivants sont des données d'affichage */
#define SSD1306_CTRL_CMD    0x00
#define SSD1306_CTRL_DATA   0x40

/* ═══════════════════════════════════════════════════════════════════════════ */
/*  Commandes du SSD1306 (table de commandes, datasheet p.28-32)              */
/* ═══════════════════════════════════════════════════════════════════════════ */

/* --- Commandes d'affichage ON/OFF --- */
/* AEh : éteindre l'affichage (OLED OFF). Les pixels sont tous éteints mais
 * le contrôleur continue de fonctionner. Utilisé pendant l'initialisation. */
#define SSD1306_CMD_DISPLAY_OFF       0xAE
/* AFh : allumer l'affichage (OLED ON). Les pixels affichent le contenu
 * de la GDDRAM. C'est la dernière commande de la séquence d'init. */
#define SSD1306_CMD_DISPLAY_ON        0xAF

/* --- Contraste --- */
/* 81h + [valeur] : règle le contraste (luminosité) de 0x00 (min) à 0xFF (max).
 * Le contraste contrôle le courant dans chaque segment OLED.
 * Valeur typique : 0xCF (élevé, bonne lisibilité). */
#define SSD1306_CMD_SET_CONTRAST      0x81

/* --- Mode d'affichage --- */
/* A4h : affichage normal (les pixels suivent le contenu de la GDDRAM).
 * A5h : tous les pixels allumés (test de l'écran, ignore la GDDRAM). */
#define SSD1306_CMD_DISPLAY_RAM       0xA4
#define SSD1306_CMD_DISPLAY_ALL_ON    0xA5

/* A6h : affichage normal (pixel ON = blanc, pixel OFF = noir).
 * A7h : affichage inversé (pixel ON = noir, pixel OFF = blanc). */
#define SSD1306_CMD_NORMAL_DISPLAY    0xA6
#define SSD1306_CMD_INVERSE_DISPLAY   0xA7

/* --- Configuration du multiplex --- */
/* A8h + [ratio] : nombre de lignes COM actives - 1.
 * Pour 128×64 : ratio = 63 (0x3F), ce qui active les 64 lignes. */
#define SSD1306_CMD_SET_MUX_RATIO     0xA8

/* --- Offset vertical --- */
/* D3h + [offset] : décale l'affichage verticalement de N lignes.
 * Offset = 0 en utilisation normale. */
#define SSD1306_CMD_SET_DISPLAY_OFFSET 0xD3

/* --- Ligne de départ --- */
/* 40h-7Fh : la ligne de la GDDRAM qui correspond à la première ligne
 * affichée à l'écran. Les bits 0-5 encodent la ligne (0-63).
 * 40h = ligne 0 (normal). Peut servir au scrolling hardware. */
#define SSD1306_CMD_SET_START_LINE    0x40

/* --- Remapping des segments et scan COM --- */
/* A1h : la colonne 127 de la GDDRAM est mappée sur le segment 0 (SEG0)
 * de l'écran. Cela "flippe" l'affichage horizontalement.
 * Combiné avec C8h, cela permet d'orienter correctement l'image
 * pour un module OLED monté dans un sens ou l'autre. */
#define SSD1306_CMD_SET_SEG_REMAP     0xA1

/* C8h : scan des lignes COM de COM63 vers COM0 (décrémentant).
 * Cela "flippe" l'affichage verticalement. */
#define SSD1306_CMD_SET_COM_SCAN_DEC  0xC8

/* --- Configuration des pins COM --- */
/* DAh + [config] : configuration du hardware COM (câblage interne des lignes).
 * 0x12 = alternative COM pin configuration (pour 128×64).
 * 0x02 = sequential COM pin configuration (pour 128×32). */
#define SSD1306_CMD_SET_COM_PINS      0xDA

/* --- Horloge et oscillateur --- */
/* D5h + [config] : bits 7-4 = fréquence de l'oscillateur (0-15),
 * bits 3-0 = diviseur d'horloge (1 à 16).
 * 0x80 = fréquence moyenne, diviseur = 1 (valeur par défaut recommandée). */
#define SSD1306_CMD_SET_CLK_DIV       0xD5

/* --- Périodes de pré-charge --- */
/* D9h + [config] : bits 7-4 = phase 2 (pré-charge), bits 3-0 = phase 1.
 * Ces paramètres contrôlent le timing de charge/décharge des pixels OLED.
 * 0xF1 = phase1=1, phase2=15 (optimisé pour charge pump interne). */
#define SSD1306_CMD_SET_PRECHARGE     0xD9

/* --- Niveau de désélection VCOMH --- */
/* DBh + [level] : tension de désélection des segments COM.
 * 0x40 = ~0.77 × VCC. Affecte le contraste et la durée de vie. */
#define SSD1306_CMD_SET_VCOMH         0xDB

/* --- Charge Pump (pompe de charge) --- */
/* 8Dh + [0x14] : active la pompe de charge intégrée.
 * La pompe de charge génère la haute tension (~7V) nécessaire pour
 * alimenter les pixels OLED à partir du 3.3V d'entrée.
 * OBLIGATOIRE pour les modules OLED qui n'ont pas de régulateur externe.
 * 0x14 = pompe activée, 0x10 = pompe désactivée. */
#define SSD1306_CMD_CHARGE_PUMP       0x8D

/* --- Mode d'adressage mémoire --- */
/* 20h + [mode] : définit comment le pointeur d'adresse de la GDDRAM
 * avance après chaque écriture de données.
 *   00b = Horizontal : colonne++, puis page++ quand fin de ligne (le plus efficace
 *         pour un transfert plein écran en une seule transaction I2C)
 *   01b = Vertical : page++, puis colonne++ quand fin de colonne
 *   10b = Page : colonne++ dans la page courante seulement */
#define SSD1306_CMD_MEM_ADDR_MODE     0x20

/* --- Commandes d'adressage (mode horizontal) --- */
/* 21h + [start_col] + [end_col] : définit la plage de colonnes pour l'écriture.
 * 22h + [start_page] + [end_page] : définit la plage de pages pour l'écriture.
 * Pour un transfert plein écran : colonnes 0-127, pages 0-7. */
#define SSD1306_CMD_SET_COL_ADDR      0x21
#define SSD1306_CMD_SET_PAGE_ADDR     0x22

/* ═══════════════════════════════════════════════════════════════════════════ */
/*  Échelles de police pour l'affichage de la vitesse                         */
/* ═══════════════════════════════════════════════════════════════════════════ */

/* La police de base est une grille 6×8 pixels (5 colonnes de données + 1 espacement).
 * Les facteurs d'échelle multiplient la taille :
 *   SMALL (1×) → 6×8 pixels par caractère (texte détail, ~21 caractères par ligne)
 *   BIG   (3×) → 18×24 pixels par caractère (vitesse en gros, ~7 caractères par ligne)
 *
 * L'agrandissement se fait par "pixel block expansion" : chaque pixel de la
 * police est remplacé par un carré de scale×scale pixels dans le framebuffer. */
#define FONT_SCALE_SMALL    1
#define FONT_SCALE_BIG      3

/* ═══════════════════════════════════════════════════════════════════════════ */
/*  API publique                                                              */
/* ═══════════════════════════════════════════════════════════════════════════ */

/**
 * @brief  Initialise l'écran SSD1306 via I2C.
 *
 * Envoie la séquence complète d'initialisation (15+ commandes) qui configure :
 * - L'oscillateur et le timing d'affichage
 * - Le multiplex ratio (64 lignes)
 * - La charge pump (génération de la haute tension OLED)
 * - Le mode d'adressage horizontal
 * - Le remapping segments/COM (orientation correcte de l'image)
 * - Le contraste
 * Puis efface le framebuffer, l'envoie à l'écran, et allume l'affichage.
 *
 * @param  hi2c  Pointeur vers le handle I2C (doit être déjà initialisé)
 * @retval HAL_OK si l'initialisation a réussi
 */
HAL_StatusTypeDef SSD1306_Init(I2C_HandleTypeDef *hi2c);

/**
 * @brief  Efface le framebuffer (tous les pixels à OFF/noir).
 *
 * Met les 1024 octets du framebuffer à 0x00 (memset).
 * L'écran n'est pas mis à jour automatiquement : il faut appeler
 * SSD1306_Update() ensuite pour que l'effacement soit visible.
 */
void SSD1306_Clear(void);

/**
 * @brief  Remplit le framebuffer (tous les pixels à ON/blanc).
 *
 * Met les 1024 octets à 0xFF. Utile pour tester l'écran.
 */
void SSD1306_Fill(void);

/**
 * @brief  Envoie le framebuffer complet à l'écran via I2C.
 *
 * Séquence :
 * 1. Commande SET_COL_ADDR (0x21) : colonnes 0 à 127
 * 2. Commande SET_PAGE_ADDR (0x22) : pages 0 à 7
 * 3. Envoi des 1024 octets de données du framebuffer
 *
 * Le mode d'adressage horizontal fait avancer automatiquement
 * le pointeur d'écriture : colonne par colonne, puis page suivante.
 * Ainsi, un seul envoi I2C de 1024 octets suffit pour tout l'écran.
 *
 * Durée typique : ~20 ms à 400 kHz I2C
 * (1024 octets × 8 bits × 1/400000 ≈ 20.5 ms + overhead adressage)
 *
 * @retval HAL_OK si le transfert a réussi
 */
HAL_StatusTypeDef SSD1306_Update(void);

/**
 * @brief  Allume ou éteint un pixel individuel dans le framebuffer.
 *
 * Chaque octet du framebuffer représente 8 pixels verticaux dans une page.
 * Pour modifier un pixel à la position (x, y) :
 *   index = x + (y / 8) × 128  → position de l'octet dans le framebuffer
 *   bit = y % 8                 → position du bit dans cet octet
 *
 * @param  x      Colonne (0 à 127, 0 = gauche)
 * @param  y      Ligne (0 à 63, 0 = haut)
 * @param  color  1 = blanc (pixel ON), 0 = noir (pixel OFF)
 */
void SSD1306_DrawPixel(uint8_t x, uint8_t y, uint8_t color);

/**
 * @brief  Dessine un caractère ASCII à une position donnée.
 *
 * Utilise la police bitmap 6×8 définie dans ssd1306_fonts.h.
 * Chaque caractère est défini par 6 octets (5 colonnes de pixels + 1 espacement),
 * format "column-major" : chaque octet représente une colonne de 8 pixels,
 * bit 0 = pixel du haut, bit 7 = pixel du bas.
 *
 * Le paramètre "scale" permet d'agrandir le caractère :
 * - scale=1 : taille native 6×8 pixels
 * - scale=2 : 12×16 pixels (chaque pixel source → carré 2×2)
 * - scale=3 : 18×24 pixels (chaque pixel source → carré 3×3)
 *
 * @param  x      Colonne du coin supérieur gauche
 * @param  y      Ligne du coin supérieur gauche
 * @param  ch     Caractère ASCII (32-126). Hors plage → remplacé par '?'
 * @param  scale  Facteur d'agrandissement (1, 2, ou 3)
 */
void SSD1306_DrawChar(uint8_t x, uint8_t y, char ch, uint8_t scale);

/**
 * @brief  Dessine une chaîne de caractères (terminée par '\0').
 *
 * Dessine chaque caractère de la chaîne côte à côte, en avançant
 * de (6 × scale) pixels à droite après chaque caractère.
 * S'arrête si le prochain caractère dépasserait le bord droit de l'écran.
 *
 * @param  x      Colonne de départ
 * @param  y      Ligne de départ
 * @param  str    Chaîne C (null-terminated)
 * @param  scale  Facteur d'agrandissement de la police
 */
void SSD1306_DrawString(uint8_t x, uint8_t y, const char *str, uint8_t scale);

/**
 * @brief  Dessine une ligne horizontale.
 *
 * Trace une ligne de w pixels de large à la position (x, y).
 * Utilise DrawPixel en boucle (simple mais suffisant pour les séparateurs).
 *
 * @param  x      Colonne de départ
 * @param  y      Ligne
 * @param  w      Largeur en pixels
 * @param  color  1 = blanc, 0 = noir
 */
void SSD1306_DrawHLine(uint8_t x, uint8_t y, uint8_t w, uint8_t color);

/**
 * @brief  Dessine un rectangle rempli.
 *
 * Remplit un rectangle de w×h pixels à la position (x, y).
 * Utilise DrawPixel en double boucle.
 *
 * @param  x, y   Coin supérieur gauche
 * @param  w, h   Largeur et hauteur en pixels
 * @param  color  1 = blanc, 0 = noir
 */
void SSD1306_FillRect(uint8_t x, uint8_t y, uint8_t w, uint8_t h, uint8_t color);

/* ═══════════════════════════════════════════════════════════════════════════ */
/*  Fonctions d'affichage spécifiques au radar                                */
/* ═══════════════════════════════════════════════════════════════════════════ */

/**
 * @brief  Affiche les mesures radar sur l'OLED.
 *
 * Cette fonction gère la mise en page complète de l'écran :
 *
 *   ┌────────────────────────────────┐
 *   │  RADAR DOPPLER 24GHz    (p0)  │  ← titre, police 1×
 *   │────────────────────────────────│  ← ligne séparatrice
 *   │     1 2 3 . 4                 │  ← vitesse en km/h, police 3×
 *   │                          km/h │
 *   │────────────────────────────────│  ← séparateur
 *   │  34.3 m/s   5523.4 Hz        │  ← détails, police 1×
 *   │  SNR: 23.1 dB                │
 *   └────────────────────────────────┘
 *
 * Si pas de cible détectée, affiche "--.-" et "NO TARGET".
 *
 * Appelle SSD1306_Update() en interne → l'écran est rafraîchi à chaque appel.
 *
 * @param  speed_kmh   Vitesse en km/h (ignorée si detected=0)
 * @param  speed_ms    Vitesse en m/s
 * @param  freq_hz     Fréquence Doppler en Hz
 * @param  snr_db      Rapport signal/bruit en dB
 * @param  detected    1 = cible détectée, 0 = pas de cible
 */
void SSD1306_ShowRadar(float speed_kmh, float speed_ms,
                       float freq_hz, float snr_db, uint8_t detected);

#endif /* SSD1306_H */
