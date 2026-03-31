/**
 * @file    ssd1306.c
 * @brief   Implémentation du pilote SSD1306 OLED 128x64 I2C
 *
 * Ce pilote utilise le mode d'adressage horizontal du SSD1306 pour des
 * mises à jour plein écran efficaces. L'affichage est piloté depuis un
 * framebuffer local de 1024 octets en RAM.
 *
 * Architecture :
 * 1. Les fonctions de dessin (DrawPixel, DrawString, etc.) modifient le
 *    framebuffer en RAM sans toucher au matériel I2C
 * 2. SSD1306_Update() envoie les 1024 octets du framebuffer à l'écran
 *    en une seule transaction I2C
 *
 * Cette approche "draw + flush" est la plus efficace car :
 * - On fait une seule transaction I2C au lieu de N petites
 * - On peut composer des images complexes sans scintillement
 * - La bande passante I2C est utilisée de manière optimale
 *
 * Temps de transfert I2C à 400 kHz (Fast Mode) :
 *   1024 octets × 8 bits / 400000 ≈ 20.5 ms (+ overhead adressage et ACK)
 *   → ~25 ms réel, soit ~40 fps maximum théorique
 *   En pratique on limite à 5 fps (OLED_UPDATE_MS = 200 ms dans main.c)
 *   pour ne pas surcharger le CPU
 */

#include "ssd1306.h"
#include "ssd1306_fonts.h"
#include <stdio.h>

/* ═══════════════════════════════════════════════════════════════════════════ */
/*  État privé du pilote                                                      */
/* ═══════════════════════════════════════════════════════════════════════════ */

/* Pointeur vers le handle I2C utilisé pour communiquer avec l'écran.
 * Initialisé une fois par SSD1306_Init() et réutilisé par toutes les
 * fonctions d'écriture. */
static I2C_HandleTypeDef *ssd1306_i2c;

/* Framebuffer : représentation en RAM de l'écran OLED.
 * Organisation : 8 pages × 128 colonnes = 1024 octets.
 *
 * La mémoire est organisée en "pages" de 8 pixels de haut :
 *   Page 0 : lignes 0-7    (128 octets)
 *   Page 1 : lignes 8-15   (128 octets)
 *   ...
 *   Page 7 : lignes 56-63  (128 octets)
 *
 * Dans chaque octet :
 *   Bit 0 = pixel du haut de la page (ligne 0 de la page)
 *   Bit 7 = pixel du bas de la page (ligne 7 de la page)
 *   1 = pixel allumé (blanc), 0 = pixel éteint (noir)
 *
 * Exemple : pour allumer le pixel à (x=10, y=20) :
 *   Page = 20/8 = 2
 *   Bit  = 20%8 = 4
 *   Index = 10 + 2×128 = 266
 *   framebuf[266] |= (1 << 4) */
static uint8_t framebuf[SSD1306_BUF_SIZE];

/* ═══════════════════════════════════════════════════════════════════════════ */
/*  Fonctions I2C bas niveau                                                  */
/* ═══════════════════════════════════════════════════════════════════════════ */

/**
 * @brief  Envoie un octet de commande au SSD1306 via I2C.
 *
 * Protocole I2C pour une commande :
 *   [START] [adresse 0x78 + W] [ACK] [0x00 (contrôle=commande)] [ACK] [cmd] [ACK] [STOP]
 *
 * L'octet de contrôle 0x00 (SSD1306_CTRL_CMD) indique au SSD1306 que
 * l'octet suivant est une commande (pas une donnée d'affichage).
 *
 * @param cmd  Octet de commande à envoyer (voir les defines SSD1306_CMD_*)
 * @return     HAL_OK si la transmission a réussi
 */
static HAL_StatusTypeDef SSD1306_WriteCmd(uint8_t cmd)
{
  /* On prépare un buffer de 2 octets : [octet de contrôle] [commande]
   * et on les envoie en une seule transaction I2C. */
  uint8_t buf[2] = { SSD1306_CTRL_CMD, cmd };
  return HAL_I2C_Master_Transmit(ssd1306_i2c, SSD1306_I2C_ADDR,
                                  buf, 2, 10);
}

/**
 * @brief  Envoie un bloc de données d'affichage au SSD1306 (GDDRAM).
 *
 * Protocole I2C pour des données :
 *   [START] [adresse 0x78 + W] [ACK] [0x40 (contrôle=data)] [ACK]
 *   [data0] [ACK] [data1] [ACK] ... [dataN] [ACK] [STOP]
 *
 * L'octet de contrôle 0x40 (SSD1306_CTRL_DATA) indique que tous les
 * octets suivants sont des données pour la GDDRAM (pixels).
 *
 * On utilise HAL_I2C_Mem_Write() qui est un raccourci pour envoyer :
 *   [adresse I2C] [adresse mémoire = 0x40] [données...]
 * C'est plus pratique que de prépendre manuellement l'octet de contrôle.
 *
 * @param data  Pointeur vers les données à envoyer
 * @param len   Nombre d'octets à envoyer
 * @return      HAL_OK si la transmission a réussi
 */
static HAL_StatusTypeDef SSD1306_WriteData(uint8_t *data, uint16_t len)
{
  return HAL_I2C_Mem_Write(ssd1306_i2c, SSD1306_I2C_ADDR,
                            SSD1306_CTRL_DATA, I2C_MEMADD_SIZE_8BIT,
                            data, len, 100);
}

/* ═══════════════════════════════════════════════════════════════════════════ */
/*  API publique                                                              */
/* ═══════════════════════════════════════════════════════════════════════════ */

/**
 * @brief  Séquence d'initialisation complète du SSD1306.
 *
 * Cette séquence est tirée de la datasheet SSD1306 (section 8.9) et
 * des notes d'application pour les modules OLED 0.96" 128×64.
 *
 * L'ordre des commandes est important :
 * 1. Display OFF (pendant la configuration)
 * 2. Configuration du timing (horloge, oscillateur)
 * 3. Configuration physique (multiplex, offset, COM pins, segment remap)
 * 4. Configuration de la charge pump (génération 7V pour les OLED)
 * 5. Mode d'adressage mémoire (horizontal pour les transferts efficaces)
 * 6. Contraste et pré-charge
 * 7. Effacement du framebuffer et envoi à l'écran
 * 8. Display ON
 *
 * @param hi2c  Handle I2C déjà initialisé (HAL_I2C_Init doit avoir été appelé)
 * @return      HAL_OK si toute la séquence a réussi
 */
HAL_StatusTypeDef SSD1306_Init(I2C_HandleTypeDef *hi2c)
{
  HAL_StatusTypeDef ret;
  ssd1306_i2c = hi2c;

  /* Attente de stabilisation des alimentations (VDD et VCC).
   * Le datasheet recommande au moins 100 ms après la mise sous tension
   * avant d'envoyer des commandes I2C. */
  HAL_Delay(100);

  /* ── Séquence d'initialisation ── */

  /* Display OFF pendant la configuration.
   * On ne veut pas afficher du bruit aléatoire pendant qu'on configure
   * les paramètres du contrôleur. */
  ret = SSD1306_WriteCmd(SSD1306_CMD_DISPLAY_OFF);   /* AEh */
  if (ret != HAL_OK) return ret;

  /* Configuration de l'oscillateur interne et du diviseur d'horloge.
   * 0x80 = oscillateur à la fréquence par défaut, diviseur = 1.
   * L'horloge interne cadence le rafraîchissement de l'affichage OLED
   * (scan des lignes COM et envoi des données aux segments). */
  SSD1306_WriteCmd(SSD1306_CMD_SET_CLK_DIV);          /* D5h */
  SSD1306_WriteCmd(0x80);

  /* Multiplex ratio : 64 lignes actives (pour un écran 128×64).
   * 0x3F = 63 = nombre de lignes COM - 1.
   * Pour un écran 128×32, on mettrait 0x1F (31). */
  SSD1306_WriteCmd(SSD1306_CMD_SET_MUX_RATIO);        /* A8h */
  SSD1306_WriteCmd(0x3F);

  /* Display offset : pas de décalage vertical (0 lignes).
   * Pourrait être utilisé pour "scroller" l'image verticalement. */
  SSD1306_WriteCmd(SSD1306_CMD_SET_DISPLAY_OFFSET);   /* D3h */
  SSD1306_WriteCmd(0x00);

  /* Start line : la ligne 0 de la GDDRAM correspond à la première
   * ligne physique de l'écran. */
  SSD1306_WriteCmd(SSD1306_CMD_SET_START_LINE | 0x00); /* 40h */

  /* Charge Pump : ACTIVATION OBLIGATOIRE.
   * La charge pump interne génère la haute tension (~7V) nécessaire
   * pour piloter les OLED à partir du 3.3V d'entrée.
   * Sans cette activation, l'écran reste complètement noir. */
  SSD1306_WriteCmd(SSD1306_CMD_CHARGE_PUMP);           /* 8Dh */
  SSD1306_WriteCmd(0x14);  /* 0x14 = charge pump ON */

  /* Mode d'adressage HORIZONTAL.
   * Dans ce mode, après chaque écriture de donnée, le pointeur de colonne
   * s'incrémente automatiquement. Quand il atteint la fin de la ligne (127),
   * il revient à 0 et le pointeur de page s'incrémente.
   * Cela permet d'envoyer tout le framebuffer (1024 octets) en une seule
   * transaction I2C continue, ce qui est la méthode la plus efficace. */
  SSD1306_WriteCmd(SSD1306_CMD_MEM_ADDR_MODE);         /* 20h */
  SSD1306_WriteCmd(0x00);  /* 00b = horizontal */

  /* Segment remap : flip horizontal.
   * A1h = la colonne 127 de la GDDRAM est envoyée au segment 0 de l'écran.
   * Nécessaire pour que l'orientation de l'image soit correcte selon
   * le montage physique du module OLED. */
  SSD1306_WriteCmd(SSD1306_CMD_SET_SEG_REMAP);         /* A1h */

  /* COM scan direction : flip vertical.
   * C8h = scan de COM63 vers COM0 (décrémentant).
   * Combiné avec A1h, cela oriente l'image correctement pour la plupart
   * des modules OLED où le connecteur FPC est en bas. */
  SSD1306_WriteCmd(SSD1306_CMD_SET_COM_SCAN_DEC);      /* C8h */

  /* Configuration des pins COM : mode alternatif pour 128×64.
   * Le SSD1306 peut câbler les lignes COM de manière séquentielle
   * ou alternée. Le mode alternatif (0x12) est standard pour 128×64. */
  SSD1306_WriteCmd(SSD1306_CMD_SET_COM_PINS);          /* DAh */
  SSD1306_WriteCmd(0x12);

  /* Contraste élevé (0xCF sur une plage de 0x00 à 0xFF).
   * Le contraste détermine le courant qui traverse chaque pixel OLED.
   * Plus le contraste est élevé, plus l'écran est lumineux et net,
   * mais plus la consommation est élevée (~15 mA à 0xCF vs ~5 mA à 0x00). */
  SSD1306_WriteCmd(SSD1306_CMD_SET_CONTRAST);          /* 81h */
  SSD1306_WriteCmd(0xCF);

  /* Périodes de pré-charge : phase1=1 cycle, phase2=15 cycles.
   * La pré-charge est le temps pendant lequel les segments sont chargés
   * avant d'être connectés aux lignes COM. Des valeurs plus élevées
   * pour phase2 donnent un meilleur contraste avec la charge pump interne. */
  SSD1306_WriteCmd(SSD1306_CMD_SET_PRECHARGE);         /* D9h */
  SSD1306_WriteCmd(0xF1);

  /* Niveau VCOMH : ~0.77 × VCC.
   * Tension de désélection des lignes COM. Affecte le contraste global. */
  SSD1306_WriteCmd(SSD1306_CMD_SET_VCOMH);             /* DBh */
  SSD1306_WriteCmd(0x40);

  /* Mode d'affichage normal : les pixels suivent la GDDRAM.
   * (par opposition au mode "all on" qui allume tout pour tester) */
  SSD1306_WriteCmd(SSD1306_CMD_DISPLAY_RAM);           /* A4h */

  /* Affichage non inversé : 1 dans la GDDRAM = pixel blanc, 0 = noir. */
  SSD1306_WriteCmd(SSD1306_CMD_NORMAL_DISPLAY);        /* A6h */

  /* Désactivation du scrolling hardware.
   * Le SSD1306 a un mode de défilement matériel (scroll horizontal/vertical)
   * mais on ne l'utilise pas — tout le scrolling est géré par logiciel. */
  SSD1306_WriteCmd(0x2E);

  /* Effacement du framebuffer et envoi à l'écran pour partir d'un état propre */
  SSD1306_Clear();
  SSD1306_Update();

  /* Allumage de l'affichage (dernière commande de la séquence d'init). */
  SSD1306_WriteCmd(SSD1306_CMD_DISPLAY_ON);            /* AFh */

  return HAL_OK;
}

/**
 * @brief  Efface le framebuffer (tous les octets à 0x00 = tous les pixels éteints).
 */
void SSD1306_Clear(void)
{
  memset(framebuf, 0x00, SSD1306_BUF_SIZE);
}

/**
 * @brief  Remplit le framebuffer (tous les octets à 0xFF = tous les pixels allumés).
 */
void SSD1306_Fill(void)
{
  memset(framebuf, 0xFF, SSD1306_BUF_SIZE);
}

/**
 * @brief  Transfère le framebuffer complet vers la GDDRAM du SSD1306.
 *
 * En mode d'adressage horizontal, on définit d'abord la fenêtre d'écriture
 * (colonnes 0-127, pages 0-7), puis on envoie les 1024 octets en continu.
 * Le pointeur d'adresse interne du SSD1306 avance automatiquement :
 * colonne 0→1→...→127→retour à 0, page 0→1→...→7.
 *
 * @return HAL_OK si le transfert I2C a réussi
 */
HAL_StatusTypeDef SSD1306_Update(void)
{
  /* Définir la fenêtre d'écriture : colonnes 0 à 127 */
  SSD1306_WriteCmd(SSD1306_CMD_SET_COL_ADDR);
  SSD1306_WriteCmd(0x00);   /* colonne de début = 0 */
  SSD1306_WriteCmd(0x7F);   /* colonne de fin = 127 */

  /* Définir la fenêtre d'écriture : pages 0 à 7 */
  SSD1306_WriteCmd(SSD1306_CMD_SET_PAGE_ADDR);
  SSD1306_WriteCmd(0x00);   /* page de début = 0 */
  SSD1306_WriteCmd(0x07);   /* page de fin = 7 */

  /* Envoi des 1024 octets du framebuffer en une seule transaction I2C.
   * C'est la partie la plus longue (~20 ms à 400 kHz). */
  return SSD1306_WriteData(framebuf, SSD1306_BUF_SIZE);
}

/**
 * @brief  Allume ou éteint un pixel individuel dans le framebuffer.
 *
 * Algorithme de calcul de la position :
 *   page = y / 8        → quelle page (groupe de 8 lignes) contient ce pixel
 *   bit  = y % 8        → quel bit dans l'octet de cette page
 *   index = x + page×128 → position de l'octet dans le framebuffer linéaire
 *
 * Pour allumer le pixel : OU logique avec un masque (1 << bit)
 * Pour éteindre le pixel : ET logique avec le complément du masque
 *
 * @param x      Colonne (0-127). Si >= 128, le pixel est ignoré (clipping).
 * @param y      Ligne (0-63). Si >= 64, le pixel est ignoré (clipping).
 * @param color  1 pour allumer (blanc), 0 pour éteindre (noir)
 */
void SSD1306_DrawPixel(uint8_t x, uint8_t y, uint8_t color)
{
  /* Clipping : ignorer les pixels hors écran */
  if (x >= SSD1306_WIDTH || y >= SSD1306_HEIGHT) return;

  /* Calcul de l'index dans le framebuffer et du masque de bit */
  uint16_t idx = x + (y / 8) * SSD1306_WIDTH;
  uint8_t  bit = y % 8;

  if (color)
    framebuf[idx] |=  (1 << bit);   /* SET : allumer le pixel */
  else
    framebuf[idx] &= ~(1 << bit);   /* CLEAR : éteindre le pixel */
}

/**
 * @brief  Dessine un caractère avec agrandissement variable.
 *
 * La police source est stockée dans font_6x8[][] (ssd1306_fonts.h).
 * Chaque caractère fait 6 colonnes × 8 lignes (5 colonnes de données
 * + 1 colonne d'espacement vide).
 *
 * L'agrandissement fonctionne par "pixel block expansion" :
 * pour scale=3, chaque pixel de la police (qui est soit ON soit OFF)
 * est remplacé par un carré de 3×3 pixels dans le framebuffer.
 * Cela donne un effet "gros pixel" propre et lisible.
 *
 * Pour scale=1, on utilise un chemin optimisé (un seul DrawPixel par pixel)
 * au lieu de la double boucle scale×scale.
 *
 * @param x      Coin supérieur gauche, colonne
 * @param y      Coin supérieur gauche, ligne
 * @param ch     Caractère ASCII à dessiner (32-126)
 * @param scale  Facteur d'agrandissement (1, 2, ou 3)
 */
void SSD1306_DrawChar(uint8_t x, uint8_t y, char ch, uint8_t scale)
{
  /* Conversion ASCII → index dans la table de police.
   * La table commence au caractère espace (ASCII 32).
   * Si le caractère est hors plage, on affiche '?' */
  if (ch < 32 || ch > 126) ch = '?';
  uint8_t idx = ch - 32;

  /* Parcours des 6 colonnes de la police */
  for (uint8_t col = 0; col < 6; col++)
  {
    /* Lecture de la colonne de pixels : 1 octet = 8 pixels verticaux.
     * Bit 0 = pixel du haut (ligne 0), bit 7 = pixel du bas (ligne 7). */
    uint8_t line = font_6x8[idx][col];

    /* Parcours des 8 lignes de la colonne */
    for (uint8_t row = 0; row < 8; row++)
    {
      /* Extraction du pixel à cette position (0 ou 1) */
      uint8_t pixel = (line >> row) & 0x01;

      if (scale == 1)
      {
        /* Taille native : un pixel source = un pixel écran */
        SSD1306_DrawPixel(x + col, y + row, pixel);
      }
      else
      {
        /* Agrandissement : un pixel source = un carré scale×scale.
         * Pour scale=3 et pixel ON : on dessine un carré 3×3 de pixels blancs.
         * Pour pixel OFF : on dessine un carré 3×3 de pixels noirs. */
        for (uint8_t sy = 0; sy < scale; sy++)
          for (uint8_t sx = 0; sx < scale; sx++)
            SSD1306_DrawPixel(x + col * scale + sx,
                              y + row * scale + sy, pixel);
      }
    }
  }
}

/**
 * @brief  Dessine une chaîne de caractères.
 *
 * Avance de (6 × scale) pixels à droite après chaque caractère.
 * S'arrête quand le prochain caractère dépasserait le bord droit (128 pixels).
 *
 * @param x, y   Position de départ
 * @param str    Chaîne C terminée par '\0'
 * @param scale  Facteur d'agrandissement
 */
void SSD1306_DrawString(uint8_t x, uint8_t y, const char *str, uint8_t scale)
{
  /* Largeur d'un caractère agrandi (en pixels) */
  uint8_t char_w = 6 * scale;

  while (*str)
  {
    /* Vérifier qu'il y a assez de place pour le prochain caractère */
    if (x + char_w > SSD1306_WIDTH) break;

    SSD1306_DrawChar(x, y, *str, scale);
    x += char_w;    /* avancer la position horizontale */
    str++;          /* caractère suivant */
  }
}

/**
 * @brief  Dessine une ligne horizontale.
 *
 * Trace w pixels blancs (ou noirs) consécutifs à la position (x, y).
 * Le clipping est géré par DrawPixel (les pixels hors écran sont ignorés).
 *
 * @param x      Colonne de départ
 * @param y      Ligne
 * @param w      Nombre de pixels de large
 * @param color  1=blanc, 0=noir
 */
void SSD1306_DrawHLine(uint8_t x, uint8_t y, uint8_t w, uint8_t color)
{
  for (uint8_t i = 0; i < w && (x + i) < SSD1306_WIDTH; i++)
    SSD1306_DrawPixel(x + i, y, color);
}

/**
 * @brief  Dessine un rectangle plein (rempli).
 *
 * Double boucle simple : pour chaque ligne j de 0 à h-1,
 * on dessine w pixels horizontaux. Clipping par DrawPixel.
 *
 * @param x, y   Coin supérieur gauche
 * @param w, h   Largeur et hauteur
 * @param color  1=blanc, 0=noir
 */
void SSD1306_FillRect(uint8_t x, uint8_t y, uint8_t w, uint8_t h,
                      uint8_t color)
{
  for (uint8_t j = 0; j < h; j++)
    for (uint8_t i = 0; i < w; i++)
      SSD1306_DrawPixel(x + i, y + j, color);
}

/* ═══════════════════════════════════════════════════════════════════════════ */
/*  Affichage dédié au radar Doppler                                          */
/* ═══════════════════════════════════════════════════════════════════════════ */

/* Disposition de l'écran (128×64 pixels, 8 pages de 8 lignes) :
 *
 * Mode CIBLE DÉTECTÉE :
 *   ┌────────────────────────────────┐
 *   │  RADAR DOPPLER 24GHz   (y=0)  │  ← titre, police 1× (6×8)
 *   │────────────────────────(y=9)──│  ← ligne séparatrice horizontale
 *   │                               │
 *   │     1 2 3 . 4          (y=12) │  ← vitesse, police 3× (18×24), 5 caractères
 *   │                          km/h │  ← label "km/h" en petit (y=28)
 *   │────────────────────────(y=38)─│  ← séparateur
 *   │  34.3 m/s   5523 Hz   (y=41) │  ← vitesse m/s + fréquence Hz
 *   │  SNR: 23.1 dB         (y=51) │  ← SNR en dB
 *   └────────────────────────────────┘
 *
 * Mode PAS DE CIBLE :
 *   ┌────────────────────────────────┐
 *   │  RADAR DOPPLER 24GHz         │
 *   │────────────────────────────────│
 *   │      - - . -                  │  ← tirets en gros
 *   │                          km/h │
 *   │────────────────────────────────│
 *   │       NO TARGET               │  ← message centré
 *   │  SNR: 3.2 dB                 │
 *   └────────────────────────────────┘
 */

/**
 * @brief  Affiche les données radar sur l'écran OLED.
 *
 * Cette fonction :
 * 1. Efface le framebuffer
 * 2. Dessine le titre et les séparateurs
 * 3. Si cible détectée : affiche la vitesse en gros (18×24 px), les détails
 * 4. Si pas de cible : affiche "--.-" et "NO TARGET"
 * 5. Envoie le framebuffer à l'écran via I2C
 *
 * Appelée depuis la boucle principale toutes les 200 ms (5 Hz).
 *
 * @param speed_kmh  Vitesse mesurée en km/h
 * @param speed_ms   Vitesse mesurée en m/s
 * @param freq_hz    Fréquence Doppler détectée en Hz
 * @param snr_db     Rapport signal/bruit en dB
 * @param detected   1 si cible présente, 0 sinon
 */
void SSD1306_ShowRadar(float speed_kmh, float speed_ms,
                       float freq_hz, float snr_db, uint8_t detected)
{
  /* Buffer temporaire pour sprintf (21 caractères max + '\0' = 22 octets,
   * car l'écran fait 128/6 ≈ 21 caractères en police 1×) */
  char line[22];

  /* Effacer le framebuffer pour partir d'un écran propre */
  SSD1306_Clear();

  /* ── Ligne 0 : barre de titre ── */
  SSD1306_DrawString(4, 0, "RADAR DOPPLER 24GHz", 1);

  /* ── Ligne 9 : séparateur horizontal (128 pixels de large) ── */
  SSD1306_DrawHLine(0, 9, 128, 1);

  if (detected)
  {
    /* ── Lignes 12-35 : vitesse en gros (police 3× = 18×24 px par caractère) ──
     *
     * Formatage de la vitesse avec 1 décimale :
     * On sépare la partie entière et la partie décimale manuellement
     * (au lieu d'utiliser %f de sprintf qui est lent et volumineux sur
     * les microcontrôleurs — certaines implémentations de newlib-nano
     * ne supportent même pas %f).
     *
     * Alignement à droite selon la magnitude :
     *   >= 100 : "123.4"  (5 caractères × 18px = 90px)
     *   >= 10  : " 23.4"  (espace de padding)
     *   < 10   : "  3.4"  (deux espaces de padding) */
    int spd_int = (int)speed_kmh;
    int spd_dec = ((int)(speed_kmh * 10.0f)) % 10;
    if (spd_dec < 0) spd_dec = -spd_dec;  /* sécurité pour les vitesses négatives */

    if (speed_kmh >= 100.0f)
      sprintf(line, "%3d.%d", spd_int, spd_dec);
    else if (speed_kmh >= 10.0f)
      sprintf(line, " %2d.%d", spd_int, spd_dec);
    else
      sprintf(line, "  %d.%d", spd_int, spd_dec);

    /* Affichage en police 3× : chaque caractère fait 18×24 pixels.
     * Position x=4 pour centrer approximativement sur l'écran. */
    SSD1306_DrawString(4, 12, line, 3);

    /* Label "km/h" en petit à côté du gros nombre (y=28 pour aligner
     * verticalement avec le bas du chiffre gros) */
    SSD1306_DrawString(98, 28, "km/h", 1);
  }
  else
  {
    /* ── Pas de cible : afficher des tirets en gros ── */
    SSD1306_DrawString(4, 12, " --.-", 3);
    SSD1306_DrawString(98, 28, "km/h", 1);
  }

  /* ── Ligne 38 : deuxième séparateur ── */
  SSD1306_DrawHLine(0, 38, 128, 1);

  if (detected)
  {
    /* ── Ligne 41 : détails (vitesse m/s + fréquence Doppler) ── */
    sprintf(line, "%.1f m/s", speed_ms);
    SSD1306_DrawString(4, 41, line, 1);

    sprintf(line, "%.0f Hz", freq_hz);
    SSD1306_DrawString(74, 41, line, 1);

    /* ── Ligne 51 : SNR en dB ── */
    sprintf(line, "SNR: %.1f dB", snr_db);
    SSD1306_DrawString(4, 51, line, 1);
  }
  else
  {
    /* ── Ligne 41 : message "NO TARGET" centré ── */
    SSD1306_DrawString(28, 41, "NO TARGET", 1);

    /* ── Ligne 51 : SNR quand même (utile pour le debug) ── */
    sprintf(line, "SNR:%.1f dB", snr_db);
    SSD1306_DrawString(4, 51, line, 1);
  }

  /* Transfert du framebuffer vers l'écran OLED via I2C.
   * C'est l'opération la plus longue (~20-25 ms à 400 kHz). */
  SSD1306_Update();
}
