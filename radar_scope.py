#!/usr/bin/env python3
"""
radar_scope.py - Oscilloscope temps-réel + Analyseur spectral pour Radar Doppler
=================================================================================
Ce script lit les données ADC brutes et les résultats FFT envoyés par le
STM32F767ZI via UART en protocole binaire, puis affiche 4 panneaux en temps réel :
  1. Forme d'onde temporelle (oscilloscope) avec stabilisation par trigger
  2. Spectre FFT en magnitude (échelle logarithmique dB)
  3. Spectrogramme (cascade/waterfall) évoluant dans le temps
  4. Affichage en direct de la vitesse / fréquence / SNR

Protocole binaire envoyé par le STM32 :
  [0xAA][0x55]             - en-tête de synchronisation (2 octets magiques)
  [N_lo][N_hi]             - nombre d'échantillons (uint16 little-endian)
  [s0_lo][s0_hi]...[sN]   - échantillons ADC (uint16 little-endian chacun)
  [0xBB][0x66]             - en-tête des résultats FFT
  [freq_f32]               - fréquence Doppler détectée (float32 LE, 4 octets)
  [speed_kmh_f32]          - vitesse en km/h (float32 LE)
  [speed_ms_f32]           - vitesse en m/s (float32 LE)
  [snr_f32]                - rapport signal/bruit en dB (float32 LE)
  [detected_u8]            - 1 = cible détectée, 0 = pas de cible (1 octet)

Utilisation :
  python radar_scope.py              (utilise COM5 par défaut)
  python radar_scope.py COM3         (spécifier un autre port)
  python radar_scope.py /dev/ttyACM0 (sous Linux)

Dépendances :
  pip install pyserial numpy matplotlib
"""

# ═══════════════════════════════════════════════════════════════════════════════
# Imports
# ═══════════════════════════════════════════════════════════════════════════════
# sys          : accès aux arguments de la ligne de commande (choix du port)
# struct       : décodage des données binaires (little-endian floats, uint16)
# time         : mesure du temps pour le calcul du FPS
# threading    : exécution du lecteur série dans un thread séparé
# collections  : (importé pour usage futur, ex: deque pour buffers circulaires)
# numpy        : calcul numérique (FFT, tableaux, fenêtrage)
# serial       : communication série UART avec la carte Nucleo
# matplotlib   : affichage graphique temps-réel des 4 panneaux
import sys
import struct
import time
import threading
import collections
import numpy as np
import serial

# On force le backend TkAgg pour matplotlib car il supporte bien les
# animations temps-réel sur toutes les plateformes (Windows, Linux, Mac).
# Sans ce choix explicite, matplotlib pourrait choisir un backend non-interactif
# qui empêcherait l'affichage de la fenêtre graphique.
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import matplotlib.gridspec as gridspec

# ═══════════════════════════════════════════════════════════════════════════════
# Configuration
# ═══════════════════════════════════════════════════════════════════════════════

# --- Port série ---
# Le port est soit passé en argument (ex: python radar_scope.py COM3),
# soit COM5 par défaut (port USB de la Nucleo-144 sous Windows)
PORT = sys.argv[1] if len(sys.argv) > 1 else 'COM7'

# Débit UART : 921600 bauds, le plus haut débit standard supporté.
# À ce débit, on peut transmettre ~92160 octets/s (10 bits par octet avec
# start+stop), ce qui permet d'envoyer ~24 trames de 1024 échantillons/s.
BAUD = 921600

# Fréquence d'échantillonnage originale de l'ADC sur le STM32 (50 kHz).
# Le Timer 2 cadence l'ADC à exactement 50000 conversions par seconde.
FS = 50000.0

# Fréquence effective du flux UART : le STM32 décime les échantillons
# d'un facteur 2 (ne garde qu'un échantillon sur deux) avant de les envoyer,
# car la bande passante UART ne permet pas d'envoyer les 2048 échantillons
# à chaque trame FFT (ça dépasserait les 92 ko/s disponibles).
# Résultat : le PC reçoit un signal échantillonné à 25 kHz au lieu de 50 kHz.
# D'après le théorème de Nyquist, on peut quand même observer des fréquences
# jusqu'à 12.5 kHz, ce qui est largement suffisant pour le radar Doppler
# (les fréquences Doppler typiques sont < 5 kHz pour des vitesses < 100 km/h).
FS_STREAM = 25000.0

# Nombre d'échantillons par trame envoyée : 1024 (après décimation 2:1 de 2048)
# Chaque trame = 2 octets de sync + 2 octets de compteur + 1024*2 octets ADC
#              + 2 octets d'en-tête FFT + 17 octets de résultats = 2071 octets
N_STREAM = 1024

# Taille de la FFT calculée sur le STM32 (sur les 2048 points à 50 kHz complet).
# On garde cette constante pour référence, même si côté PC on recalcule
# une FFT sur 1024 points à 25 kHz.
FFT_SIZE = 2048

# Facteur de conversion Doppler : fd = 2*v*f0/c ≈ 161 Hz par m/s
# pour un radar à 24.125 GHz. Ce facteur n'est pas utilisé directement ici
# car le STM32 envoie déjà la vitesse calculée, mais on le garde en référence.
DOPPLER_SCALE = 161.0

# --- Paramètres du trigger (stabilisation oscilloscope) ---
# TRIGGER_LEVEL_FRAC : position du seuil de trigger en fraction de l'amplitude
# du signal. 0.5 = au milieu du swing (point médian entre min et max).
# Comme sur un oscilloscope réel, le trigger attend que le signal traverse
# ce niveau pour "déclencher" l'affichage, ce qui stabilise la forme d'onde.
TRIGGER_LEVEL_FRAC = 0.5

# Type de front du trigger : 'rising' (front montant) ou 'falling' (front descendant).
# Front montant = on attend que le signal passe de sous le seuil à au-dessus.
# C'est le mode par défaut sur la plupart des oscilloscopes.
TRIGGER_EDGE = 'rising'

# Nombre d'échantillons à afficher dans la vue oscilloscope.
# 512 échantillons à 25 kHz = 20.48 ms de signal visible.
# On prend la moitié de N_STREAM pour avoir de la marge pour le trigger :
# il faut chercher un point de déclenchement dans les 512 premiers échantillons,
# puis afficher les 512 suivants à partir de ce point.
SCOPE_WINDOW = 512

# --- Spectrogramme ---
# Nombre de trames FFT historiques gardées pour le spectrogramme (waterfall).
# 80 trames à ~24 fps ≈ 3.3 secondes de profondeur visible.
# Le spectrogramme est une image 2D (temps × fréquence) qui défile vers le
# haut : chaque nouvelle trame FFT est ajoutée en bas et les anciennes
# remontent progressivement.
SPECTROGRAM_HISTORY = 80

# ═══════════════════════════════════════════════════════════════════════════════
# Données partagées entre le thread de lecture série et le thread GUI
# ═══════════════════════════════════════════════════════════════════════════════
# Architecture multi-thread :
# - Thread 1 (serial_reader) : lit en continu les données UART, les parse,
#   calcule la FFT côté PC, et met à jour l'objet SharedData.
# - Thread 2 (main/GUI) : matplotlib appelle animate() ~20x/s via FuncAnimation,
#   qui lit les données depuis SharedData et met à jour les graphiques.
#
# Le verrou (threading.Lock) empêche les accès simultanés aux données :
# un thread doit acquérir le lock avant de lire ou écrire, garantissant
# la cohérence des données (pas de lecture d'une trame "à moitié écrite").
class SharedData:
    def __init__(self):
        # Verrou pour synchronisation inter-threads (mutex)
        self.lock = threading.Lock()

        # Échantillons ADC de la dernière trame (en float64 après soustraction DC)
        # Initialisés à zéro : N_STREAM = 1024 échantillons
        self.adc_samples = np.zeros(N_STREAM, dtype=np.float64)

        # Spectre FFT pré-calculé en dB (N_STREAM/2 = 512 bins de fréquence)
        # La FFT réelle de N points donne N/2+1 valeurs, mais on tronque à N/2
        # pour correspondre à l'axe fréquentiel du graphique.
        # Initialisé à -80 dB (plancher de bruit visuel)
        self.fft_db = np.full(N_STREAM // 2, -80.0)

        # Résultats envoyés par le STM32 (extraits de la trame binaire)
        self.freq = 0.0         # fréquence Doppler détectée (Hz)
        self.speed_kmh = 0.0    # vitesse cible (km/h)
        self.speed_ms = 0.0     # vitesse cible (m/s)
        self.snr = 0.0          # rapport signal/bruit (dB)
        self.detected = False   # True si une cible est détectée

        # Compteur de trames reçues (pour debug et monitoring)
        self.frame_count = 0

        # Matrice du spectrogramme : tableau 2D (temps × fréquence)
        # Chaque ligne = un spectre FFT en dB, les lignes défilent vers le haut
        # Dimensions : SPECTROGRAM_HISTORY lignes × N_STREAM/2 colonnes
        self.spectrogram = np.full((SPECTROGRAM_HISTORY, N_STREAM // 2), -80.0)

        # Taux de rafraîchissement mesuré (trames UART par seconde)
        self.fps = 0.0

        # État de la connexion série
        self.connected = False

# Instance globale unique des données partagées
shared = SharedData()

# ═══════════════════════════════════════════════════════════════════════════════
# Algorithme de trigger (stabilisation de l'affichage oscilloscope)
# ═══════════════════════════════════════════════════════════════════════════════
# Sur un oscilloscope réel, le "trigger" est un mécanisme fondamental qui
# synchronise le balayage horizontal avec le signal. Sans trigger, un signal
# périodique (comme notre écho Doppler) semblerait "défiler" à l'écran car
# chaque nouvelle acquisition commencerait à une phase aléatoire du signal.
#
# Le trigger fonctionne en 3 étapes :
# 1. On calcule un seuil (trigger level) basé sur l'amplitude du signal
# 2. On cherche le premier point où le signal traverse ce seuil
#    (front montant ou descendant selon le mode choisi)
# 3. On affiche la fenêtre de données à partir de ce point de traversée
#
# Résultat : le signal apparaît toujours à la même position horizontale,
# donnant une image stable même si le signal est continu.

def find_trigger_point(data, level, edge='rising', holdoff=50):
    """
    Recherche le premier point de déclenchement (trigger) dans les données.

    Paramètres :
    - data    : tableau numpy des échantillons ADC (après soustraction DC)
    - level   : seuil de déclenchement (valeur ADC à traverser)
    - edge    : 'rising' = front montant, 'falling' = front descendant
    - holdoff : nombre d'échantillons à ignorer au début du buffer.
                Le holdoff évite les faux déclenchements sur le bruit
                en début de trame (zone de transition entre deux trames).

    Retourne :
    - L'index du point de trigger, ou 0 si aucun trigger trouvé.
      Si aucun trigger n'est trouvé, on affiche depuis le début (mode "auto-run"
      comme sur un oscilloscope quand le signal est trop faible).
    """
    n = len(data)

    # On ne cherche le trigger que dans la première moitié des données,
    # car il faut garder SCOPE_WINDOW échantillons après le trigger pour
    # l'affichage. On laisse aussi une marge de 10 échantillons.
    # Exemple : N_STREAM=1024, SCOPE_WINDOW=512 → search_end=502
    search_end = n - SCOPE_WINDOW - 10
    if search_end < holdoff:
        # Pas assez de données pour chercher un trigger et afficher la fenêtre
        return 0

    if edge == 'rising':
        # Front montant : on cherche la première transition
        # où l'échantillon précédent est SOUS le seuil et l'actuel est AU-DESSUS.
        # C'est exactement comme le comparateur analogique d'un vrai oscilloscope
        # qui détecte le passage du signal au-dessus de la tension de trigger.
        for i in range(holdoff, search_end):
            if data[i - 1] < level and data[i] >= level:
                return i
    else:  # front descendant (falling)
        # Front descendant : transition de AU-DESSUS vers EN-DESSOUS du seuil.
        # Utile pour certains signaux où le front descendant est plus net.
        for i in range(holdoff, search_end):
            if data[i - 1] > level and data[i] <= level:
                return i

    # Aucun trigger trouvé : le signal ne traverse pas le seuil dans la zone
    # de recherche. On retourne 0 pour afficher depuis le début (mode "auto-run").
    return 0


def compute_trigger_level(data):
    """
    Calcul automatique du niveau de trigger (mode AUTO).

    Sur un vrai oscilloscope, le mode AUTO ajuste automatiquement le seuil
    de trigger au point médian de l'excursion du signal. Cela permet
    un déclenchement fiable sans réglage manuel.

    Algorithme :
    1. On mesure le min et le max du signal sur la trame entière
    2. Si l'amplitude crête-à-crête est très faible (< 20 LSB), le signal
       est considéré comme plat (bruit uniquement) et on trigger sur la moyenne
    3. Sinon, le seuil = min + amplitude × TRIGGER_LEVEL_FRAC (50% par défaut)

    Paramètres :
    - data : tableau numpy des échantillons ADC

    Retourne :
    - La valeur du seuil de trigger (en unités ADC après DC removal)
    """
    sig_min = np.min(data)
    sig_max = np.max(data)
    amplitude = sig_max - sig_min

    if amplitude < 20:
        # Signal quasi-plat : amplitude < 20 LSB ≈ bruit de fond du convertisseur
        # 12 bits. Dans ce cas, on place le trigger sur la valeur moyenne,
        # ce qui provoquera des déclenchements aléatoires (comme le mode AUTO
        # d'un oscilloscope quand il n'y a pas de signal).
        return np.mean(data)

    # Trigger au point médian : entre le creux et le pic du signal.
    # Avec TRIGGER_LEVEL_FRAC = 0.5, c'est exactement le milieu.
    return sig_min + amplitude * TRIGGER_LEVEL_FRAC


# ═══════════════════════════════════════════════════════════════════════════════
# Thread de lecture série (tourne en arrière-plan en continu)
# ═══════════════════════════════════════════════════════════════════════════════
# Ce thread est le cœur du pipeline de données. Il :
# 1. Ouvre la connexion série vers la Nucleo-144
# 2. Accumule les octets reçus dans un buffer
# 3. Cherche les en-têtes de synchronisation (0xAA 0x55)
# 4. Parse chaque trame complète (ADC + résultats FFT)
# 5. Recalcule la FFT côté PC (sur les données décimées)
# 6. Met à jour l'objet SharedData protégé par un lock
#
# L'architecture "thread producteur + GUI consommateur" est classique :
# le thread série ne bloque jamais le GUI, et le GUI ne ralentit jamais
# la lecture série. Les deux tournent de façon indépendante.

def serial_reader():
    """
    Thread d'arrière-plan : lecture continue des trames binaires UART,
    décodage du protocole, calcul FFT, mise à jour des données partagées.
    """
    # Compteur pour mesurer le débit réel en trames/seconde (FPS)
    fps_counter = 0
    fps_timer = time.time()

    # Boucle infinie avec reconnexion automatique :
    # si le câble USB est débranché ou la carte redémarrée,
    # le thread attend 2 secondes et retente la connexion.
    while True:
        try:
            # Ouverture du port série avec les paramètres configurés.
            # timeout=1 : les lectures bloquantes expirent après 1 seconde,
            # ce qui permet de détecter la déconnexion.
            ser = serial.Serial(PORT, BAUD, timeout=1)

            # Vider le buffer de réception : on jette les données résiduelles
            # qui pourraient être des trames incomplètes d'une session précédente.
            ser.reset_input_buffer()
            print(f"[*] Connected to {PORT} @ {BAUD} baud")
            shared.connected = True

            # Buffer d'accumulation : on y concatène les octets reçus au fur
            # et à mesure. Les trames sont extraites du buffer dès qu'elles
            # sont complètes (parse "on-the-fly").
            buf = bytearray()

            while True:
                # Lire les données disponibles dans le buffer du driver série.
                # ser.in_waiting = nombre d'octets en attente dans le buffer OS.
                # On lit au maximum 8192 octets à la fois pour éviter de
                # consommer trop de mémoire, et au minimum 1 octet (bloquant
                # avec timeout si rien n'est disponible).
                chunk = ser.read(min(ser.in_waiting or 1, 8192))
                if not chunk:
                    # timeout sans données → on recommence la boucle
                    continue
                buf.extend(chunk)

                # === Boucle de parsing des trames ===
                # On peut avoir plusieurs trames complètes dans le buffer,
                # donc on boucle tant qu'on trouve des trames valides.
                while True:
                    # Recherche de l'en-tête de synchronisation 0xAA 0x55.
                    # Ces deux octets "magiques" marquent le début d'une trame.
                    # La méthode .find() cherche la sous-séquence dans le buffer.
                    idx = buf.find(b'\xAA\x55')
                    if idx == -1:
                        # Pas de sync trouvé : on vide le buffer sauf le dernier
                        # octet (qui pourrait être 0xAA, début d'un sync incomplet).
                        if len(buf) > 1:
                            buf = buf[-1:]
                        break

                    # Si le sync n'est pas au début du buffer, il y a des
                    # octets parasites avant → on les jette (désynchronisation
                    # après un démarrage en cours de trame, ou corruption).
                    if idx > 0:
                        buf = buf[idx:]

                    # Vérifier qu'on a assez d'octets pour lire le compteur
                    # d'échantillons : 2 (sync) + 2 (compteur) = 4 octets minimum
                    if len(buf) < 4:
                        break  # attendre plus de données

                    # Décodage du nombre d'échantillons ADC dans cette trame.
                    # '<H' = uint16 little-endian, lu à l'offset 2 dans le buffer.
                    # Normalement = 1024 (N_STREAM), mais on le lit dynamiquement
                    # pour être robuste à un changement de configuration.
                    n_samples = struct.unpack_from('<H', buf, 2)[0]

                    # Contrôle de cohérence : un nombre d'échantillons nul
                    # ou supérieur à 8192 indique une trame corrompue.
                    # On saute les 2 octets de sync et on recommence la recherche.
                    if n_samples == 0 or n_samples > 8192:
                        buf = buf[2:]
                        continue

                    # Calcul de la taille totale de la trame :
                    #   2 octets sync (0xAA 0x55)
                    # + 2 octets compteur (n_samples en uint16)
                    # + n_samples × 2 octets (échantillons ADC en uint16)
                    # + 2 octets en-tête FFT (0xBB 0x66)
                    # + 4 × 4 octets = 16 octets (4 floats : freq, speed_kmh, speed_ms, snr)
                    # + 1 octet (detected flag)
                    # = 4 + n_samples*2 + 2 + 17 = n_samples*2 + 23
                    frame_size = 4 + n_samples * 2 + 2 + 17
                    if len(buf) < frame_size:
                        break  # trame incomplète, attendre plus de données

                    # === Décodage des échantillons ADC ===
                    # Les échantillons commencent à l'offset 4 (après sync + compteur).
                    # On les lit comme un tableau numpy de uint16 little-endian.
                    # np.frombuffer est très efficace : il crée une "vue" sur le
                    # buffer sans copier les données.
                    adc_offset = 4
                    samples = np.frombuffer(buf, dtype='<u2',
                                             count=n_samples,
                                             offset=adc_offset)

                    # === Vérification de l'en-tête FFT ===
                    # Juste après les échantillons ADC, on doit trouver 0xBB 0x66.
                    # Si ce n'est pas le cas, la trame est corrompue (décalage
                    # d'octets). On saute le sync et on recommence.
                    fft_hdr_offset = adc_offset + n_samples * 2
                    if buf[fft_hdr_offset] != 0xBB or buf[fft_hdr_offset + 1] != 0x66:
                        buf = buf[2:]
                        continue

                    # === Décodage des résultats FFT du STM32 ===
                    # 4 valeurs float32 little-endian consécutives, puis 1 octet.
                    # Ces résultats viennent de la FFT 2048 points calculée sur
                    # le STM32 à la fréquence d'échantillonnage complète (50 kHz).
                    res_offset = fft_hdr_offset + 2
                    freq_val = struct.unpack_from('<f', buf, res_offset)[0]      # Hz
                    spd_kmh  = struct.unpack_from('<f', buf, res_offset + 4)[0]  # km/h
                    spd_ms   = struct.unpack_from('<f', buf, res_offset + 8)[0]  # m/s
                    snr_val  = struct.unpack_from('<f', buf, res_offset + 12)[0] # dB
                    det_val  = buf[res_offset + 16]  # 0 ou 1

                    # === Calcul de la FFT côté PC (pour l'affichage spectral) ===
                    # On recalcule une FFT sur les échantillons décimés reçus,
                    # car le STM32 n'envoie pas le spectre complet (trop de données).
                    # Cette FFT est différente de celle du STM32 :
                    # - 1024 points au lieu de 2048
                    # - fréquence d'échantillonnage 25 kHz au lieu de 50 kHz
                    # - résolution fréquentielle = 25000/1024 ≈ 24.4 Hz/bin
                    #   (vs 50000/2048 ≈ 24.4 Hz/bin sur le STM32 — même résolution !)

                    # Conversion en float64 pour la précision des calculs
                    samples_f = samples.astype(np.float64)

                    # Suppression de la composante continue (DC removal) :
                    # On soustrait la moyenne du signal. Sans cela, la composante
                    # DC (offset de l'ADC) créerait un pic énorme à 0 Hz dans
                    # le spectre FFT qui masquerait les fréquences Doppler utiles.
                    samples_f -= np.mean(samples_f)

                    # Application de la fenêtre de Hann (fenêtrage spectral) :
                    # Sans fenêtrage, la FFT d'un signal qui ne contient pas un
                    # nombre entier de périodes produit des "fuites spectrales"
                    # (énergie qui se répand dans les bins voisins). La fenêtre
                    # de Hann atténue progressivement les bords du signal à zéro,
                    # réduisant drastiquement ces fuites et donnant des pics plus
                    # nets dans le spectre. C'est la fenêtre la plus utilisée
                    # en analyse spectrale car elle offre un bon compromis entre
                    # résolution fréquentielle et atténuation des lobes secondaires.
                    window = np.hanning(len(samples_f))
                    windowed = samples_f * window

                    # FFT réelle (rfft) : comme le signal est réel, la FFT est
                    # symétrique et on ne garde que la moitié positive (N/2+1 bins).
                    # rfft est 2× plus rapide que fft pour les signaux réels.
                    fft_result = np.fft.rfft(windowed)
                    fft_mag = np.abs(fft_result)

                    # Conversion en décibels : dB = 20 × log10(magnitude)
                    # On ajoute 1e-12 pour éviter log10(0) = -inf.
                    # L'échelle dB compresse la dynamique : un signal 1000×
                    # plus fort ne fait que +60 dB, ce qui permet de voir
                    # simultanément les pics forts et le plancher de bruit.
                    fft_db = 20.0 * np.log10(fft_mag + 1e-12)

                    # === Mise à jour des données partagées (section critique) ===
                    # Le bloc "with shared.lock" acquiert le mutex :
                    # - Si le GUI est en train de lire → on attend qu'il finisse
                    # - Sinon on acquiert le lock, on écrit, on relâche
                    # Cela garantit que le GUI ne lira jamais un état incohérent
                    # (ex: les échantillons d'une trame N avec le spectre de N-1).
                    with shared.lock:
                        # Copie des échantillons (après DC removal) pour l'oscilloscope
                        shared.adc_samples = samples_f.copy()

                        # Copie du spectre FFT (tronqué à N_STREAM/2 = 512 bins)
                        # Le bin rfft[0] = DC, rfft[512] = Nyquist.
                        # On garde [0..511] pour correspondre à l'axe du graphique.
                        shared.fft_db = fft_db[:N_STREAM // 2].copy()

                        # Résultats du STM32 (déjà extraits de la trame)
                        shared.freq = freq_val
                        shared.speed_kmh = spd_kmh
                        shared.speed_ms = spd_ms
                        shared.snr = snr_val
                        shared.detected = (det_val == 1)
                        shared.frame_count += 1

                        # Mise à jour du spectrogramme : décalage vers le haut
                        # np.roll(..., -1, axis=0) décale toutes les lignes d'un
                        # cran vers le haut (la ligne 0 = la plus ancienne est perdue,
                        # la dernière ligne est remplacée par le nouveau spectre).
                        # Cela crée l'effet "cascade" (waterfall) du spectrogramme.
                        shared.spectrogram = np.roll(shared.spectrogram, -1, axis=0)
                        shared.spectrogram[-1, :] = fft_db[:N_STREAM // 2]

                    # === Calcul du FPS ===
                    # On compte les trames reçues et on divise par le temps écoulé
                    # chaque seconde pour obtenir le débit réel en trames/s.
                    fps_counter += 1
                    now = time.time()
                    if now - fps_timer >= 1.0:
                        shared.fps = fps_counter / (now - fps_timer)
                        fps_counter = 0
                        fps_timer = now

                    # Retirer la trame traitée du buffer et passer à la suivante
                    buf = buf[frame_size:]

        except serial.SerialException as e:
            # Erreur série (port fermé, câble débranché, etc.)
            # On repasse en état "déconnecté" et on retente après 2 secondes.
            shared.connected = False
            print(f"[!] Serial error: {e}. Retrying in 2s...")
            time.sleep(2)
        except Exception as e:
            # Erreur inattendue (parse, numpy, etc.) — on log et on continue
            print(f"[!] Reader error: {e}")
            time.sleep(1)


# ═══════════════════════════════════════════════════════════════════════════════
# Interface graphique : affichage temps-réel avec matplotlib
# ═══════════════════════════════════════════════════════════════════════════════
# On utilise matplotlib avec FuncAnimation pour le rafraîchissement périodique.
# L'interface est composée de 4 panneaux organisés en grille 3×2 :
#   Ligne 0, colonnes 0-1 : Oscilloscope (pleine largeur)
#   Ligne 1, colonne 0    : Spectre FFT
#   Ligne 1, colonne 1    : Spectrogramme (waterfall)
#   Ligne 2, colonnes 0-1 : Affichage numérique (vitesse, fréquence, SNR)

def setup_gui():
    """
    Crée et configure la disposition des 4 panneaux graphiques.

    Retourne un tuple contenant tous les objets graphiques nécessaires
    à la fonction animate() pour mettre à jour l'affichage.
    """

    # Thème sombre pour l'esthétique (fond noir comme un vrai instrument)
    plt.style.use('dark_background')

    # Création de la fenêtre principale (14×9 pouces)
    fig = plt.figure(figsize=(14, 9))
    fig.canvas.manager.set_window_title('Radar Doppler 24 GHz - Live Scope')

    # GridSpec : disposition flexible des panneaux avec ratios de hauteur.
    # height_ratios=[3, 3, 2] : les 2 premières lignes font 3 parts chacune,
    # la dernière (info) fait 2 parts. hspace/wspace = espacement inter-panneaux.
    gs = gridspec.GridSpec(3, 2, height_ratios=[3, 3, 2],
                           hspace=0.35, wspace=0.25,
                           left=0.07, right=0.97, top=0.93, bottom=0.06)

    # ── Panneau 1 : Oscilloscope (forme d'onde temporelle) ──
    # Occupe toute la largeur (gs[0, :]) pour avoir une vue large du signal.
    # L'axe X est en millisecondes, calculé d'après la fréquence d'échantillonnage
    # du flux (FS_STREAM = 25 kHz) et le nombre de points affichés (SCOPE_WINDOW).
    ax_scope = fig.add_subplot(gs[0, :])
    ax_scope.set_title('Oscilloscope (ADC)', fontsize=11, fontweight='bold',
                        color='#00FF88')
    ax_scope.set_xlabel('Time (ms)')
    ax_scope.set_ylabel('ADC Value')
    ax_scope.set_ylim(-2100, 2100)
    # Axe temporel : 512 points / 25000 Hz × 1000 = 0 à 20.48 ms
    t_axis = np.arange(SCOPE_WINDOW) / FS_STREAM * 1000.0  # conversion en ms
    ax_scope.set_xlim(t_axis[0], t_axis[-1])
    # Tracé initial : signal plat (zéros), sera mis à jour par animate()
    line_scope, = ax_scope.plot(t_axis, np.zeros(SCOPE_WINDOW),
                                 color='#00FF88', linewidth=0.8)
    # Ligne horizontale pointillée orange : indique le niveau de trigger
    line_trigger = ax_scope.axhline(y=0, color='#FF6600', linewidth=0.5,
                                     linestyle='--', alpha=0.6)
    ax_scope.grid(True, alpha=0.2, color='#444444')

    # ── Panneau 2 : Spectre FFT (domaine fréquentiel) ──
    # Affiche la magnitude FFT en dB en fonction de la fréquence.
    # L'axe X va de 0 à 8 kHz (suffisant pour observer les fréquences Doppler
    # correspondant à des vitesses jusqu'à ~180 km/h).
    ax_fft = fig.add_subplot(gs[1, 0])
    ax_fft.set_title('FFT Spectrum', fontsize=11, fontweight='bold',
                      color='#44AAFF')
    ax_fft.set_xlabel('Frequency (Hz)')
    ax_fft.set_ylabel('Magnitude (dB)')
    # Nombre de bins FFT = N_STREAM / 2 = 512
    # Chaque bin couvre FS_STREAM / N_STREAM ≈ 24.4 Hz
    n_fft_bins = N_STREAM // 2
    # Axe fréquentiel : 512 points de 0 Hz à FS_STREAM/2 = 12500 Hz
    freq_axis = np.linspace(0, FS_STREAM / 2, n_fft_bins)
    ax_fft.set_xlim(0, 8000)  # limité à 8 kHz pour le zoom utile
    ax_fft.set_ylim(-20, 80)  # plage dynamique typique du radar
    line_fft, = ax_fft.plot(freq_axis, np.full(n_fft_bins, -80),
                             color='#44AAFF', linewidth=0.8)
    # Ligne verticale rouge pointillée : marqueur du pic de fréquence détecté
    line_peak = ax_fft.axvline(x=0, color='#FF4444', linewidth=1.0,
                                linestyle='--', alpha=0.8)
    # Texte affichant la fréquence et le SNR du pic
    text_peak = ax_fft.text(0.02, 0.95, '', transform=ax_fft.transAxes,
                             fontsize=9, color='#FF4444', va='top',
                             fontfamily='monospace')
    ax_fft.grid(True, alpha=0.2, color='#444444')

    # ── Panneau 3 : Spectrogramme (cascade/waterfall) ──
    # Image 2D couleur : l'axe X = fréquence, l'axe Y = temps (trames),
    # la couleur = magnitude en dB (colormap 'inferno' : noir → rouge → jaune).
    # Les nouvelles données apparaissent en bas et remontent progressivement.
    # C'est le même type d'affichage utilisé en astronomie radio, sonar,
    # et analyse audio.
    ax_spectro = fig.add_subplot(gs[1, 1])
    ax_spectro.set_title('Spectrogram', fontsize=11, fontweight='bold',
                          color='#FFAA00')
    ax_spectro.set_xlabel('Frequency (Hz)')
    ax_spectro.set_ylabel('Time (frames)')
    # extent définit les coordonnées réelles de l'image (pas les pixels)
    extent = [0, FS_STREAM / 2, 0, SPECTROGRAM_HISTORY]
    img_spectro = ax_spectro.imshow(
        np.full((SPECTROGRAM_HISTORY, N_STREAM // 2), -80),
        aspect='auto',       # étirer l'image pour remplir le panneau
        origin='lower',      # l'origine (0,0) est en bas à gauche
        cmap='inferno',      # palette de couleurs : noir → rouge → jaune
        extent=extent,       # échelle des axes en Hz et en trames
        vmin=-20, vmax=70    # plage de dB pour la colormap
    )
    ax_spectro.set_xlim(0, 8000)  # zoom sur 0-8 kHz comme le spectre FFT

    # ── Panneau 4 : Affichage numérique (info readout) ──
    # Zone sans axes dédiée à l'affichage de la vitesse en gros,
    # des détails (m/s, fréquence, SNR) en plus petit, et du statut.
    ax_info = fig.add_subplot(gs[2, :])
    ax_info.set_xlim(0, 10)
    ax_info.set_ylim(0, 3)
    ax_info.axis('off')  # pas d'axes visibles pour ce panneau

    # Vitesse en grand (police monospace taille 32)
    text_speed = ax_info.text(0.5, 2.2, '-- km/h', fontsize=32,
                               fontweight='bold', color='#00FF88',
                               fontfamily='monospace', ha='left')
    # Détails : m/s, fréquence Doppler, SNR
    text_info = ax_info.text(0.5, 0.8, '', fontsize=12,
                              color='#AAAAAA', fontfamily='monospace',
                              ha='left')
    # Statut de détection (en haut à droite)
    text_status = ax_info.text(9.5, 2.2, 'CONNECTING...', fontsize=14,
                                fontweight='bold', color='#FF6600',
                                fontfamily='monospace', ha='right')
    # Compteur FPS et nom du port (en bas à droite)
    text_fps = ax_info.text(9.5, 0.8, '', fontsize=10,
                             color='#666666', fontfamily='monospace',
                             ha='right')

    # ── Titre général de la fenêtre ──
    fig.suptitle('RADAR DOPPLER 24.125 GHz - Live Analysis',
                  fontsize=13, fontweight='bold', color='#CCCCCC')

    # On retourne tous les objets graphiques dans un tuple pour que
    # animate() puisse les mettre à jour sans recréer les objets.
    return (fig, ax_scope, ax_fft, ax_spectro, ax_info,
            line_scope, line_trigger, line_fft, line_peak, text_peak,
            img_spectro, text_speed, text_info, text_status, text_fps)


def animate(frame, components):
    """
    Callback d'animation, appelé ~20 fois par seconde par FuncAnimation.

    Cette fonction est le lien entre les données partagées (mises à jour
    par le thread série) et l'affichage graphique (matplotlib).

    Paramètres :
    - frame      : numéro de trame (auto-incrémenté par FuncAnimation, non utilisé)
    - components : tuple contenant tous les objets graphiques retournés par setup_gui()

    Fonctionnement :
    1. Copie atomique de toutes les données partagées (sous lock)
    2. Application de l'algorithme de trigger sur les échantillons ADC
    3. Mise à jour de chaque panneau graphique
    """
    # Décompactage du tuple de composants graphiques
    (fig, ax_scope, ax_fft, ax_spectro, ax_info,
     line_scope, line_trigger, line_fft, line_peak, text_peak,
     img_spectro, text_speed, text_info, text_status, text_fps) = components

    # === Copie atomique des données partagées ===
    # On acquiert le lock pour copier TOUTES les données d'un seul coup.
    # Les .copy() créent des copies indépendantes des tableaux numpy,
    # ce qui nous permet de relâcher le lock rapidement sans bloquer
    # le thread série pendant le rendu graphique (qui est plus lent).
    with shared.lock:
        samples = shared.adc_samples.copy()     # 1024 échantillons ADC (float64)
        fft_db = shared.fft_db.copy()           # 512 bins FFT (dB)
        freq_val = shared.freq                  # fréquence Doppler (Hz)
        spd_kmh = shared.speed_kmh              # vitesse (km/h)
        spd_ms = shared.speed_ms                # vitesse (m/s)
        snr_val = shared.snr                    # SNR (dB)
        detected = shared.detected              # booléen cible détectée
        spectro_data = shared.spectrogram.copy() # matrice spectrogramme
        fps = shared.fps                        # FPS mesuré
        connected = shared.connected            # état connexion série

    # Si pas connecté, afficher l'état et sortir sans mettre à jour les graphiques
    if not connected:
        text_status.set_text('DISCONNECTED')
        text_status.set_color('#FF4444')
        return

    # ── Oscilloscope : application du trigger pour stabiliser l'affichage ──
    # 1. On calcule le niveau de trigger automatique (mode AUTO)
    trigger_level = compute_trigger_level(samples)
    # 2. On cherche le premier front montant traversant ce niveau
    trig_idx = find_trigger_point(samples, trigger_level, TRIGGER_EDGE)

    # 3. On extrait SCOPE_WINDOW échantillons à partir du point de trigger.
    #    C'est comme si l'oscilloscope "démarrait le balayage" à cet instant.
    end_idx = trig_idx + SCOPE_WINDOW
    if end_idx > len(samples):
        # Protection : si le trigger est trop tard dans le buffer,
        # on recule pour avoir quand même SCOPE_WINDOW points à afficher.
        end_idx = len(samples)
        trig_idx = max(0, end_idx - SCOPE_WINDOW)

    display_data = samples[trig_idx:trig_idx + SCOPE_WINDOW]
    # Si on n'a pas assez de points (ne devrait pas arriver), on complète
    # avec des zéros (zero-padding).
    if len(display_data) < SCOPE_WINDOW:
        display_data = np.pad(display_data,
                               (0, SCOPE_WINDOW - len(display_data)))

    # Mise à jour du tracé de l'oscilloscope
    line_scope.set_ydata(display_data)
    # Mise à jour de la ligne de trigger (repère visuel)
    line_trigger.set_ydata([trigger_level, trigger_level])

    # Auto-scale vertical : ajuster l'échelle Y pour que le signal
    # occupe ~80% de la hauteur (facteur 1.2), avec un minimum de ±100
    # pour éviter un zoom excessif sur le bruit.
    sig_range = max(np.max(np.abs(display_data)) * 1.2, 100)
    ax_scope.set_ylim(-sig_range, sig_range)

    # ── Spectre FFT (pré-calculé par le thread série) ──
    # On utilise directement le spectre fft_db déjà calculé dans serial_reader()
    # plutôt que de recalculer ici, pour ne pas gaspiller du CPU dans le thread GUI.
    # fft_db contient 512 bins correspondant à [0 Hz .. 12500 Hz].
    line_fft.set_ydata(fft_db)

    # Marqueur de pic : ligne verticale rouge à la fréquence du pic détecté
    if detected:
        line_peak.set_xdata([freq_val, freq_val])
        line_peak.set_alpha(0.8)
        # Annotation textuelle : fréquence et SNR du pic
        text_peak.set_text(f'{freq_val:.1f} Hz ({snr_val:.1f} dB)')
    else:
        line_peak.set_alpha(0.2)  # estomper le marqueur quand pas de cible
        text_peak.set_text('No target')

    # ── Spectrogramme (waterfall) ──
    # On remplace simplement les données de l'image. La matrice spectro_data
    # a déjà été décalée (roll) dans le thread série, donc le spectrogramme
    # "défile" naturellement vers le haut.
    img_spectro.set_data(spectro_data)

    # ── Panneau d'information numérique ──
    if detected:
        # Cible détectée : affichage vert avec vitesse, fréquence et SNR
        text_speed.set_text(f'{spd_kmh:6.1f} km/h')
        text_speed.set_color('#00FF88')
        text_info.set_text(
            f'{spd_ms:.1f} m/s  |  f = {freq_val:.1f} Hz  |  '
            f'SNR = {snr_val:.1f} dB')
        text_status.set_text('TARGET')
        text_status.set_color('#00FF88')
    else:
        # Pas de cible : affichage grisé avec SNR uniquement
        text_speed.set_text('  --.- km/h')
        text_speed.set_color('#666666')
        text_info.set_text(f'SNR = {snr_val:.1f} dB  |  Waiting for target...')
        text_status.set_text('NO TARGET')
        text_status.set_color('#FF6600')

    # Affichage du FPS et du port série (coin inférieur droit)
    text_fps.set_text(f'{fps:.1f} fps  |  {PORT}')


# ═══════════════════════════════════════════════════════════════════════════════
# Point d'entrée principal
# ═══════════════════════════════════════════════════════════════════════════════
# Architecture du programme :
# 1. Le thread principal affiche le banner et démarre le thread série
# 2. Le thread série tourne en boucle infinie en arrière-plan (daemon=True
#    signifie qu'il sera tué automatiquement quand le programme principal sort)
# 3. setup_gui() crée la fenêtre matplotlib avec les 4 panneaux
# 4. FuncAnimation appelle animate() toutes les 50 ms (20 fps),
#    ce qui lit les données partagées et met à jour les graphiques
# 5. plt.show() bloque le thread principal dans la boucle d'événements
#    matplotlib (la fenêtre reste ouverte jusqu'à fermeture manuelle)
# 6. À la fermeture, le thread daemon est terminé automatiquement

if __name__ == '__main__':
    # Bannière de démarrage dans la console
    print("=" * 50)
    print("  RADAR DOPPLER 24 GHz - Live Scope")
    print(f"  Port: {PORT}  Baud: {BAUD}")
    print("=" * 50)

    # Lancement du thread de lecture série en tant que "daemon" :
    # un thread daemon est automatiquement arrêté quand le programme
    # principal se termine (fermeture de la fenêtre matplotlib).
    # Sans daemon=True, le thread continuerait à tourner indéfiniment.
    reader_thread = threading.Thread(target=serial_reader, daemon=True)
    reader_thread.start()

    # Création de l'interface graphique (retourne tous les objets à animer)
    components = setup_gui()
    fig = components[0]

    # FuncAnimation : mécanisme de matplotlib pour les animations temps-réel.
    # - fig           : la figure à animer
    # - animate       : callback appelé à chaque rafraîchissement
    # - fargs         : arguments supplémentaires passés à animate()
    # - interval=50   : 50 ms entre chaque appel = 20 images/seconde cible
    # - blit=False    : on redessine tout le canvas (pas d'optimisation partielle,
    #                   plus simple et plus robuste pour notre usage)
    # - cache_frame_data=False : pas de mise en cache (données changent à chaque trame)
    ani = FuncAnimation(fig, animate, fargs=(components,),
                         interval=50, blit=False, cache_frame_data=False)

    # Bloque ici : la fenêtre matplotlib reste ouverte jusqu'à ce que
    # l'utilisateur la ferme manuellement (croix rouge ou Ctrl+C).
    plt.show()
