# Banc de Test Avionique - STM32

Projet de système embarqué réalisé dans le cadre du Master 1 CMI IMSAT. Ce banc de test acquiert, affiche et sauvegarde des données de vol en temps réel, tout en pilotant des indicateurs physiques.

## 🚀 Fonctionnalités

* **Acquisition capteurs (5 Hz) :** Lecture de la pression et température (BMP280), ainsi que des angles de roulis, tangage et lacet (ICM-20948) via bus I2C.
* **Affichage temps réel :** Retransmission des données sur écran OLED et terminal série (UART à 115200 baud).
* **Data Logging :** Enregistrement des mesures sur carte microSD au format `.csv` via bus SPI et la bibliothèque FatFs.
* **Retour physique :**
  * Pilotage d'un servomoteur (PWM) pour refléter physiquement l'angle de lacet.
  * LEDs d'alerte (GPIO) signalant le dépassement des limites d'angles de roulis et de tangage ($\pm 40^\circ$).
  * Bouton-poussoir mettant en pause ou relançant l'acquisition.
  * LED d'état signalant l'état de l'acquisition (acquisition en cours, en pause, ou terminée).

## 🛠️ Matériel utilisé

* **Microcontrôleur :** NUCLEO-STM32L432KC
* **Capteurs :** BMP280 (Température et pression), ICM-20948 (Centrale inertielle 9-axes)
* **Périphériques :** Écran OLED (SSD1306), Lecteur MicroSD (Digilent Pmod)
* **Actionneurs :** Servomoteur standard (SG90), LEDs et bouton-poussoir

## ⚙️ Architecture logicielle

* Développé sous **STM32CubeIDE** en langage C.
* Utilisation conjointe des bibliothèques matérielles **HAL** et d'accès directs aux registres (Bare-metal) pour les GPIOs.
* Implémentation du middleware **FatFs** pour la gestion du système de fichiers FAT.

Dans le répertoire, vous pouvez lire le rapport technique _CHASSING_Tom_ProjetS8_Banc_Avionique.pdf_.
