# LoraWemosGPS

Contient du code pour les cartes TTGO T-Beam ou TTGO LoRa32 v1 lié à :
* https://wiki.fablab-lannion.org/index.php?title=WemosTTGO_GPS
* https://wiki.fablab-lannion.org/index.php?title=WemosTTGO#Ajout_GPS


# Installation

bibliothèque nécessaires :
* [TinyGPS++](https://github.com/mikalhart/TinyGPSPlus) à installer en ligne de commande
* [MCCI LoRaWAN LMIC library](https://github.com/mcci-catena/arduino-lmic)  à installer via le gestionaire de bibliothèques

Pour le TTGO LoRa32 v1 :
* [U8g2](https://github.com/olikraus/u8g2) à installer via le gestionaire de bibliothèques

# Configuration

* copier le fichier `keys.h.template` en `keys.h` et remplacer les valeurs
* éditer le fichier `hw.h` pour définir les bonnes constantes en fonction de la carte utilisée
