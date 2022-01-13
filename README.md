# SUZETTE : Système Unicycle Zélé Et Télécommandé Tourné vers l'Enseignement
Suzette est un gyropode à réaction.
Sa roue motorisée lui permet de se maintenir en équilibre d'avant en arrière, tandis que son volant d'intertie lui permet de rester à la verticale sur le plan latéral.

L'inclinaison est estimée à l'aide de gyroscopes et d'accéléromètres (MPU9250).

La commande est de type 'Integral Term Sliding Mode Controller', implémentée en python sur un ordinateur embarqué (RPI4).
Ce dernier est configuré en point d'accès Wi-Fi.
Le pilotage se fait au travers d'une page web accessible dans un navigateur une fois un smartphone ou autre appareil connecté sur le point d'accès.

Dans un futur proche une instrumentation sera également possible via l'interface web (réglage des gains de la commande, récupération de données en temps réel, etc.).

![Suzette](Suzette.png)
