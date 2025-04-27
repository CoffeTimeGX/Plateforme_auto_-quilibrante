# Projet de plateforme auto-équilibrante
***
# Objectif du projet
Mettre en oeuvre un système autonome capable de s'ajuster automatiquement en réponse à diverses perturbations externes.
# Conduite du projet
Le projet a été réalisé durant diverses séances mises en place par l'école nous ayant permis de travailler en continu sur le projet
# Exigences techniques
Le résultat final doit atteindre un certain standard qui prend en compte diverses aspects techniques et logiciels tels que  : 
- Un système temps réel capable de s'ajuster quasi immédiatement aux diverses perturbations, et donc pouvoir capter tout d'abord les données mesurées par ce dernier.
- Un logiciel menant avec précision toute l'électronique du projet afin que les entrées gérées par la carte puissent engendrer un mouvement précis.
- Du matériel capable de mener à bien la tâche qui lui sera déléguée, doté donc de robustesse, de précision et de longévité.


Ces différentes exigences seront ainsi assumées par le choix du matériel ainsi que la programmation de l'algorithme choisi.


# Flot de conception
Le flot de conception fut entièrement mathématique, logiciel et mécanique, en raison d'un manque de matériel. Le code fut la première étape de conception du projet et cela a été fait en amont de la réception des pièces (qui n'a pas eu lieu).

Ainsi, le rendu du projet ne sera pas complet pour cette même raison, et la majeure partie de la présentation sera logicielle et mécanique, sans la partie électronique donc. En raison de l'absence d'électronique, je me suis référé à divers tutoriels pour implémenter les divers composants que j'ai choisi pour ce projet, ainsi que les exemples des librairies.

Ainsi, ce flot de conception visera à présenter une liste d'étape, implémentable pour divers projets d'électronique et de mécanique, et qui permettra la réalisation d'un tel projet de manière reproductible.


## I. Conception Mathématique

### **Modélisation Cinématique Inverse (MCI) :**

**Objectif : Déterminer les angles de rotation des trois actionneurs rotatifs (3R) nécessaires pour atteindre une pose (position et orientation) désirée de la plateforme supérieure tactile.**

**Approche :**
Définir les systèmes de coordonnées pour la base fixe et la plateforme mobile.
Décrire la géométrie de la structure 3RPS (positions des points d'attache sur la base et la plateforme, longueurs des bielles).
Établir les équations de fermeture de boucle vectorielle reliant les angles des actionneurs à la pose de la plateforme.

Résoudre le système d'équations non-linéaires pour obtenir les angles des actionneurs (θ1, θ2, θ3) en fonction de la position (x,y,z) et de l'orientation (roulis ϕ, tangage θ, lacet ψ) de la plateforme.

### **Stratégie de Contrôle de l'Équilibrage :**

**Objectif : Définir l'algorithme qui permettra à la plateforme de maintenir son équilibre en réponse aux perturbations.**

**Approche :**
Pour détecter l'inclinaison, il faudra utiliser les données de la plaque tactile pour estimer l'angle d'inclinaison de la plateforme par rapport à l'horizontale. On peut considérer le centre de pression du contact comme un indicateur de l'inclinaison.

La définition de l'algorithme de stabilisation se basera sur la mise en oeuvre du contrôleur PID  qui prendra en entrée l'angle d'inclinaison et génèrera les commandes de couple nécessaires pour chaque actionneur afin de ramener la plateforme à l'équilibre. 

Le contrôleur devra être conçu en tenant compte de la dynamique du système. Il sera vu dans la partie logicielle ci-dessous.
Trajectoires de référence (optionnel) : Si l'on souhaite que la plateforme suive des mouvements spécifiques tout en maintenant son équilibre, il faudra définir des trajectoires de référence pour la position et l'orientation (Egalement vu dans la partie ci-dessous)



## II. Conception Logicielle (Teensy 4.1)

### Acquisition des Données de la Plaque Tactile :

**Interface :** Déterminer le protocole de communication entre la plaque tactile et le Teensy 4.1 (I2C, SPI, etc.).

**Traitement des données brutes :** Lire les données brutes de la plaque tactile (pressions en différents points) et les traiter pour obtenir des informations utiles, telles que la position du centre de pression et potentiellement l'angle d'inclinaison estimé.
### Implémentation du Contrôleur :

**Discrétisation du contrôleur :** Convertir l'algorithme de contrôle continu (défini dans la phase mathématique) en un algorithme discret adapté à l'exécution sur un microcontrôleur. Choisir une fréquence d'échantillonnage appropriée en fonction de la dynamique du système (On pourra se permettre une forte fréquence étant-donné la carte choisie)

**Calcul des commandes des stepper-moteurs :** Implémenter l'algorithme de contrôle qui calcule les angles de consigne nécessaires pour chaque moteur en fonction de l'état actuel de la plateforme (estimé à partir des données de la plaque tactile).

**Cinématique Inverse en temps réel :** Implémenter l'algorithme de conversion des angles de consigne des actionneurs en signaux de commande pour les moteurs

### Gestion des Actionneurs :

**Interface moteur :** Configurer les broches du Teensy 4.1 pour piloter les moteurs (servomoteurs, moteurs pas à pas avec drivers, etc.).

**Génération des signaux de commande :** Générer les signaux de commande appropriés (PWM, signaux de pas et direction) en fonction des angles de consigne calculés.

**Asservissement local (si applicable) :** Si les actionneurs sont des servomoteurs avec un asservissement de position intégré, envoyer directement les angles de consigne. Pour d'autres types de moteurs, une boucle d'asservissement de position locale pourrait être nécessaire sur le Teensy.

### Boucle de Contrôle Principale :

Organiser l'ensemble des tâches (acquisition des données, traitement, calcul du contrôle, commande des actionneurs) dans une boucle d'exécution en temps réel sur le Teensy 4.1. Assurer une synchronisation appropriée entre les différentes étapes. (Assez compliqué sans matériel, rendant impossible le test unitaire et d'intégration)


Au final, le travail sur toutes ces étapes s'est un peu réalisé en même temps, sans que je ne me concentre sur une tâche particulière à la fois.


## III. Conception Mécanique

### Architecture 3RPS :

**Configuration :** Choisir la configuration spécifique de la structure 3RPS (parallèle, série). La configuration parallèle est généralement privilégiée pour la rigidité et la capacité de charge.

**Géométrie :** Déterminer les dimensions clés de la structure :
Rayon et disposition des points d'attache sur la base fixe.
Rayon et disposition des points d'attache sur la plateforme mobile.
Longueur des bielles (actionneurs linéaires ou rotatifs avec des liaisons).
### Choix des Actionneurs :

**Type :** Sélectionner le type d'actionneurs rotatifs (servomoteurs, moteurs pas à pas avec réducteur). Les servomoteurs offrent généralement un bon compromis entre vitesse, précision et couple pour des applications d'équilibrage.

**Spécifications :** Déterminer les spécifications requises pour les actionneurs : couple maximal, vitesse maximale, résolution, précision. Ces spécifications dépendront des charges attendues et de la dynamique souhaitée.

### Conception des Liaisons :

T**ypes de liaisons :** Choisir les types de liaisons appropriés pour la structure 3RPS (rotules, pivots). Assurer un mouvement fluide et sans jeu.

**Matériaux :** Sélectionner des matériaux appropriés pour les liaisons en fonction des contraintes mécaniques et de l'usure.

### Conception de la Plateforme et de la Base :

**Matériaux :** Choisir des matériaux légers et rigides pour la plateforme et la base (aluminium, fibre de carbone, etc.) afin de minimiser l'inertie et maximiser la réactivité.

**Fixations :** Concevoir des systèmes de fixation robustes pour les actionneurs et les liaisons.
Intégration de la plaque tactile : Prévoir un emplacement et un système de fixation appropriés pour la plaque tactile sur la plateforme.


### Considérations d'Équilibrage Statique :

Concevoir la géométrie et la distribution des masses de la plateforme pour minimiser le déséquilibre statique initial. Cela facilitera la tâche du système de contrôle.


### Modélisation :
Enfin, concevoir le modèle sur logiciel de modélisation 3D. On peut se baser sur différents croquis réalisés. La liberté de conception permet un vaste panel de conception, à la condition que la structure finale soit mathématiquement correcte.

# Tests unitaires & tests d'intégration

Comme dit precédemment, l'absence de matériel rend impossible les tests, mis à part que le programme tourne bien sur carte. L'implémentation des équations cinématiques inverses étant probablement correctes, il ne sera pas possible en revanche de s'assurer du bon fonctionnement du PID mis-en-place, l'ajustement des coefficients étant capital pour le bon fonctionnement du système.

# Résultats et analyse critique des résultats

Pour ce qui est des résulats, il n'y a rien de particulier à tirer pour la même raison citée précédememnt.
 

 # Conclusion et perspectives

La conception d'une plateforme auto-équilibrante 3RPS représente un projet pluridisciplinaire stimulant, où les mathématiques, le logiciel, l'électronique et la mécanique s'interconnectent étroitement. La réussite d'un tel système repose sur une modélisation mathématique rigoureuse pour comprendre la cinématique et la dynamique, sur un développement logiciel efficace pour l'acquisition des données de la plaque tactile, l'implémentation de l'algorithme de contrôle et la commande des actionneurs, ainsi que sur une conception mécanique soignée garantissant a précision et la réactivité de la structure. L'intégration réussie de ces trois domaines (sans l'électronique) permettra de créer une plateforme capable de maintenir son équilibre de manière autonome face à des perturbations.

Originellement, la  plateforme aurait non seulement dû être construite, mais il aurait aussi fallut qu'elle permette le rebond de la balle tout en continuant de l'asservir, chose qu'il fut compliqué à implémenter en raison du manque de matériel. Cette étape aurait pû constituer un nouveau challenge d'implémentation logicielle. Le mise en place d'un Kalmann aurait pû être la réponse à une telle fonctionalité, permettant de mettre en place un système s'asservissant avec des mesures de position pour la balle limtés dans le temps.

En conclusion, concevoir cette plateforme auto-équilibrante 3RPS est un défi stimulant qui marie diverses professions pour créer un système capable de s'équilibrer de manière autonome, ce qui permet de faire le parallèle avec des sytèmes plus robustes et sophistiqués, qui nécessitant plusieurs corps de métiers permettant la réalisation d'un tel système.
