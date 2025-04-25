#include "InverseKinematics.h"

/**
 * @brief Instance globale du système cinématique pour le calcul des angles des actionneurs.
 * Configurée avec les paramètres géométriques :
 * - Portée de la base : 2.5 unités (distance du centre de la base aux coins).
 * - Portée de la plateforme : 2.5 unités (distance du centre de la plateforme aux coins).
 * - Longueur du lien primaire : 1.5 unités.
 * - Longueur du lien secondaire : 3.0 unités.
 */
CelestialRig kinematicCore(2.5, 2.5, 1.5, 3.0);

/**
 * @brief Initialise le système et calcule les angles initiaux des actionneurs.
 * Configure la communication série et affiche les angles pour tous les actionneurs en fonction
 * d'entrées positionnelles prédéfinies (zPos=3.5, xVec=0.2, yVec=0.2).
 */
void setup() {
  Serial.begin(115200);
  
  // Calcule et affiche les déplacements angulaires pour chaque actionneur
  double angleX = kinematicCore.computeAngle(NEXUS_X, 3.5, 0.2, 0.2);
  Serial.println((String) "Angle Axe X = " + angleX + " degrés");
  
  double angleY = kinematicCore.computeAngle(NEXUS_Y, 3.5, 0.2, 0.2);
  Serial.println((String) "Angle Axe Y = " + angleY + " degrés");
  
  double angleZ = kinematicCore.computeAngle(NEXUS_Z, 3.5, 0.2, 0.2);
  Serial.println((String) "Angle Axe Z = " + angleZ + " degrés");
}

/**
 * @brief Fonction de boucle principale, actuellement inactive.
 * Espace réservé pour les opérations système continues, si nécessaire.
 */
void loop() {
}