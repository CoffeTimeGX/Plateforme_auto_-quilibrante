#include "InverseKinematics.h"

/**
 * @brief Paramètres de configuration globaux pour la géométrie du système cinématique.
 */
double geoBaseSpan;   // Décalage spatial du centre de la base aux sommets des coins
double geoPlateSpan;  // Décalage spatial du centre de la plateforme aux sommets des coins
double linkPrimary;   // Longueur du composant de liaison primaire
double linkSecondary; // Longueur du composant de liaison secondaire

/**
 * @brief Variables de calcul pour les opérations vectorielles et angulaires.
 */
double vecNormMag;    // Magnitude du vecteur directionnel normalisé
double vecNormZ;      // Composante Z du vecteur directionnel normalisé
double coordX, coordY, coordZ; // Coordonnées temporaires pour les vecteurs des actionneurs
double vecLength;     // Longueur calculée du vecteur de l'actionneur
double arcResult;     // Déplacement angulaire résultant pour les actionneurs

/**
 * @brief Initialise le système cinématique avec les paramètres géométriques.
 * @param v1 Distance du centre de la base aux coins.
 * @param v2 Distance du centre de la plateforme aux coins.
 * @param v3 Longueur de la première liaison.
 * @param v4 Longueur de la seconde liaison.
 */
CelestialRig::CelestialRig(double v1, double v2, double v3, double v4) {
  geoBaseSpan = v1;
  geoPlateSpan = v2;
  linkPrimary = v3;
  linkSecondary = v4;
}

/**
 * @brief Calcule le déplacement angulaire pour un nœud d'actionneur spécifié.
 * @param node Identifiant de l'actionneur (NEXUS_X, NEXUS_Y ou NEXUS_Z).
 * @param zPos Entrée de position verticale.
 * @param xVec Composante X du vecteur directionnel.
 * @param yVec Composante Y du vecteur directionnel.
 * @return Déplacement angulaire en degrés pour l'actionneur spécifié.
 */
double CelestialRig::computeAngle(uint8_t node, double zPos, double xVec, double yVec) {
  // Normalise le vecteur directionnel
  vecNormMag = sqrt(xVec * xVec + yVec * yVec + 1.0);
  double normX = xVec / vecNormMag;
  double normY = yVec / vecNormMag;
  vecNormZ = 1.0 / vecNormMag;

  // Calcule les angles spécifiques à l'actionneur
  switch (node) {
    case NEXUS_X: {
      // Calculs pour l'actionneur X
      coordY = geoBaseSpan + (geoPlateSpan / 2.0) * (1.0 - (normX * normX + 3.0 * vecNormZ * vecNormZ + 3.0 * vecNormZ) / (vecNormZ + 1.0 - normX * normX + (normX * normX * normX * normX - 3.0 * normX * normX * normY * normY) / ((vecNormZ + 1.0) * (vecNormZ + 1.0 - normX * normX))));
      coordZ = zPos + geoPlateSpan * normY;
      vecLength = sqrt(coordY * coordY + coordZ * coordZ);
      arcResult = acos(coordY / vecLength) + acos((vecLength * vecLength + linkPrimary * linkPrimary - linkSecondary * linkSecondary) / (2.0 * vecLength * linkPrimary));
      break;
    }
    case NEXUS_Y: {
      // Calculs pour l'actionneur Y
      coordX = (sqrt(3.0) / 2.0) * (geoPlateSpan * (1.0 - (normX * normX + sqrt(3.0) * normX * normY) / (vecNormZ + 1.0)) - geoBaseSpan);
      coordY = coordX / sqrt(3.0);
      coordZ = zPos - (geoPlateSpan / 2.0) * (sqrt(3.0) * normX + normY);
      vecLength = sqrt(coordX * coordX + coordY * coordY + coordZ * coordZ);
      arcResult = acos((sqrt(3.0) * coordX + coordY) / (-2.0 * vecLength)) + acos((vecLength * vecLength + linkPrimary * linkPrimary - linkSecondary * linkSecondary) / (2.0 * vecLength * linkPrimary));
      break;
    }
    case NEXUS_Z: {
      // Calculs pour l'actionneur Z
      coordX = (sqrt(3.0) / 2.0) * (geoBaseSpan - geoPlateSpan * (1.0 - (normX * normX - sqrt(3.0) * normX * normY) / (vecNormZ + 1.0)));
      coordY = -coordX / sqrt(3.0);
      coordZ = zPos + (geoPlateSpan / 2.0) * (sqrt(3.0) * normX - normY);
      vecLength = sqrt(coordX * coordX + coordY * coordY + coordZ * coordZ);
      arcResult = acos((sqrt(3.0) * coordX - coordY) / (2.0 * vecLength)) + acos((vecLength * vecLength + linkPrimary * linkPrimary - linkSecondary * linkSecondary) / (2.0 * vecLength * linkPrimary));
      break;
    }
  }

  // Convertit le résultat en degrés
  return arcResult * (180.0 / 3.141592653589793);
}