#ifndef INVERSE_KINEMATICS_H
#define INVERSE_KINEMATICS_H
#include "Arduino.h"

/**
 * @brief Constantes d'énumération pour identifier les nœuds des actionneurs dans le système cinématique.
 * Elles définissent les indices pour les trois axes principaux du contrôle de mouvement.
 */
#define NEXUS_X 0
#define NEXUS_Y 1
#define NEXUS_Z 2

/**
 * @class CelestialRig
 * @brief Encapsule la configuration cinématique pour un mécanisme de stabilisation multi-axes.
 * Cette classe fournit des méthodes pour calculer les déplacements angulaires des actionneurs en fonction des entrées spatiales.
 */
class CelestialRig {
public:
  /**
   * @brief Construit un modèle cinématique avec les paramètres géométriques spécifiés.
   * @param v1 Premier coefficient dimensionnel pour la géométrie du système.
   * @param v2 Deuxième coefficient dimensionnel pour la géométrie du système.
   * @param v3 Troisième coefficient dimensionnel pour la géométrie du système.
   * @param v4 Quatrième coefficient dimensionnel pour la géométrie du système.
   */
  CelestialRig(double v1, double v2, double v3, double v4);

  /**
   * @brief Calcule le déplacement angulaire pour un nœud d'actionneur spécifié.
   * @param node Identifiant de l'actionneur (NEXUS_X, NEXUS_Y ou NEXUS_Z).
   * @param zPos Entrée de position verticale pour le calcul cinématique.
   * @param xVec Entrée du vecteur horizontal sur l'axe X.
   * @param yVec Entrée du vecteur horizontal sur l'axe Y.
   * @return La valeur angulaire calculée pour le nœud spécifié.
   */
  double computeAngle(uint8_t node, double zPos, double xVec, double yVec);
};

#endif