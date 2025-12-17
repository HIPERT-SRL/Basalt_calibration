#include "apriltags/GLineSegment2D.h"
#include <limits>
#include <algorithm>
#include <iostream>  // Aggiunto per std::cout
#include <iomanip>   // Aggiunto per std::setprecision

namespace AprilTags {

GLineSegment2D::GLineSegment2D(const std::pair<float,float>& p0Arg, const std::pair<float,float>& p1Arg)
: line(p0Arg,p1Arg), p0(p0Arg), p1(p1Arg), weight() {}

GLineSegment2D GLineSegment2D::lsqFitXYW(const std::vector<XYWeight>& xyweight) {
    // 1. Fitting della retta infinita (basato su medie e covarianza dei punti)
	//std::cout << "Costrisco la retta" <<  " x: " << xyweight[0].x << " y: " << xyweight[0].y <<  " weight: " << xyweight[0].weight << std::endl;
    GLine2D gline = GLine2D::lsqFitXYW(xyweight);
    
    float maxcoord = -std::numeric_limits<float>::infinity();
    float mincoord = std::numeric_limits<float>::infinity();
    
    // 2. Proiezione dei punti sulla retta per trovare gli estremi
    for (unsigned int i = 0; i < xyweight.size(); i++) {
        std::pair<float,float> p(xyweight[i].x, xyweight[i].y);
        float coord = gline.getLineCoordinate(p);
        //std::cout << "coord: " << std::setprecision(10) << coord << std::endl;
        // Debug per vedere fluttuazioni microscopiche sui singoli punti
        // if (xyweight.size() > 50 && i == 0) {
        //    std::cout << "[LSQ Debug] Point 0 coord: " << std::setprecision(10) << coord << std::endl;
        // }

        maxcoord = std::max(maxcoord, coord);
        mincoord = std::min(mincoord, coord);
    }
    
    float length = maxcoord - mincoord;

    // --- STAMPA DI INDAGINE ---
    // Stampiamo i dettagli dei cluster che producono segmenti vicini alla soglia (es. 6.0)
    if (true) {
        // std::cout << "[LSQ Indagine] Cluster size: " << xyweight.size() 
        //           << " | MinCoord: " << std::setprecision(10) << mincoord 
        //           << " | MaxCoord: " << maxcoord 
        //           << " | Final Length: " << length << std::endl;
    }

    std::pair<float,float> minValue = gline.getPointOfCoordinate(mincoord);
    std::pair<float,float> maxValue = gline.getPointOfCoordinate(maxcoord);
    
    return GLineSegment2D(minValue, maxValue);
}

} // namespace
