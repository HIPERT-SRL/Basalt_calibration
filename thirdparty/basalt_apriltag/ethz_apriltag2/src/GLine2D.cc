#include "apriltags/GLine2D.h"
#include <iostream> 
namespace AprilTags {

GLine2D::GLine2D() 
  : dx(0), dy(0), p(0,0), didNormalizeSlope(false), didNormalizeP(false) {}

GLine2D::GLine2D(float slope, float b) 
  : dx(1), dy(slope), p(0,b), didNormalizeSlope(false), didNormalizeP(false){}

GLine2D::GLine2D(float dX, float dY, const std::pair<float,float>& pt) 
  : dx(dX), dy(dY), p(pt), didNormalizeSlope(false), didNormalizeP(false) {}

GLine2D::GLine2D(const std::pair<float,float>& p1, const std::pair<float,float>& p2)
  : dx(p2.first - p1.first), dy(p2.second - p1.second), p(p1), didNormalizeSlope(false), didNormalizeP(false) {}

float GLine2D::getLineCoordinate(const std::pair<float,float>& pt) {
  normalizeSlope();
  // Calcolo intermedio in double per evitare fluttuazioni FMA/Precision
  double res = (double)pt.first * (double)dx + (double)pt.second * (double)dy;
  return (float)res;
}

std::pair<float,float> GLine2D::getPointOfCoordinate(float coord) {
  normalizeP();
  return std::pair<float,float>(p.first + coord*dx, p.second + coord*dy);
}

std::pair<float,float> GLine2D::intersectionWith(const GLine2D& line) const {
  float m00 = dx;
  float m01 = -line.getDx();
  float m10 = dy;
  float m11 = -line.getDy();

  // determinant of 'm'
  float det = m00*m11 - m01*m10;

  // parallel lines? if so, return (-1,0).
  if (fabs(det) < 1e-10)
    return std::pair<float,float>(-1,0);

  // inverse of 'm'
  float i00 = m11/det;
  // float i11 = m00/det;
  float i01 = -m01/det;
  // float i10 = -m10/det;

  float b00 = line.getFirst() - p.first;
  float b10 = line.getSecond() - p.second;

  float x00 = i00*b00 + i01*b10;

  return std::pair<float,float>(dx*x00+p.first, dy*x00+p.second);
}

GLine2D GLine2D::lsqFitXYW(const std::vector<XYWeight>& xyweights) {
  float n = 0;
  float sum_x = 0;
  float sum_y = 0;

  // Passata 1: Calcolo delle medie pesate (Ex, Ey)
  for (unsigned int i = 0; i < xyweights.size(); i++) {
    float alpha = xyweights[i].weight;
    sum_x += xyweights[i].x * alpha;
    sum_y += xyweights[i].y * alpha;
    n     += alpha;
  }

  if (n <= 0) return GLine2D(0, 1, std::make_pair(0.0f, 0.0f));

  float Ex = sum_x / n;
  float Ey = sum_y / n;

  // Passata 2: Calcolo delle covarianze usando gli scarti dalla media
  // Questo evita di maneggiare numeri enormi (x*x) e previene la cancellazione catastrofica.
  float Cxx = 0, Cyy = 0, Cxy = 0;

  for (unsigned int i = 0; i < xyweights.size(); i++) {
    float alpha = xyweights[i].weight;
    float dx = xyweights[i].x - Ex;
    float dy = xyweights[i].y - Ey;

    Cxx += dx * dx * alpha;
    Cyy += dy * dy * alpha;
    Cxy += dx * dy * alpha;
  }

  Cxx /= n;
  Cyy /= n;
  Cxy /= n;

  // Debug stabile
  // std::cout << "[Stabile] Ex: " << Ex << " Ey: " << Ey << std::endl;
  // std::cout << "[Stabile] Cxx: " << Cxx << " Cxy: " << Cxy << " Cyy: " << Cyy << std::endl;

  // Trova la direzione dominante
  float phi = 0.5f * std::atan2(-2.0f * Cxy, (Cyy - Cxx));
  
  std::pair<float, float> pts(Ex, Ey);

  //std::cout << "phi: " << phi << " | pts: " << pts.first << "," << pts.second << std::endl;

  return GLine2D(-std::sin(phi), std::cos(phi), pts);
}

void GLine2D::normalizeSlope() {
  if ( !didNormalizeSlope ) {
    float mag = std::sqrt(dx*dx+dy*dy);
    dx /= mag;
    dy /= mag;
    didNormalizeSlope=true;
  }
}

void GLine2D::normalizeP() {
  if ( !didNormalizeP ) {
    normalizeSlope();
    // we already have a point (P) on the line, and we know the line vector U
    // and its perpendicular vector V: so, P'=P.*V *V
    float dotprod = -dy*p.first + dx*p.second;
    p = std::pair<float,float>(-dy*dotprod, dx*dotprod);
    didNormalizeP = true;
  }
}

} // namespace
