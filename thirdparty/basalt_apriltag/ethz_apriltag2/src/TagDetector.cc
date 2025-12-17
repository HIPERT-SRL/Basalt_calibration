#include <algorithm>
#include <climits>
#include <cmath>
#include <iostream>
#include <map>
#include <vector>

#include <Eigen/Dense>

#include "apriltags/Edge.h"
#include "apriltags/FloatImage.h"
#include "apriltags/GLine2D.h"
#include "apriltags/GLineSegment2D.h"
#include "apriltags/Gaussian.h"
#include "apriltags/GrayModel.h"
#include "apriltags/Gridder.h"
#include "apriltags/Homography33.h"
#include "apriltags/MathUtil.h"
#include "apriltags/Quad.h"
#include "apriltags/Segment.h"
#include "apriltags/TagFamily.h"
#include "apriltags/UnionFindSimple.h"
#include "apriltags/XYWeight.h"

#include "apriltags/TagDetector.h"

//#define DEBUG_APRIL

#ifdef DEBUG_APRIL
#include <opencv/cv.h>
#include <opencv/highgui.h>
#endif

using namespace std;

namespace AprilTags {

std::vector<TagDetection> TagDetector::extractTags(const cv::Mat &image) {
  int width = image.cols;
  int height = image.rows;
  
  //std::cout << "\n[AprilTag Debug] --- Inizio processing frame ---" << std::endl;

  AprilTags::FloatImage fimOrig(width, height);
  int i = 0;
  for (int y = 0; y < height; y++) {
    for (int x = 0; x < width; x++) {
      fimOrig.set(x, y, image.data[i] / 255.);
      i++;
    }
  }
  std::pair<int, int> opticalCenter(width / 2, height / 2);

  // --- Step 1: Preprocess ---
  FloatImage fim = fimOrig;
  float sigma = 0;
  float segSigma = 0.8f;

  if (sigma > 0) {
    int filtsz = ((int)max(3.0f, 3 * sigma)) | 1;
    std::vector<float> filt = Gaussian::makeGaussianFilter(sigma, filtsz);
    fim.filterFactoredCentered(filt, filt);
  }

  // --- Step 2: Gradienti ---
  FloatImage fimSeg;
  if (segSigma > 0) {
    if (segSigma == sigma) {
      fimSeg = fim;
    } else {
      int filtsz = ((int)max(3.0f, 3 * segSigma)) | 1;
      std::vector<float> filt = Gaussian::makeGaussianFilter(segSigma, filtsz);
      fimSeg = fimOrig;
      fimSeg.filterFactoredCentered(filt, filt);
    }
  } else {
    fimSeg = fimOrig;
  }

  FloatImage fimTheta(fimSeg.getWidth(), fimSeg.getHeight());
  FloatImage fimMag(fimSeg.getWidth(), fimSeg.getHeight());

  for (int y = 1; y < fimSeg.getHeight() - 1; y++) {
    for (int x = 1; x < fimSeg.getWidth() - 1; x++) {
      float Ix = fimSeg.get(x + 1, y) - fimSeg.get(x - 1, y);
      float Iy = fimSeg.get(x, y + 1) - fimSeg.get(x, y - 1);
      float mag = Ix * Ix + Iy * Iy;
      float theta = atan2(Iy, Ix);
      fimTheta.set(x, y, theta);
      fimMag.set(x, y, mag);
    }
  }

  // --- Step 3: Edges e UnionFind ---
  UnionFindSimple uf(fimSeg.getWidth() * fimSeg.getHeight());
  vector<Edge> edges(width * height * 4);
  size_t nEdges = 0;

  {
    vector<float> storage(width * height * 4);
    float *tmin = &storage[width * height * 0];
    float *tmax = &storage[width * height * 1];
    float *mmin = &storage[width * height * 2];
    float *mmax = &storage[width * height * 3];

    for (int y = 0; y + 1 < height; y++) {
      for (int x = 0; x + 1 < width; x++) {
        float mag0 = fimMag.get(x, y);
        if (mag0 < Edge::minMag) continue;
        mmax[y * width + x] = mag0;
        mmin[y * width + x] = mag0;
        float theta0 = fimTheta.get(x, y);
        tmin[y * width + x] = theta0;
        tmax[y * width + x] = theta0;
        Edge::calcEdges(theta0, x, y, fimTheta, fimMag, edges, nEdges);
      }
    }
    edges.resize(nEdges);
    std::stable_sort(edges.begin(), edges.end());
    Edge::mergeEdges(edges, uf, tmin, tmax, mmin, mmax);
  }
  //std::cout << "[Step 3] Edges totali rilevati: " << nEdges << std::endl;

  // --- Step 4: Clusters ---
  map<int, vector<XYWeight> > clusters;
  for (int y = 0; y + 1 < fimSeg.getHeight(); y++) {
    for (int x = 0; x + 1 < fimSeg.getWidth(); x++) {
      if (uf.getSetSize(y * fimSeg.getWidth() + x) < Segment::minimumSegmentSize)
        continue;
      int rep = (int)uf.getRepresentative(y * fimSeg.getWidth() + x);
      clusters[rep].push_back(XYWeight(x, y, fimMag.get(x, y)));
    }
  }
  //std::cout << "[Step 4] Clusters (gruppi di pixel): " << clusters.size() << std::endl;

// --- Step 5: Segments fitting ---
  std::vector<Segment> segments;
  int cluster_count = 0;
  int rejected_length = 0;

  for (auto const& [rep, points] : clusters) {
    cluster_count++;
    
    // Fitting ai minimi quadrati: calcola la retta che meglio approssima i pixel
    GLineSegment2D gseg = GLineSegment2D::lsqFitXYW(points);
    
    // CALCOLO LUNGHEZZA
    float length = MathUtil::distance2D(gseg.getP0(), gseg.getP1());
    
    // --- DEBUG CRITICO: Cluster borderline ---
    // Se la lunghezza è molto vicina alla soglia minima (es. 6.0),
    // fluttuazioni nel floating point possono cambiare l'esito.
    if (std::abs(length - Segment::minimumLineLength) < 0.5f) {
        // std::cout << "[Step 5 Debug] Borderline Segment: ClusterID=" << rep 
        //           << " Length=" << std::setprecision(10) << length 
        //           << " Threshold=" << Segment::minimumLineLength 
        //           << " Points=" << points.size() 
        //           << "points[0]=" << points[0].x << ", " << points[0].y << ", " << points[0].weight << std::endl;
        // std::cout << gseg.getP0().first << " " << gseg.getP0().second << std::endl;
        // std::cout << gseg.getP1().first << " " << gseg.getP1().second << std::endl;
    }

    if (length < Segment::minimumLineLength) {
      rejected_length++;
      continue;
    }

    Segment seg;
    float dy = gseg.getP1().second - gseg.getP0().second;
    float dx = gseg.getP1().first - gseg.getP0().first;
    
    // atan2 è sensibile alla precisione decimale di dx e dy
    float tmpTheta = std::atan2(dy, dx);
    seg.setTheta(tmpTheta);
    seg.setLength(length);

    // Orientamento del segmento (Left-hand rule: scuro a sinistra)
    float flip = 0, noflip = 0;
    for (unsigned int i = 0; i < points.size(); i++) {
      XYWeight xyw = points[i];
      float theta = fimTheta.get((int)xyw.x, (int)xyw.y);
      float mag = fimMag.get((int)xyw.x, (int)xyw.y);
      float err = MathUtil::mod2pi(theta - seg.getTheta());
      if (err < 0) noflip += mag; else flip += mag;
    }
    if (flip > noflip) seg.setTheta(seg.getTheta() + (float)M_PI);

    // Impostazione estremi X0, Y0, X1, Y1
    float dot = dx * std::cos(seg.getTheta()) + dy * std::sin(seg.getTheta());
    if (dot > 0) {
      seg.setX0(gseg.getP1().first); seg.setY0(gseg.getP1().second);
      seg.setX1(gseg.getP0().first); seg.setY1(gseg.getP0().second);
    } else {
      seg.setX0(gseg.getP0().first); seg.setY0(gseg.getP0().second);
      seg.setX1(gseg.getP1().first); seg.setY1(gseg.getP1().second);
    }
    segments.push_back(seg);
  }

  // std::cout << "[Step 5 Summary] Totale Cluster processati: " << cluster_count << std::endl;
  // std::cout << "[Step 5 Summary] Segmenti scartati per lunghezza: " << rejected_length << std::endl;
  // std::cout << "[Step 5 Summary] Segmenti finali: " << segments.size() << std::endl;
  // std::cout << "[Step 5] Segmenti validi (linee): " << segments.size() << std::endl;

  // --- Step 6: Gridder e Chaining ---
  Gridder<Segment> gridder(0, 0, width, height, 10);
  for (unsigned int i = 0; i < segments.size(); i++) {
    gridder.add(segments[i].getX0(), segments[i].getY0(), &segments[i]);
  }

  for (unsigned i = 0; i < segments.size(); i++) {
    Segment &parentseg = segments[i];
    GLine2D parentLine(std::pair<float, float>(parentseg.getX0(), parentseg.getY0()),
                       std::pair<float, float>(parentseg.getX1(), parentseg.getY1()));
    Gridder<Segment>::iterator iter = gridder.find(parentseg.getX1(), parentseg.getY1(), 0.5f * parentseg.getLength());
    while (iter.hasNext()) {
      Segment &child = iter.next();
      if (MathUtil::mod2pi(child.getTheta() - parentseg.getTheta()) > 0) continue;
      GLine2D childLine(std::pair<float, float>(child.getX0(), child.getY0()),
                        std::pair<float, float>(child.getX1(), child.getY1()));
      std::pair<float, float> p = parentLine.intersectionWith(childLine);
      if (p.first == -1) continue;
      float parentDist = MathUtil::distance2D(p, std::pair<float, float>(parentseg.getX1(), parentseg.getY1()));
      float childDist = MathUtil::distance2D(p, std::pair<float, float>(child.getX0(), child.getY0()));
      if (max(parentDist, childDist) > parentseg.getLength()) continue;
      parentseg.children.push_back(&child);
    }
  }

  // --- Step 7: Quad search ---
  vector<Quad> quads;
  vector<Segment *> tmp(5);
  for (unsigned int i = 0; i < segments.size(); i++) {
    tmp[0] = &segments[i];
    Quad::search(fimOrig, tmp, segments[i], 0, quads, opticalCenter);
  }
  //std::cout << "[Step 7] Quads candidati (loop di 4 segmenti): " << quads.size() << std::endl;

  // --- Step 8: Decode ---
  std::vector<TagDetection> detections;
  for (unsigned int qi = 0; qi < quads.size(); qi++) {
    Quad &quad = quads[qi];
    GrayModel blackModel, whiteModel;
    const int dd = 2 * thisTagFamily.blackBorder + thisTagFamily.dimension;

    for (int iy = -1; iy <= dd; iy++) {
      float y = (iy + 0.5f) / dd;
      for (int ix = -1; ix <= dd; ix++) {
        float x = (ix + 0.5f) / dd;
        std::pair<float, float> pxy = quad.interpolate01(x, y);
        int irx = (int)(pxy.first + 0.5);
        int iry = (int)(pxy.second + 0.5);
        if (irx < 0 || irx >= width || iry < 0 || iry >= height) continue;
        float v = fim.get(irx, iry);
        if (iy == -1 || iy == dd || ix == -1 || ix == dd)
          whiteModel.addObservation(x, y, v);
        else if (iy == 0 || iy == (dd - 1) || ix == 0 || ix == (dd - 1))
          blackModel.addObservation(x, y, v);
      }
    }

    bool bad = false;
    unsigned long long tagCode = 0;
    for (int iy = thisTagFamily.dimension - 1; iy >= 0; iy--) {
      float y = (thisTagFamily.blackBorder + iy + 0.5f) / dd;
      for (int ix = 0; ix < thisTagFamily.dimension; ix++) {
        float x = (thisTagFamily.blackBorder + ix + 0.5f) / dd;
        std::pair<float, float> pxy = quad.interpolate01(x, y);
        int irx = (int)(pxy.first + 0.5);
        int iry = (int)(pxy.second + 0.5);
        if (irx < 0 || irx >= width || iry < 0 || iry >= height) { bad = true; continue; }
        float threshold = (blackModel.interpolate(x, y) + whiteModel.interpolate(x, y)) * 0.5f;
        float v = fim.get(irx, iry);
        tagCode = tagCode << 1;
        if (v > threshold) tagCode |= 1;
      }
    }

    if (!bad) {
      TagDetection thisTagDetection;
      thisTagFamily.decode(thisTagDetection, tagCode);
      thisTagDetection.homography = quad.homography.getH();
      thisTagDetection.hxy = quad.homography.getCXY();
      float c = std::cos(thisTagDetection.rotation * (float)M_PI / 2);
      float s = std::sin(thisTagDetection.rotation * (float)M_PI / 2);
      Eigen::Matrix3d R; R.setZero(); R(0, 0) = R(1, 1) = c; R(0, 1) = -s; R(1, 0) = s; R(2, 2) = 1;
      Eigen::Matrix3d tmp_mat = thisTagDetection.homography * R;
      thisTagDetection.homography = tmp_mat;

      std::pair<float, float> bottomLeft = thisTagDetection.interpolate(-1, -1);
      int bestRot = -1; float bestDist = FLT_MAX;
      for (int i = 0; i < 4; i++) {
        float const dist = AprilTags::MathUtil::distance2D(bottomLeft, quad.quadPoints[i]);
        if (dist < bestDist) { bestDist = dist; bestRot = i; }
      }
      for (int i = 0; i < 4; i++) thisTagDetection.p[i] = quad.quadPoints[(i + bestRot) % 4];

      if (thisTagDetection.good) {
        thisTagDetection.cxy = quad.interpolate01(0.5f, 0.5f);
        thisTagDetection.observedPerimeter = quad.observedPerimeter;
        if (thisTagDetection.id == 23 || thisTagDetection.id == 5) {
             //std::cout << "[Step 8] Rilevato ID: " << thisTagDetection.id << " (Hamming: " << thisTagDetection.hammingDistance << ")" << std::endl;
        }
        detections.push_back(thisTagDetection);
      }
    }
  }

  // --- Step 9: Overlap resolution ---
  std::vector<TagDetection> goodDetections;
  for (const auto& thisTagDetection : detections) {
    bool newFeature = true;
    for (unsigned int odidx = 0; odidx < goodDetections.size(); odidx++) {
      TagDetection &otherTagDetection = goodDetections[odidx];
      if (thisTagDetection.id != otherTagDetection.id || !thisTagDetection.overlapsTooMuch(otherTagDetection))
        continue;
      newFeature = false;
      if (thisTagDetection.hammingDistance < otherTagDetection.hammingDistance ||
          (thisTagDetection.hammingDistance == otherTagDetection.hammingDistance && 
           thisTagDetection.observedPerimeter > otherTagDetection.observedPerimeter))
        goodDetections[odidx] = thisTagDetection;
    }
    if (newFeature) goodDetections.push_back(thisTagDetection);
  }
  
  //std::cout << "[AprilTag] Fine estrazione. Tag totali restituiti: " << goodDetections.size() << "\n" << std::endl;
  return goodDetections;
}

}  // namespace