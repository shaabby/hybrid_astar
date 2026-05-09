/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2010, Rice University
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Rice University nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Mark Moll
 * Adapted locally to remove OMPL and Boost dependencies.
 */

#include "OmplReedsShepp.hpp"

#include <algorithm>
#include <array>
#include <cassert>
#include <cmath>
#include <limits>

namespace {

using ompl_rs::Path;
using ompl_rs::SegmentType;
using ompl_rs::State;

constexpr double kPi = 3.14159265358979323846;
constexpr double kTwoPi = 2.0 * kPi;
constexpr double kRsEps = 1e-6;
constexpr double kZero = 10.0 * std::numeric_limits<double>::epsilon();

constexpr SegmentType kPathTypes[18][5] = {
    {SegmentType::Left, SegmentType::Right, SegmentType::Left, SegmentType::Nop, SegmentType::Nop},
    {SegmentType::Right, SegmentType::Left, SegmentType::Right, SegmentType::Nop, SegmentType::Nop},
    {SegmentType::Left, SegmentType::Right, SegmentType::Left, SegmentType::Right, SegmentType::Nop},
    {SegmentType::Right, SegmentType::Left, SegmentType::Right, SegmentType::Left, SegmentType::Nop},
    {SegmentType::Left, SegmentType::Right, SegmentType::Straight, SegmentType::Left, SegmentType::Nop},
    {SegmentType::Right, SegmentType::Left, SegmentType::Straight, SegmentType::Right, SegmentType::Nop},
    {SegmentType::Left, SegmentType::Straight, SegmentType::Right, SegmentType::Left, SegmentType::Nop},
    {SegmentType::Right, SegmentType::Straight, SegmentType::Left, SegmentType::Right, SegmentType::Nop},
    {SegmentType::Left, SegmentType::Right, SegmentType::Straight, SegmentType::Right, SegmentType::Nop},
    {SegmentType::Right, SegmentType::Left, SegmentType::Straight, SegmentType::Left, SegmentType::Nop},
    {SegmentType::Right, SegmentType::Straight, SegmentType::Right, SegmentType::Left, SegmentType::Nop},
    {SegmentType::Left, SegmentType::Straight, SegmentType::Left, SegmentType::Right, SegmentType::Nop},
    {SegmentType::Left, SegmentType::Straight, SegmentType::Right, SegmentType::Nop, SegmentType::Nop},
    {SegmentType::Right, SegmentType::Straight, SegmentType::Left, SegmentType::Nop, SegmentType::Nop},
    {SegmentType::Left, SegmentType::Straight, SegmentType::Left, SegmentType::Nop, SegmentType::Nop},
    {SegmentType::Right, SegmentType::Straight, SegmentType::Right, SegmentType::Nop, SegmentType::Nop},
    {SegmentType::Left, SegmentType::Right, SegmentType::Straight, SegmentType::Left, SegmentType::Right},
    {SegmentType::Right, SegmentType::Left, SegmentType::Straight, SegmentType::Right, SegmentType::Left}
};

Path makePath(int type_index,
              double t = std::numeric_limits<double>::max(),
              double u = 0.0,
              double v = 0.0,
              double w = 0.0,
              double x = 0.0) {
    Path path;
    path.type = kPathTypes[type_index];
    path.length = {t, u, v, w, x};
    path.total_length = std::abs(t) + std::abs(u) + std::abs(v)
        + std::abs(w) + std::abs(x);
    path.type_index = type_index;
    return path;
}

double mod2pi(double x) {
    double v = std::fmod(x, kTwoPi);
    if (v < -kPi) {
        v += kTwoPi;
    } else if (v > kPi) {
        v -= kTwoPi;
    }
    return v;
}

void polar(double x, double y, double& r, double& theta) {
    r = std::sqrt(x * x + y * y);
    theta = std::atan2(y, x);
}

void tauOmega(double u,
              double v,
              double xi,
              double eta,
              double phi,
              double& tau,
              double& omega) {
    const double delta = mod2pi(u - v);
    const double a = std::sin(u) - std::sin(delta);
    const double b = std::cos(u) - std::cos(delta) - 1.0;
    const double t1 = std::atan2(eta * a - xi * b, xi * a + eta * b);
    const double t2 = 2.0 * (std::cos(delta) - std::cos(v) - std::cos(u)) + 3.0;
    tau = (t2 < 0.0) ? mod2pi(t1 + kPi) : mod2pi(t1);
    omega = mod2pi(tau - u + v - phi);
}

bool LpSpLp(double x, double y, double phi, double& t, double& u, double& v) {
    polar(x - std::sin(phi), y - 1.0 + std::cos(phi), u, t);
    if (t >= -kZero) {
        v = mod2pi(phi - t);
        if (v >= -kZero) {
            assert(std::abs(u * std::cos(t) + std::sin(phi) - x) < kRsEps);
            assert(std::abs(u * std::sin(t) - std::cos(phi) + 1.0 - y) < kRsEps);
            assert(std::abs(mod2pi(t + v - phi)) < kRsEps);
            return true;
        }
    }
    return false;
}

bool LpSpRp(double x, double y, double phi, double& t, double& u, double& v) {
    double t1 = 0.0;
    double u1 = 0.0;
    polar(x + std::sin(phi), y - 1.0 - std::cos(phi), u1, t1);
    u1 *= u1;
    if (u1 >= 4.0) {
        u = std::sqrt(u1 - 4.0);
        const double theta = std::atan2(2.0, u);
        t = mod2pi(t1 + theta);
        v = mod2pi(t - phi);
        assert(std::abs(2.0 * std::sin(t) + u * std::cos(t) - std::sin(phi) - x) < kRsEps);
        assert(std::abs(-2.0 * std::cos(t) + u * std::sin(t) + std::cos(phi) + 1.0 - y) < kRsEps);
        assert(std::abs(mod2pi(t - v - phi)) < kRsEps);
        return t >= -kZero && v >= -kZero;
    }
    return false;
}

void CSC(double x, double y, double phi, Path& path) {
    double t = 0.0;
    double u = 0.0;
    double v = 0.0;
    double l_min = path.total_length;
    double l = 0.0;

    if (LpSpLp(x, y, phi, t, u, v) && l_min > (l = std::abs(t) + std::abs(u) + std::abs(v))) {
        path = makePath(14, t, u, v);
        l_min = l;
    }
    if (LpSpLp(-x, y, -phi, t, u, v) && l_min > (l = std::abs(t) + std::abs(u) + std::abs(v))) {
        path = makePath(14, -t, -u, -v);
        l_min = l;
    }
    if (LpSpLp(x, -y, -phi, t, u, v) && l_min > (l = std::abs(t) + std::abs(u) + std::abs(v))) {
        path = makePath(15, t, u, v);
        l_min = l;
    }
    if (LpSpLp(-x, -y, phi, t, u, v) && l_min > (l = std::abs(t) + std::abs(u) + std::abs(v))) {
        path = makePath(15, -t, -u, -v);
        l_min = l;
    }
    if (LpSpRp(x, y, phi, t, u, v) && l_min > (l = std::abs(t) + std::abs(u) + std::abs(v))) {
        path = makePath(12, t, u, v);
        l_min = l;
    }
    if (LpSpRp(-x, y, -phi, t, u, v) && l_min > (l = std::abs(t) + std::abs(u) + std::abs(v))) {
        path = makePath(12, -t, -u, -v);
        l_min = l;
    }
    if (LpSpRp(x, -y, -phi, t, u, v) && l_min > (l = std::abs(t) + std::abs(u) + std::abs(v))) {
        path = makePath(13, t, u, v);
        l_min = l;
    }
    if (LpSpRp(-x, -y, phi, t, u, v) && l_min > (l = std::abs(t) + std::abs(u) + std::abs(v))) {
        path = makePath(13, -t, -u, -v);
    }
}

bool LpRmL(double x, double y, double phi, double& t, double& u, double& v) {
    const double xi = x - std::sin(phi);
    const double eta = y - 1.0 + std::cos(phi);
    double u1 = 0.0;
    double theta = 0.0;
    polar(xi, eta, u1, theta);
    if (u1 <= 4.0) {
        u = -2.0 * std::asin(0.25 * u1);
        t = mod2pi(theta + 0.5 * u + kPi);
        v = mod2pi(phi - t + u);
        assert(std::abs(2.0 * (std::sin(t) - std::sin(t - u)) + std::sin(phi) - x) < kRsEps);
        assert(std::abs(2.0 * (-std::cos(t) + std::cos(t - u)) - std::cos(phi) + 1.0 - y) < kRsEps);
        assert(std::abs(mod2pi(t - u + v - phi)) < kRsEps);
        return t >= -kZero && u <= kZero;
    }
    return false;
}

void CCC(double x, double y, double phi, Path& path) {
    double t = 0.0;
    double u = 0.0;
    double v = 0.0;
    double l_min = path.total_length;
    double l = 0.0;

    if (LpRmL(x, y, phi, t, u, v) && l_min > (l = std::abs(t) + std::abs(u) + std::abs(v))) {
        path = makePath(0, t, u, v);
        l_min = l;
    }
    if (LpRmL(-x, y, -phi, t, u, v) && l_min > (l = std::abs(t) + std::abs(u) + std::abs(v))) {
        path = makePath(0, -t, -u, -v);
        l_min = l;
    }
    if (LpRmL(x, -y, -phi, t, u, v) && l_min > (l = std::abs(t) + std::abs(u) + std::abs(v))) {
        path = makePath(1, t, u, v);
        l_min = l;
    }
    if (LpRmL(-x, -y, phi, t, u, v) && l_min > (l = std::abs(t) + std::abs(u) + std::abs(v))) {
        path = makePath(1, -t, -u, -v);
        l_min = l;
    }

    const double xb = x * std::cos(phi) + y * std::sin(phi);
    const double yb = x * std::sin(phi) - y * std::cos(phi);
    if (LpRmL(xb, yb, phi, t, u, v) && l_min > (l = std::abs(t) + std::abs(u) + std::abs(v))) {
        path = makePath(0, v, u, t);
        l_min = l;
    }
    if (LpRmL(-xb, yb, -phi, t, u, v) && l_min > (l = std::abs(t) + std::abs(u) + std::abs(v))) {
        path = makePath(0, -v, -u, -t);
        l_min = l;
    }
    if (LpRmL(xb, -yb, -phi, t, u, v) && l_min > (l = std::abs(t) + std::abs(u) + std::abs(v))) {
        path = makePath(1, v, u, t);
        l_min = l;
    }
    if (LpRmL(-xb, -yb, phi, t, u, v) && l_min > (l = std::abs(t) + std::abs(u) + std::abs(v))) {
        path = makePath(1, -v, -u, -t);
    }
}

bool LpRupLumRm(double x, double y, double phi, double& t, double& u, double& v) {
    const double xi = x + std::sin(phi);
    const double eta = y - 1.0 - std::cos(phi);
    const double rho = 0.25 * (2.0 + std::sqrt(xi * xi + eta * eta));
    if (rho <= 1.0) {
        u = std::acos(rho);
        tauOmega(u, -u, xi, eta, phi, t, v);
        assert(std::abs(2.0 * (std::sin(t) - std::sin(t - u) + std::sin(t - 2.0 * u)) - std::sin(phi) - x) < kRsEps);
        assert(std::abs(2.0 * (-std::cos(t) + std::cos(t - u) - std::cos(t - 2.0 * u)) + std::cos(phi) + 1.0 - y) < kRsEps);
        assert(std::abs(mod2pi(t - 2.0 * u - v - phi)) < kRsEps);
        return t >= -kZero && v <= kZero;
    }
    return false;
}

bool LpRumLumRp(double x, double y, double phi, double& t, double& u, double& v) {
    const double xi = x + std::sin(phi);
    const double eta = y - 1.0 - std::cos(phi);
    const double rho = (20.0 - xi * xi - eta * eta) / 16.0;
    if (rho >= 0.0 && rho <= 1.0) {
        u = -std::acos(rho);
        if (u >= -0.5 * kPi) {
            tauOmega(u, u, xi, eta, phi, t, v);
            assert(std::abs(4.0 * std::sin(t) - 2.0 * std::sin(t - u) - std::sin(phi) - x) < kRsEps);
            assert(std::abs(-4.0 * std::cos(t) + 2.0 * std::cos(t - u) + std::cos(phi) + 1.0 - y) < kRsEps);
            assert(std::abs(mod2pi(t - v - phi)) < kRsEps);
            return t >= -kZero && v >= -kZero;
        }
    }
    return false;
}

void CCCC(double x, double y, double phi, Path& path) {
    double t = 0.0;
    double u = 0.0;
    double v = 0.0;
    double l_min = path.total_length;
    double l = 0.0;

    if (LpRupLumRm(x, y, phi, t, u, v) && l_min > (l = std::abs(t) + 2.0 * std::abs(u) + std::abs(v))) {
        path = makePath(2, t, u, -u, v);
        l_min = l;
    }
    if (LpRupLumRm(-x, y, -phi, t, u, v) && l_min > (l = std::abs(t) + 2.0 * std::abs(u) + std::abs(v))) {
        path = makePath(2, -t, -u, u, -v);
        l_min = l;
    }
    if (LpRupLumRm(x, -y, -phi, t, u, v) && l_min > (l = std::abs(t) + 2.0 * std::abs(u) + std::abs(v))) {
        path = makePath(3, t, u, -u, v);
        l_min = l;
    }
    if (LpRupLumRm(-x, -y, phi, t, u, v) && l_min > (l = std::abs(t) + 2.0 * std::abs(u) + std::abs(v))) {
        path = makePath(3, -t, -u, u, -v);
        l_min = l;
    }
    if (LpRumLumRp(x, y, phi, t, u, v) && l_min > (l = std::abs(t) + 2.0 * std::abs(u) + std::abs(v))) {
        path = makePath(2, t, u, u, v);
        l_min = l;
    }
    if (LpRumLumRp(-x, y, -phi, t, u, v) && l_min > (l = std::abs(t) + 2.0 * std::abs(u) + std::abs(v))) {
        path = makePath(2, -t, -u, -u, -v);
        l_min = l;
    }
    if (LpRumLumRp(x, -y, -phi, t, u, v) && l_min > (l = std::abs(t) + 2.0 * std::abs(u) + std::abs(v))) {
        path = makePath(3, t, u, u, v);
        l_min = l;
    }
    if (LpRumLumRp(-x, -y, phi, t, u, v) && l_min > (l = std::abs(t) + 2.0 * std::abs(u) + std::abs(v))) {
        path = makePath(3, -t, -u, -u, -v);
    }
}

bool LpRmSmLm(double x, double y, double phi, double& t, double& u, double& v) {
    const double xi = x - std::sin(phi);
    const double eta = y - 1.0 + std::cos(phi);
    double rho = 0.0;
    double theta = 0.0;
    polar(xi, eta, rho, theta);
    if (rho >= 2.0) {
        const double r = std::sqrt(rho * rho - 4.0);
        u = 2.0 - r;
        t = mod2pi(theta + std::atan2(r, -2.0));
        v = mod2pi(phi - 0.5 * kPi - t);
        assert(std::abs(2.0 * (std::sin(t) - std::cos(t)) - u * std::sin(t) + std::sin(phi) - x) < kRsEps);
        assert(std::abs(-2.0 * (std::sin(t) + std::cos(t)) + u * std::cos(t) - std::cos(phi) + 1.0 - y) < kRsEps);
        assert(std::abs(mod2pi(t + kPi / 2.0 + v - phi)) < kRsEps);
        return t >= -kZero && u <= kZero && v <= kZero;
    }
    return false;
}

bool LpRmSmRm(double x, double y, double phi, double& t, double& u, double& v) {
    const double xi = x + std::sin(phi);
    const double eta = y - 1.0 - std::cos(phi);
    double rho = 0.0;
    double theta = 0.0;
    polar(-eta, xi, rho, theta);
    if (rho >= 2.0) {
        t = theta;
        u = 2.0 - rho;
        v = mod2pi(t + 0.5 * kPi - phi);
        assert(std::abs(2.0 * std::sin(t) - std::cos(t - v) - u * std::sin(t) - x) < kRsEps);
        assert(std::abs(-2.0 * std::cos(t) - std::sin(t - v) + u * std::cos(t) + 1.0 - y) < kRsEps);
        assert(std::abs(mod2pi(t + kPi / 2.0 - v - phi)) < kRsEps);
        return t >= -kZero && u <= kZero && v <= kZero;
    }
    return false;
}

void CCSC(double x, double y, double phi, Path& path) {
    double t = 0.0;
    double u = 0.0;
    double v = 0.0;
    double l_min = path.total_length - 0.5 * kPi;
    double l = 0.0;

    if (LpRmSmLm(x, y, phi, t, u, v) && l_min > (l = std::abs(t) + std::abs(u) + std::abs(v))) {
        path = makePath(4, t, -0.5 * kPi, u, v);
        l_min = l;
    }
    if (LpRmSmLm(-x, y, -phi, t, u, v) && l_min > (l = std::abs(t) + std::abs(u) + std::abs(v))) {
        path = makePath(4, -t, 0.5 * kPi, -u, -v);
        l_min = l;
    }
    if (LpRmSmLm(x, -y, -phi, t, u, v) && l_min > (l = std::abs(t) + std::abs(u) + std::abs(v))) {
        path = makePath(5, t, -0.5 * kPi, u, v);
        l_min = l;
    }
    if (LpRmSmLm(-x, -y, phi, t, u, v) && l_min > (l = std::abs(t) + std::abs(u) + std::abs(v))) {
        path = makePath(5, -t, 0.5 * kPi, -u, -v);
        l_min = l;
    }
    if (LpRmSmRm(x, y, phi, t, u, v) && l_min > (l = std::abs(t) + std::abs(u) + std::abs(v))) {
        path = makePath(8, t, -0.5 * kPi, u, v);
        l_min = l;
    }
    if (LpRmSmRm(-x, y, -phi, t, u, v) && l_min > (l = std::abs(t) + std::abs(u) + std::abs(v))) {
        path = makePath(8, -t, 0.5 * kPi, -u, -v);
        l_min = l;
    }
    if (LpRmSmRm(x, -y, -phi, t, u, v) && l_min > (l = std::abs(t) + std::abs(u) + std::abs(v))) {
        path = makePath(9, t, -0.5 * kPi, u, v);
        l_min = l;
    }
    if (LpRmSmRm(-x, -y, phi, t, u, v) && l_min > (l = std::abs(t) + std::abs(u) + std::abs(v))) {
        path = makePath(9, -t, 0.5 * kPi, -u, -v);
        l_min = l;
    }

    const double xb = x * std::cos(phi) + y * std::sin(phi);
    const double yb = x * std::sin(phi) - y * std::cos(phi);
    if (LpRmSmLm(xb, yb, phi, t, u, v) && l_min > (l = std::abs(t) + std::abs(u) + std::abs(v))) {
        path = makePath(6, v, u, -0.5 * kPi, t);
        l_min = l;
    }
    if (LpRmSmLm(-xb, yb, -phi, t, u, v) && l_min > (l = std::abs(t) + std::abs(u) + std::abs(v))) {
        path = makePath(6, -v, -u, 0.5 * kPi, -t);
        l_min = l;
    }
    if (LpRmSmLm(xb, -yb, -phi, t, u, v) && l_min > (l = std::abs(t) + std::abs(u) + std::abs(v))) {
        path = makePath(7, v, u, -0.5 * kPi, t);
        l_min = l;
    }
    if (LpRmSmLm(-xb, -yb, phi, t, u, v) && l_min > (l = std::abs(t) + std::abs(u) + std::abs(v))) {
        path = makePath(7, -v, -u, 0.5 * kPi, -t);
        l_min = l;
    }
    if (LpRmSmRm(xb, yb, phi, t, u, v) && l_min > (l = std::abs(t) + std::abs(u) + std::abs(v))) {
        path = makePath(10, v, u, -0.5 * kPi, t);
        l_min = l;
    }
    if (LpRmSmRm(-xb, yb, -phi, t, u, v) && l_min > (l = std::abs(t) + std::abs(u) + std::abs(v))) {
        path = makePath(10, -v, -u, 0.5 * kPi, -t);
        l_min = l;
    }
    if (LpRmSmRm(xb, -yb, -phi, t, u, v) && l_min > (l = std::abs(t) + std::abs(u) + std::abs(v))) {
        path = makePath(11, v, u, -0.5 * kPi, t);
        l_min = l;
    }
    if (LpRmSmRm(-xb, -yb, phi, t, u, v) && l_min > (l = std::abs(t) + std::abs(u) + std::abs(v))) {
        path = makePath(11, -v, -u, 0.5 * kPi, -t);
    }
}

bool LpRmSLmRp(double x, double y, double phi, double& t, double& u, double& v) {
    const double xi = x + std::sin(phi);
    const double eta = y - 1.0 - std::cos(phi);
    double rho = 0.0;
    double theta = 0.0;
    polar(xi, eta, rho, theta);
    if (rho >= 2.0) {
        u = 4.0 - std::sqrt(rho * rho - 4.0);
        if (u <= kZero) {
            t = mod2pi(std::atan2((4.0 - u) * xi - 2.0 * eta,
                                  -2.0 * xi + (u - 4.0) * eta));
            v = mod2pi(t - phi);
            assert(std::abs(4.0 * std::sin(t) - 2.0 * std::cos(t) - u * std::sin(t) - std::sin(phi) - x) < kRsEps);
            assert(std::abs(-4.0 * std::cos(t) - 2.0 * std::sin(t) + u * std::cos(t) + std::cos(phi) + 1.0 - y) < kRsEps);
            assert(std::abs(mod2pi(t - v - phi)) < kRsEps);
            return t >= -kZero && v >= -kZero;
        }
    }
    return false;
}

void CCSCC(double x, double y, double phi, Path& path) {
    double t = 0.0;
    double u = 0.0;
    double v = 0.0;
    double l_min = path.total_length - kPi;
    double l = 0.0;

    if (LpRmSLmRp(x, y, phi, t, u, v) && l_min > (l = std::abs(t) + std::abs(u) + std::abs(v))) {
        path = makePath(16, t, -0.5 * kPi, u, -0.5 * kPi, v);
        l_min = l;
    }
    if (LpRmSLmRp(-x, y, -phi, t, u, v) && l_min > (l = std::abs(t) + std::abs(u) + std::abs(v))) {
        path = makePath(16, -t, 0.5 * kPi, -u, 0.5 * kPi, -v);
        l_min = l;
    }
    if (LpRmSLmRp(x, -y, -phi, t, u, v) && l_min > (l = std::abs(t) + std::abs(u) + std::abs(v))) {
        path = makePath(17, t, -0.5 * kPi, u, -0.5 * kPi, v);
        l_min = l;
    }
    if (LpRmSLmRp(-x, -y, phi, t, u, v) && l_min > (l = std::abs(t) + std::abs(u) + std::abs(v))) {
        path = makePath(17, -t, 0.5 * kPi, -u, 0.5 * kPi, -v);
    }
}

Path getPath(double x, double y, double phi) {
    Path path = makePath(0);
    CSC(x, y, phi, path);
    CCC(x, y, phi, path);
    CCCC(x, y, phi, path);
    CCSC(x, y, phi, path);
    CCSCC(x, y, phi, path);
    return path;
}

} // namespace

namespace ompl_rs {

Path shortestPath(const State& start, const State& goal) {
    const double dx = goal.x - start.x;
    const double dy = goal.y - start.y;
    const double c = std::cos(start.yaw);
    const double s = std::sin(start.yaw);
    const double x = c * dx + s * dy;
    const double y = -s * dx + c * dy;
    const double phi = goal.yaw - start.yaw;
    return getPath(x, y, phi);
}

State interpolate(const State& start, const Path& path, double distance) {
    State local;
    local.yaw = start.yaw;

    double remaining = std::clamp(distance, 0.0, path.total_length);
    for (std::size_t i = 0; i < path.length.size() && remaining > 0.0; ++i) {
        double v = 0.0;
        if (path.length[i] < 0.0) {
            v = std::max(-remaining, path.length[i]);
            remaining += v;
        } else {
            v = std::min(remaining, path.length[i]);
            remaining -= v;
        }

        const double phi = local.yaw;
        switch (path.type[i]) {
        case SegmentType::Left:
            local.x += std::sin(phi + v) - std::sin(phi);
            local.y += -std::cos(phi + v) + std::cos(phi);
            local.yaw = phi + v;
            break;
        case SegmentType::Right:
            local.x += -std::sin(phi - v) + std::sin(phi);
            local.y += std::cos(phi - v) - std::cos(phi);
            local.yaw = phi - v;
            break;
        case SegmentType::Straight:
            local.x += v * std::cos(phi);
            local.y += v * std::sin(phi);
            break;
        case SegmentType::Nop:
            break;
        }
    }

    return {
        start.x + local.x,
        start.y + local.y,
        mod2pi(local.yaw)
    };
}

double distance(const State& start, const State& goal) {
    return shortestPath(start, goal).total_length;
}

} // namespace ompl_rs
