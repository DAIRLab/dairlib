#pragma once

// copy of https://github.com/ZJU-FAST-Lab/large_scale_traj_optimizer
/*
    MIT License

    Copyright (c) 2020 Zhepei Wang (wangzhepei@live.com)
                  2023 Brian Acosta (bjacosta@seas.upenn.edu)

    Permission is hereby granted, free of charge, to any person obtaining a copy
    of this software and associated documentation files (the "Software"), to deal
    in the Software without restriction, including without limitation the rights
    to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
    copies of the Software, and to permit persons to whom the Software is
    furnished to do so, subject to the following conditions:

    The above copyright notice and this permission notice shall be included in all
    copies or substantial portions of the Software.

    THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
    IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
    FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
    AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
    LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
    OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
    SOFTWARE.
*/

#include "Eigen/Eigen"

#include <iostream>
#include <cmath>
#include <set>
#include <vector>
#include <limits>

#include "drake/common/trajectories/piecewise_polynomial.h"

namespace dairlib::minsnap {

#ifndef DBL_EPSILON
#define DBL_EPSILON std::numeric_limits<double>::epsilon()
#endif
#ifndef FLT_EPSILON
#define FLT_EPSILON std::numeric_limits<float>::epsilon()
#endif
namespace RootFinderParam
{
constexpr size_t highestOrder = 64;
}

namespace RootFinderPriv
{

inline int polyMod(double *u, double *v, double *r, int lu, int lv)
// Modulus of u(x)/v(x)
// The leading coefficient of v, i.e., v[0], must be 1.0 or -1.0
// The length of u, v, and r are lu, lv, and lu, respectively
{
  int orderu = lu - 1;
  int orderv = lv - 1;

  memcpy(r, u, lu * sizeof(double));

  if (v[0] < 0.0)
  {
    for (int i = orderv + 1; i <= orderu; i += 2)
    {
      r[i] = -r[i];
    }
    for (int i = 0; i <= orderu - orderv; i++)
    {
      for (int j = i + 1; j <= orderv + i; j++)
      {
        r[j] = -r[j] - r[i] * v[j - i];
      }
    }
  }
  else
  {
    for (int i = 0; i <= orderu - orderv; i++)
    {
      for (int j = i + 1; j <= orderv + i; j++)
      {
        r[j] = r[j] - r[i] * v[j - i];
      }
    }
  }

  int k = orderv - 1;
  while (k >= 0 && fabs(r[orderu - k]) < DBL_EPSILON)
  {
    r[orderu - k] = 0.0;
    k--;
  }

  return (k <= 0) ? 1 : (k + 1);
}

inline double polyEval(double *p, int len, double x)
// Evaluate the polynomial p(x), which has len coefficients
// Note: Horner scheme should not be employed here !!!
// Horner scheme has bad numerical stability despite of its efficiency.
// These errors are particularly troublesome for root-finding algorithms.
// When the polynomial is evaluated near a zero, catastrophic
// cancellation (subtracting two nearby numbers) is guaranteed to occur.
// Therefore, Horner scheme may slow down some root-finding algorithms.
{
  double retVal = 0.0;

  if (len > 0)
  {
    if (fabs(x) < DBL_EPSILON)
    {
      retVal = p[len - 1];
    }
    else if (x == 1.0)
    {
      for (int i = len - 1; i >= 0; i--)
      {
        retVal += p[i];
      }
    }
    else
    {
      double xn = 1.0;

      for (int i = len - 1; i >= 0; i--)
      {
        retVal += p[i] * xn;
        xn *= x;
      }
    }
  }

  return retVal;
}

inline std::set<double> solveCub(double a, double b, double c, double d)
// Calculate all roots of a*x^3 + b*x^2 + c*x + d = 0
{
  std::set<double> roots;

  constexpr double cos120 = -0.50;
  constexpr double sin120 = 0.866025403784438646764;

  if (fabs(d) < DBL_EPSILON)
  {
    // First solution is x = 0
    roots.insert(0.0);

    // Converting to a quadratic equation
    d = c;
    c = b;
    b = a;
    a = 0.0;
  }

  if (fabs(a) < DBL_EPSILON)
  {
    if (fabs(b) < DBL_EPSILON)
    {
      // Linear equation
      if (fabs(c) > DBL_EPSILON)
        roots.insert(-d / c);
    }
    else
    {
      // Quadratic equation
      double discriminant = c * c - 4.0 * b * d;
      if (discriminant >= 0)
      {
        double inv2b = 1.0 / (2.0 * b);
        double y = sqrt(discriminant);
        roots.insert((-c + y) * inv2b);
        roots.insert((-c - y) * inv2b);
      }
    }
  }
  else
  {
    // Cubic equation
    double inva = 1.0 / a;
    double invaa = inva * inva;
    double bb = b * b;
    double bover3a = b * (1.0 / 3.0) * inva;
    double p = (3.0 * a * c - bb) * (1.0 / 3.0) * invaa;
    double halfq = (2.0 * bb * b - 9.0 * a * b * c + 27.0 * a * a * d) * (0.5 / 27.0) * invaa * inva;
    double yy = p * p * p / 27.0 + halfq * halfq;

    if (yy > DBL_EPSILON)
    {
      // Sqrt is positive: one real solution
      double y = sqrt(yy);
      double uuu = -halfq + y;
      double vvv = -halfq - y;
      double www = fabs(uuu) > fabs(vvv) ? uuu : vvv;
      double w = (www < 0) ? -pow(fabs(www), 1.0 / 3.0) : pow(www, 1.0 / 3.0);
      roots.insert(w - p / (3.0 * w) - bover3a);
    }
    else if (yy < -DBL_EPSILON)
    {
      // Sqrt is negative: three real solutions
      double x = -halfq;
      double y = sqrt(-yy);
      double theta;
      double r;
      double ux;
      double uyi;
      // Convert to polar form
      if (fabs(x) > DBL_EPSILON)
      {
        theta = (x > 0.0) ? atan(y / x) : (atan(y / x) + M_PI);
        r = sqrt(x * x - yy);
      }
      else
      {
        // Vertical line
        theta = M_PI / 2.0;
        r = y;
      }
      // Calculate cube root
      theta /= 3.0;
      r = pow(r, 1.0 / 3.0);
      // Convert to complex coordinate
      ux = cos(theta) * r;
      uyi = sin(theta) * r;
      // First solution
      roots.insert(ux + ux - bover3a);
      // Second solution, rotate +120 degrees
      roots.insert(2.0 * (ux * cos120 - uyi * sin120) - bover3a);
      // Third solution, rotate -120 degrees
      roots.insert(2.0 * (ux * cos120 + uyi * sin120) - bover3a);
    }
    else
    {
      // Sqrt is zero: two real solutions
      double www = -halfq;
      double w = (www < 0.0) ? -pow(fabs(www), 1.0 / 3.0) : pow(www, 1.0 / 3.0);
      // First solution
      roots.insert(w + w - bover3a);
      // Second solution, rotate +120 degrees
      roots.insert(2.0 * w * cos120 - bover3a);
    }
  }
  return roots;
}

inline int solveResolvent(double *x, double a, double b, double c)
// Solve resolvent eqaution of corresponding Quartic equation
// The input x must be of length 3
// Number of zeros are returned
{
  double a2 = a * a;
  double q = (a2 - 3.0 * b) / 9.0;
  double r = (a * (2.0 * a2 - 9.0 * b) + 27.0 * c) / 54.0;
  double r2 = r * r;
  double q3 = q * q * q;
  double A, B;
  if (r2 < q3)
  {
    double t = r / sqrt(q3);
    if (t < -1.0)
    {
      t = -1.0;
    }
    if (t > 1.0)
    {
      t = 1.0;
    }
    t = acos(t);
    a /= 3.0;
    q = -2.0 * sqrt(q);
    x[0] = q * cos(t / 3.0) - a;
    x[1] = q * cos((t + M_PI * 2.0) / 3.0) - a;
    x[2] = q * cos((t - M_PI * 2.0) / 3.0) - a;
    return 3;
  }
  else
  {
    A = -pow(fabs(r) + sqrt(r2 - q3), 1.0 / 3.0);
    if (r < 0.0)
    {
      A = -A;
    }
    B = (0.0 == A ? 0.0 : q / A);

    a /= 3.0;
    x[0] = (A + B) - a;
    x[1] = -0.5 * (A + B) - a;
    x[2] = 0.5 * sqrt(3.0) * (A - B);
    if (fabs(x[2]) < DBL_EPSILON)
    {
      x[2] = x[1];
      return 2;
    }

    return 1;
  }
}

inline std::set<double> solveQuartMonic(double a, double b, double c, double d)
// Calculate all roots of the monic quartic equation:
// x^4 + a*x^3 + b*x^2 + c*x +d = 0
{
  std::set<double> roots;

  double a3 = -b;
  double b3 = a * c - 4.0 * d;
  double c3 = -a * a * d - c * c + 4.0 * b * d;

  // Solve the resolvent: y^3 - b*y^2 + (ac - 4*d)*y - a^2*d - c^2 + 4*b*d = 0
  double x3[3];
  int iZeroes = solveResolvent(x3, a3, b3, c3);

  double q1, q2, p1, p2, D, sqrtD, y;

  y = x3[0];
  // Choosing Y with maximal absolute value.
  if (iZeroes != 1)
  {
    if (fabs(x3[1]) > fabs(y))
    {
      y = x3[1];
    }
    if (fabs(x3[2]) > fabs(y))
    {
      y = x3[2];
    }
  }

  // h1 + h2 = y && h1*h2 = d  <=>  h^2 - y*h + d = 0    (h === q)

  D = y * y - 4.0 * d;
  if (fabs(D) < DBL_EPSILON) //In other words: D == 0
  {
    q1 = q2 = y * 0.5;
    // g1 + g2 = a && g1 + g2 = b - y   <=>   g^2 - a*g + b - y = 0    (p === g)
    D = a * a - 4.0 * (b - y);
    if (fabs(D) < DBL_EPSILON) //In other words: D == 0
    {
      p1 = p2 = a * 0.5;
    }
    else
    {
      sqrtD = sqrt(D);
      p1 = (a + sqrtD) * 0.5;
      p2 = (a - sqrtD) * 0.5;
    }
  }
  else
  {
    sqrtD = sqrt(D);
    q1 = (y + sqrtD) * 0.5;
    q2 = (y - sqrtD) * 0.5;
    // g1 + g2 = a && g1*h2 + g2*h1 = c   ( && g === p )  Krammer
    p1 = (a * q1 - c) / (q1 - q2);
    p2 = (c - a * q2) / (q1 - q2);
  }

  // Solve the quadratic equation: x^2 + p1*x + q1 = 0
  D = p1 * p1 - 4.0 * q1;
  if (fabs(D) < DBL_EPSILON)
  {
    roots.insert(-p1 * 0.5);
  }
  else if (D > 0.0)
  {
    sqrtD = sqrt(D);
    roots.insert((-p1 + sqrtD) * 0.5);
    roots.insert((-p1 - sqrtD) * 0.5);
  }

  // Solve the quadratic equation: x^2 + p2*x + q2 = 0
  D = p2 * p2 - 4.0 * q2;
  if (fabs(D) < DBL_EPSILON)
  {
    roots.insert(-p2 * 0.5);
  }
  else if (D > 0.0)
  {
    sqrtD = sqrt(D);
    roots.insert((-p2 + sqrtD) * 0.5);
    roots.insert((-p2 - sqrtD) * 0.5);
  }

  return roots;
}

inline std::set<double> solveQuart(double a, double b, double c, double d, double e)
// Calculate the quartic equation: a*x^4 + b*x^3 + c*x^2 + d*x + e = 0
// All coefficients can be zero
{
  if (fabs(a) < DBL_EPSILON)
  {
    return solveCub(b, c, d, e);
  }
  else
  {
    return solveQuartMonic(b / a, c / a, d / a, e / a);
  }
}

inline std::set<double> eigenSolveRealRoots(const Eigen::VectorXd &coeffs, double lbound, double ubound, double tol)
// Calculate roots of coeffs(x) inside (lbound, rbound) by computing eigen values of its companion matrix
// Complex roots with magnitude of imaginary part less than tol are considered real
{
  std::set<double> rts;

  int order = (int)coeffs.size() - 1;
  Eigen::VectorXd monicCoeffs(order + 1);
  monicCoeffs << 1.0, coeffs.tail(order) / coeffs(0);

  Eigen::MatrixXd companionMat(order, order);
  companionMat.setZero();
  companionMat(0, order - 1) = -monicCoeffs(order);
  for (int i = 1; i < order; i++)
  {
    companionMat(i, i - 1) = 1.0;
    companionMat(i, order - 1) = -monicCoeffs(order - i);
  }
  Eigen::VectorXcd eivals = companionMat.eigenvalues();
  double real;
  int eivalsNum = eivals.size();
  for (int i = 0; i < eivalsNum; i++)
  {
    real = eivals(i).real();
    if (eivals(i).imag() < tol && real > lbound && real < ubound)
      rts.insert(real);
  }

  return rts;
}

inline double numSignVar(double x, double **sturmSeqs, int *szSeq, int len)
// Calculate the number of sign variations of the Sturm sequences at x
// The i-th sequence with size szSeq[i] stored in sturmSeqs[i][], 0 <= i < len
{
  double y, lasty;
  int signVar = 0;
  lasty = polyEval(sturmSeqs[0], szSeq[0], x);
  for (int i = 1; i < len; i++)
  {
    y = polyEval(sturmSeqs[i], szSeq[i], x);
    if (lasty == 0.0 || lasty * y < 0.0)
    {
      ++signVar;
    }
    lasty = y;
  }

  return signVar;
};

inline void polyDeri(double *coeffs, double *dcoeffs, int len)
// Calculate the derivative poly coefficients of a given poly
{
  int horder = len - 1;
  for (int i = 0; i < horder; i++)
  {
    dcoeffs[i] = (horder - i) * coeffs[i];
  }
  return;
}

template <typename F, typename DF>
inline double safeNewton(const F &func, const DF &dfunc,
                         const double &l, const double &h,
                         const double &tol, const int &maxIts)
// Safe Newton Method
// Requirements: f(l)*f(h)<=0
{
  double xh, xl;
  double fl = func(l);
  double fh = func(h);
  if (fl == 0.0)
  {
    return l;
  }
  if (fh == 0.0)
  {
    return h;
  }
  if (fl < 0.0)
  {
    xl = l;
    xh = h;
  }
  else
  {
    xh = l;
    xl = h;
  }

  double rts = 0.5 * (xl + xh);
  double dxold = fabs(xh - xl);
  double dx = dxold;
  double f = func(rts);
  double df = dfunc(rts);
  double temp;
  for (int j = 0; j < maxIts; j++)
  {
    if ((((rts - xh) * df - f) * ((rts - xl) * df - f) > 0.0) ||
        (fabs(2.0 * f) > fabs(dxold * df)))
    {
      dxold = dx;
      dx = 0.5 * (xh - xl);
      rts = xl + dx;
      if (xl == rts)
      {
        break;
      }
    }
    else
    {
      dxold = dx;
      dx = f / df;
      temp = rts;
      rts -= dx;
      if (temp == rts)
      {
        break;
      }
    }

    if (fabs(dx) < tol)
    {
      break;
    }

    f = func(rts);
    df = dfunc(rts);
    if (f < 0.0)
    {
      xl = rts;
    }
    else
    {
      xh = rts;
    }
  }

  return rts;
}

inline double shrinkInterval(double *coeffs, int numCoeffs, double lbound, double ubound, double tol)
// Calculate a single zero of poly coeffs(x) inside [lbound, ubound]
// Requirements: coeffs(lbound)*coeffs(ubound) < 0, lbound < ubound
{
  double *dcoeffs = new double[numCoeffs - 1];
  polyDeri(coeffs, dcoeffs, numCoeffs);
  auto func = [&coeffs, &numCoeffs](double x) { return polyEval(coeffs, numCoeffs, x); };
  auto dfunc = [&dcoeffs, &numCoeffs](double x) { return polyEval(dcoeffs, numCoeffs - 1, x); };
  constexpr int maxDblIts = 128;
  double rts = safeNewton(func, dfunc, lbound, ubound, tol, maxDblIts);
  delete[] dcoeffs;
  return rts;
}

inline void recurIsolate(double l, double r, double fl, double fr, int lnv, int rnv,
                         double tol, double **sturmSeqs, int *szSeq, int len,
                         std::set<double> &rts)
// Isolate all roots of sturmSeqs[0](x) inside interval (l, r) recursively and store them in rts
// Requirements: fl := sturmSeqs[0](l) != 0, fr := sturmSeqs[0](r) != 0, l < r,
//               lnv != rnv, lnv = numSignVar(l), rnv = numSignVar(r)
//               sturmSeqs[0](x) must have at least one root inside (l, r)
{
  int nrts = lnv - rnv;
  double fm;
  double m;

  if (nrts == 0)
  {
    return;
  }
  else if (nrts == 1)
  {
    if (fl * fr < 0)
    {
      rts.insert(shrinkInterval(sturmSeqs[0], szSeq[0], l, r, tol));
      return;
    }
    else
    {
      // Bisect when non of above works
      int maxDblIts = 128;

      for (int i = 0; i < maxDblIts; i++)
      {
        // Calculate the root with even multiplicity
        if (fl * fr < 0)
        {
          rts.insert(shrinkInterval(sturmSeqs[1], szSeq[1], l, r, tol));
          return;
        }

        m = (l + r) / 2.0;
        fm = polyEval(sturmSeqs[0], szSeq[0], m);

        if (fm == 0 || fabs(r - l) < tol)
        {
          rts.insert(m);
          return;
        }
        else
        {
          if (lnv == numSignVar(m, sturmSeqs, szSeq, len))
          {
            l = m;
            fl = fm;
          }
          else
          {
            r = m;
            fr = fm;
          }
        }
      }

      rts.insert(m);
      return;
    }
  }
  else if (nrts > 1)
  {
    // More than one root exists in the interval
    int maxDblIts = 128;

    int mnv;
    int bias = 0;
    bool biased = false;
    for (int i = 0; i < maxDblIts; i++)
    {
      bias = biased ? bias : 0;
      if (!biased)
      {
        m = (l + r) / 2.0;
      }
      else
      {
        m = (r - l) / pow(2.0, bias + 1.0) + l;
        biased = false;
      }
      mnv = numSignVar(m, sturmSeqs, szSeq, len);

      if (fabs(r - l) < tol)
      {
        rts.insert(m);
        return;
      }
      else
      {
        fm = polyEval(sturmSeqs[0], szSeq[0], m);
        if (fm == 0)
        {
          bias++;
          biased = true;
        }
        else if (lnv != mnv && rnv != mnv)
        {
          recurIsolate(l, m, fl, fm, lnv, mnv, tol, sturmSeqs, szSeq, len, rts);
          recurIsolate(m, r, fm, fr, mnv, rnv, tol, sturmSeqs, szSeq, len, rts);
          return;
        }
        else if (lnv == mnv)
        {
          l = m;
          fl = fm;
        }
        else
        {
          r = m;
          fr = fm;
        }
      }
    }

    rts.insert(m);
    return;
  }
};

inline std::set<double> isolateRealRoots(const Eigen::VectorXd &coeffs, double lbound, double ubound, double tol)
// Calculate roots of coeffs(x) inside (lbound, rbound) leveraging Sturm theory
// Requirement: leading coefficient must be nonzero
//              coeffs(lbound) != 0, coeffs(rbound) != 0, lbound < rbound
{
  std::set<double> rts;

  // Calculate monic coefficients
  int order = (int)coeffs.size() - 1;
  Eigen::VectorXd monicCoeffs(order + 1);
  monicCoeffs << 1.0, coeffs.tail(order) / coeffs(0);

  // Calculate Cauchy’s bound for the roots of a polynomial
  double rho_c = 1 + monicCoeffs.tail(order).cwiseAbs().maxCoeff();

  // Calculate Kojima’s bound for the roots of a polynomial
  Eigen::VectorXd nonzeroCoeffs(order + 1);
  nonzeroCoeffs.setZero();
  int nonzeros = 0;
  double tempEle;
  for (int i = 0; i < order + 1; i++)
  {
    tempEle = monicCoeffs(i);
    if (fabs(tempEle) >= DBL_EPSILON)
    {
      nonzeroCoeffs(nonzeros++) = tempEle;
    }
  }
  nonzeroCoeffs = nonzeroCoeffs.head(nonzeros).eval();
  Eigen::VectorXd kojimaVec = nonzeroCoeffs.tail(nonzeros - 1).cwiseQuotient(nonzeroCoeffs.head(nonzeros - 1)).cwiseAbs();
  kojimaVec.tail(1) /= 2.0;
  double rho_k = 2.0 * kojimaVec.maxCoeff();

  // Choose a sharper one then loosen it by 1.0 to get an open interval
  double rho = std::min(rho_c, rho_k) + 1.0;

  // Tighten the bound to search in
  lbound = std::max(lbound, -rho);
  ubound = std::min(ubound, rho);

  // Build Sturm sequence
  int len = monicCoeffs.size();
  double sturmSeqs[(RootFinderParam::highestOrder + 1) * (RootFinderParam::highestOrder + 1)];
  int szSeq[RootFinderParam::highestOrder + 1] = {0}; // Explicit ini as zero (gcc may neglect this in -O3)
  double *offsetSeq[RootFinderParam::highestOrder + 1];
  int num = 0;

  for (int i = 0; i < len; i++)
  {
    sturmSeqs[i] = monicCoeffs(i);
    sturmSeqs[i + 1 + len] = (order - i) * sturmSeqs[i] / order;
  }
  szSeq[0] = len;
  szSeq[1] = len - 1;
  offsetSeq[0] = sturmSeqs + len - szSeq[0];
  offsetSeq[1] = sturmSeqs + 2 * len - szSeq[1];

  num += 2;

  bool remainderConstant = false;
  int idx = 0;
  while (!remainderConstant)
  {
    szSeq[idx + 2] = polyMod(offsetSeq[idx],
                             offsetSeq[idx + 1],
                             &(sturmSeqs[(idx + 3) * len - szSeq[idx]]),
                             szSeq[idx], szSeq[idx + 1]);
    offsetSeq[idx + 2] = sturmSeqs + (idx + 3) * len - szSeq[idx + 2];

    remainderConstant = szSeq[idx + 2] == 1;
    for (int i = 1; i < szSeq[idx + 2]; i++)
    {
      offsetSeq[idx + 2][i] /= -fabs(offsetSeq[idx + 2][0]);
    }
    offsetSeq[idx + 2][0] = offsetSeq[idx + 2][0] > 0.0 ? -1.0 : 1.0;
    num++;
    idx++;
  }

  // Isolate all distinct roots inside the open interval recursively
  recurIsolate(lbound, ubound,
               polyEval(offsetSeq[0], szSeq[0], lbound),
               polyEval(offsetSeq[0], szSeq[0], ubound),
               numSignVar(lbound, offsetSeq, szSeq, len),
               numSignVar(ubound, offsetSeq, szSeq, len),
               tol, offsetSeq, szSeq, len, rts);

  return rts;
}

} // namespace RootFinderPriv

namespace RootFinder
{

inline Eigen::VectorXd polyConv(const Eigen::VectorXd &lCoef, const Eigen::VectorXd &rCoef)
// Calculate the convolution of lCoef(x) and rCoef(x)
{
  Eigen::VectorXd result(lCoef.size() + rCoef.size() - 1);
  result.setZero();
  for (int i = 0; i < result.size(); i++)
  {
    for (int j = 0; j <= i; j++)
    {
      result(i) += (j < lCoef.size() && (i - j) < rCoef.size()) ? (lCoef(j) * rCoef(i - j)) : 0;
    }
  }

  return result;
}

// // This function needs FFTW 3 and only performs better when the scale is large
// inline Eigen::VectorXd polyConvFFT(const Eigen::VectorXd &lCoef, const Eigen::VectorXd &rCoef)
// // Calculate the convolution of lCoef(x) and rCoef(x) using FFT
// // This function is fast when orders of both poly are larger than 100
// {
//     int paddedLen = lCoef.size() + rCoef.size() - 1;
//     int complexLen = paddedLen / 2 + 1;
//     Eigen::VectorXd result(paddedLen);
//     double *rBuffer = fftw_alloc_real(paddedLen);
//     // Construct FFT plan and buffers
//     fftw_complex *cForwardBuffer = fftw_alloc_complex(complexLen);
//     fftw_complex *cBackwardBuffer = fftw_alloc_complex(complexLen);
//     fftw_plan forwardPlan = fftw_plan_dft_r2c_1d(paddedLen, rBuffer, cForwardBuffer,
//                                                  FFTW_ESTIMATE | FFTW_DESTROY_INPUT);
//     fftw_plan backwardPlan = fftw_plan_dft_c2r_1d(paddedLen, cBackwardBuffer, rBuffer,
//                                                   FFTW_ESTIMATE | FFTW_DESTROY_INPUT);
//     // Pad lCoef by zeros
//     int len = lCoef.size();
//     for (int i = 0; i < len; i++)
//     {
//         rBuffer[i] = lCoef(i);
//     }
//     for (int i = len; i < paddedLen; i++)
//     {
//         rBuffer[i] = 0.0;
//     }
//     // Compute fft(pad(lCoef(x)) and back it up
//     fftw_execute(forwardPlan);
//     memcpy(cBackwardBuffer, cForwardBuffer, sizeof(fftw_complex) * complexLen);
//     // Pad rCoef by zeros
//     len = rCoef.size();
//     for (int i = 0; i < len; i++)
//     {
//         rBuffer[i] = rCoef(i);
//     }
//     for (int i = len; i < paddedLen; i++)
//     {
//         rBuffer[i] = 0.0;
//     }
//     // Compute fft(pad(rCoef(x))
//     fftw_execute(forwardPlan);
//     // Compute fft(pad(lCoef(x)).fft(pad(rCoef(x))
//     double real, imag;
//     for (int i = 0; i < complexLen; i++)
//     {
//         real = cBackwardBuffer[i][0];
//         imag = cBackwardBuffer[i][1];
//         cBackwardBuffer[i][0] = real * cForwardBuffer[i][0] -
//                                 imag * cForwardBuffer[i][1];
//         cBackwardBuffer[i][1] = imag * cForwardBuffer[i][0] +
//                                 real * cForwardBuffer[i][1];
//     }
//     // Compute ifft(fft(pad(lCoef(x)).fft(pad(rCoef(x)))
//     fftw_execute(backwardPlan);
//     // Recover the original intensity
//     double intensity = 1.0 / paddedLen;
//     for (int i = 0; i < paddedLen; i++)
//     {
//         result(i) = rBuffer[i] * intensity;
//     }
//     // Destruct FFT plan and buffers
//     fftw_destroy_plan(forwardPlan);
//     fftw_destroy_plan(backwardPlan);
//     fftw_free(rBuffer);
//     fftw_free(cForwardBuffer);
//     fftw_free(cBackwardBuffer);
//     return result;
// }

inline Eigen::VectorXd polySqr(const Eigen::VectorXd &coef)
// Calculate self-convolution of coef(x)
{
  int coefSize = coef.size();
  int resultSize = coefSize * 2 - 1;
  int lbound, rbound;
  Eigen::VectorXd result(resultSize);
  double temp;
  for (int i = 0; i < resultSize; i++)
  {
    temp = 0;
    lbound = i - coefSize + 1;
    lbound = lbound > 0 ? lbound : 0;
    rbound = coefSize < (i + 1) ? coefSize : (i + 1);
    rbound += lbound;
    if (rbound & 1) //faster than rbound % 2 == 1
    {
      rbound >>= 1; //faster than rbound /= 2
      temp += coef(rbound) * coef(rbound);
    }
    else
    {
      rbound >>= 1; //faster than rbound /= 2
    }

    for (int j = lbound; j < rbound; j++)
    {
      temp += 2.0 * coef(j) * coef(i - j);
    }
    result(i) = temp;
  }

  return result;
}

inline double polyVal(const Eigen::VectorXd &coeffs, double x,
                      bool numericalStability = true)
// Evaluate the polynomial at x, i.e., coeffs(x)
// Horner scheme is faster yet less stable
// Stable one should be used when coeffs(x) is close to 0.0
{
  double retVal = 0.0;
  int order = (int)coeffs.size() - 1;

  if (order >= 0)
  {
    if (fabs(x) < DBL_EPSILON)
    {
      retVal = coeffs(order);
    }
    else if (x == 1.0)
    {
      retVal = coeffs.sum();
    }
    else
    {
      if (numericalStability)
      {
        double xn = 1.0;

        for (int i = order; i >= 0; i--)
        {
          retVal += coeffs(i) * xn;
          xn *= x;
        }
      }
      else
      {
        int len = coeffs.size();

        for (int i = 0; i < len; i++)
        {
          retVal = retVal * x + coeffs(i);
        }
      }
    }
  }

  return retVal;
}

inline int countRoots(const Eigen::VectorXd &coeffs, double l, double r)
// Count the number of distinct roots of coeffs(x) inside (l, r), leveraging Sturm theory
// Boundary values, i.e., coeffs(l) and coeffs(r), must be nonzero
{
  int nRoots = 0;

  int originalSize = coeffs.size();
  int valid = originalSize;
  for (int i = 0; i < originalSize; i++)
  {
    if (fabs(coeffs(i)) < DBL_EPSILON)
    {
      valid--;
    }
    else
    {
      break;
    }
  }

  if (valid > 0 && fabs(coeffs(originalSize - 1)) > DBL_EPSILON)
  {
    Eigen::VectorXd monicCoeffs(valid);
    monicCoeffs << 1.0, coeffs.segment(originalSize - valid + 1, valid - 1) / coeffs(originalSize - valid);

    // Build the Sturm sequence
    int len = monicCoeffs.size();
    int order = len - 1;
    double sturmSeqs[(RootFinderParam::highestOrder + 1) * (RootFinderParam::highestOrder + 1)];
    int szSeq[RootFinderParam::highestOrder + 1] = {0}; // Explicit ini as zero (gcc may neglect this in -O3)
    int num = 0;

    for (int i = 0; i < len; i++)
    {
      sturmSeqs[i] = monicCoeffs(i);
      sturmSeqs[i + 1 + len] = (order - i) * sturmSeqs[i] / order;
    }
    szSeq[0] = len;
    szSeq[1] = len - 1;
    num += 2;

    bool remainderConstant = false;
    int idx = 0;
    while (!remainderConstant)
    {
      szSeq[idx + 2] = RootFinderPriv::polyMod(&(sturmSeqs[(idx + 1) * len - szSeq[idx]]),
                                               &(sturmSeqs[(idx + 2) * len - szSeq[idx + 1]]),
                                               &(sturmSeqs[(idx + 3) * len - szSeq[idx]]),
                                               szSeq[idx], szSeq[idx + 1]);
      remainderConstant = szSeq[idx + 2] == 1;
      for (int i = 1; i < szSeq[idx + 2]; i++)
      {
        sturmSeqs[(idx + 3) * len - szSeq[idx + 2] + i] /= -fabs(sturmSeqs[(idx + 3) * len - szSeq[idx + 2]]);
      }
      sturmSeqs[(idx + 3) * len - szSeq[idx + 2]] /= -fabs(sturmSeqs[(idx + 3) * len - szSeq[idx + 2]]);
      num++;
      idx++;
    }

    // Count numbers of sign variations at two boundaries
    double yl, lastyl, yr, lastyr;
    lastyl = RootFinderPriv::polyEval(&(sturmSeqs[len - szSeq[0]]), szSeq[0], l);
    lastyr = RootFinderPriv::polyEval(&(sturmSeqs[len - szSeq[0]]), szSeq[0], r);
    for (int i = 1; i < num; i++)
    {
      yl = RootFinderPriv::polyEval(&(sturmSeqs[(i + 1) * len - szSeq[i]]), szSeq[i], l);
      yr = RootFinderPriv::polyEval(&(sturmSeqs[(i + 1) * len - szSeq[i]]), szSeq[i], r);
      if (lastyl == 0.0 || lastyl * yl < 0.0)
      {
        ++nRoots;
      }
      if (lastyr == 0.0 || lastyr * yr < 0.0)
      {
        --nRoots;
      }
      lastyl = yl;
      lastyr = yr;
    }
  }

  return nRoots;
}

inline std::set<double> solvePolynomial(const Eigen::VectorXd &coeffs, double lbound, double ubound, double tol, bool isolation = true)
// Calculate roots of coeffs(x) inside (lbound, rbound)
//
// Closed-form solutions are employed for reduced_order < 5
// isolation = true:
//                    Sturm' theory and some geometrical property are employed to bracket each root
//                    Safe-Newton is employed to shrink the interval efficiently
// isolation = false:
//                    Eigen values of polynomial companion matrix are calculated
//
// Requirement: leading coefficient must be nonzero
//              coeffs(lbound) != 0, coeffs(rbound) != 0, lbound < rbound
{
  std::set<double> rts;

  int valid = coeffs.size();
  for (int i = 0; i < coeffs.size(); i++)
  {
    if (fabs(coeffs(i)) < DBL_EPSILON)
    {
      valid--;
    }
    else
    {
      break;
    }
  }

  int offset = 0;
  int nonzeros = valid;
  if (valid > 0)
  {
    for (int i = 0; i < valid; i++)
    {
      if (fabs(coeffs(coeffs.size() - i - 1)) < DBL_EPSILON)
      {
        nonzeros--;
        offset++;
      }
      else
      {
        break;
      }
    }
  }

  if (nonzeros == 0)
  {
    rts.insert(INFINITY);
    rts.insert(-INFINITY);
  }
  else if (nonzeros == 1 && offset == 0)
  {
    rts.clear();
  }
  else
  {
    Eigen::VectorXd ncoeffs(std::max(5, nonzeros));
    ncoeffs.setZero();
    ncoeffs.tail(nonzeros) << coeffs.segment(coeffs.size() - valid, nonzeros);

    if (nonzeros <= 5)
    {
      rts = RootFinderPriv::solveQuart(ncoeffs(0), ncoeffs(1), ncoeffs(2), ncoeffs(3), ncoeffs(4));
    }
    else
    {
      if (isolation)
      {
        rts = RootFinderPriv::isolateRealRoots(ncoeffs, lbound, ubound, tol);
      }
      else
      {
        rts = RootFinderPriv::eigenSolveRealRoots(ncoeffs, lbound, ubound, tol);
      }
    }

    if (offset > 0)
    {
      rts.insert(0.0);
    }
  }

  for (auto it = rts.begin(); it != rts.end();)
  {
    if (*it > lbound && *it < ubound)
    {
      it++;
    }
    else
    {
      it = rts.erase(it);
    }
  }

  return rts;
}

}

// Polynomial order and trajectory dimension are fixed here
constexpr int TrajOrder = 7;
constexpr int TrajDim = 3;

// Type for piece boundary condition and coefficient matrix
typedef Eigen::Matrix<double, TrajDim, TrajOrder + 1> BoundaryCond;
typedef Eigen::Matrix<double, TrajDim, TrajOrder + 1> CoefficientMat;
typedef Eigen::Matrix<double, TrajDim, TrajOrder> VelCoefficientMat;
typedef Eigen::Matrix<double, TrajDim, TrajOrder - 1> AccCoefficientMat;

// A single piece of a trajectory, which is indeed a polynomial
class Piece
{
 private:
  // A piece is totally determined by boundary condition and duration
  // boundCond = [p(0),v(0),a(0),j(0),p(T),v(T),a(T),j(T)]
  BoundaryCond boundCond;
  double duration;

  // The normalized coefficient is generated from boundCond and duration
  // These members CANNOT be accessed unless through normalizedCoeffMat()
  // p(t) = c7*t^7+c6*t^6 + ... + c1*t + c0
  // nCoeffMat = [c7*T^7,c6*T^6,c5*T^5,c4*T^4,c3*T^3,c2*T^2,c1*T,c0*1]
  bool synced;
  CoefficientMat nCoeffMat;
  inline const CoefficientMat &normalizedCoeffMat(void)
  {
    if (!synced)
    {
      double t1 = duration;
      double t2 = t1 * t1;
      double t3 = t2 * t1;

      nCoeffMat.col(0) = (boundCond.col(7) / 6.0 + boundCond.col(3) / 6.0) * t3 +
          (-2.0 * boundCond.col(6) + 2.0 * boundCond.col(2)) * t2 +
          (10.0 * boundCond.col(5) + 10.0 * boundCond.col(1)) * t1 +
          (-20.0 * boundCond.col(4) + 20.0 * boundCond.col(0));
      nCoeffMat.col(1) = (-0.5 * boundCond.col(7) - boundCond.col(3) / 1.5) * t3 +
          (6.5 * boundCond.col(6) - 7.5 * boundCond.col(2)) * t2 +
          (-34.0 * boundCond.col(5) - 36.0 * boundCond.col(1)) * t1 +
          (70.0 * boundCond.col(4) - 70.0 * boundCond.col(0));
      nCoeffMat.col(2) = (0.5 * boundCond.col(7) + boundCond.col(3)) * t3 +
          (-7.0 * boundCond.col(6) + 10.0 * boundCond.col(2)) * t2 +
          (39.0 * boundCond.col(5) + 45.0 * boundCond.col(1)) * t1 +
          (-84.0 * boundCond.col(4) + 84.0 * boundCond.col(0));
      nCoeffMat.col(3) = (-boundCond.col(7) / 6.0 - boundCond.col(3) / 1.5) * t3 +
          (2.5 * boundCond.col(6) - 5.0 * boundCond.col(2)) * t2 +
          (-15.0 * boundCond.col(5) - 20.0 * boundCond.col(1)) * t1 +
          (35.0 * boundCond.col(4) - 35.0 * boundCond.col(0));
      nCoeffMat.col(4) = boundCond.col(3) * t3 / 6.0;
      nCoeffMat.col(5) = boundCond.col(2) * t2 / 2.0;
      nCoeffMat.col(6) = boundCond.col(1) * t1;
      nCoeffMat.col(7) = boundCond.col(0);

      synced = true;
    }

    return nCoeffMat;
  }

 public:
  Piece() = default;

  // Constructor from boundary condition and duration
  Piece(BoundaryCond bdCond, double dur) : boundCond(bdCond), duration(dur), synced(false) {}

  inline int getDim() const
  {
    return TrajDim;
  }

  inline int getOrder() const
  {
    return TrajOrder;
  }

  inline double getDuration() const
  {
    return duration;
  }

  // Get the position at time t in this piece
  inline Eigen::Vector3d getPos(double t)
  {
    // Normalize the time
    t /= duration;
    Eigen::Vector3d pos(0.0, 0.0, 0.0);
    double tn = 1.0;
    for (int i = TrajOrder; i >= 0; i--)
    {
      pos += tn * normalizedCoeffMat().col(i);
      tn *= t;
    }
    // The pos is not affected by normalization
    return pos;
  }

  // Get the velocity at time t in this piece
  inline Eigen::Vector3d getVel(double t)
  {
    // Normalize the time
    t /= duration;
    Eigen::Vector3d vel(0.0, 0.0, 0.0);
    double tn = 1.0;
    int n = 1;
    for (int i = TrajOrder - 1; i >= 0; i--)
    {
      vel += n * tn * normalizedCoeffMat().col(i);
      tn *= t;
      n++;
    }
    // Recover the actual vel
    vel /= duration;
    return vel;
  }

  // Get the acceleration at time t in this piece
  inline Eigen::Vector3d getAcc(double t)
  {
    // Normalize the time
    t /= duration;
    Eigen::Vector3d acc(0.0, 0.0, 0.0);
    double tn = 1.0;
    int m = 1;
    int n = 2;
    for (int i = TrajOrder - 2; i >= 0; i--)
    {
      acc += m * n * tn * normalizedCoeffMat().col(i);
      tn *= t;
      m++;
      n++;
    }
    // Recover the actual acc
    acc /= duration * duration;
    return acc;
  }

  // Get the boundary condition of this piece
  inline const BoundaryCond &getBoundCond() const
  {
    return boundCond;
  }

  // Get the coefficient matrix of the piece
  // Default arg chooses the natural coefficients
  // If normalized version is needed, set the arg true
  inline CoefficientMat getCoeffMat(bool normalized = false)
  {
    CoefficientMat posCoeffsMat;
    double t = 1;
    for (int i = TrajOrder; i >= 0; i--)
    {
      posCoeffsMat.col(i) = normalizedCoeffMat().col(i) / t;
      t *= normalized ? 1.0 : duration;
    }
    return posCoeffsMat;
  }

  // Get the polynomial coefficients of velocity of this piece
  // Default arg chooses the natural coefficients
  // If normalized version is needed, set the arg true
  inline VelCoefficientMat getVelCoeffMat(bool normalized = false)
  {
    VelCoefficientMat velCoeffMat;
    int n = 1;
    double t = 1.0;
    t *= normalized ? 1.0 : duration;
    for (int i = TrajOrder - 1; i >= 0; i--)
    {
      velCoeffMat.col(i) = n * normalizedCoeffMat().col(i) / t;
      n++;
      t *= normalized ? 1.0 : duration;
    }
    return velCoeffMat;
  }

  // Get the polynomial coefficients of acceleration of this piece
  // Default arg chooses the natural coefficients
  // If normalized version is needed, set the arg true
  inline AccCoefficientMat getAccCoeffMat(bool normalized = false)
  {
    AccCoefficientMat accCoeffMat;
    int n = 2;
    int m = 1;
    double t = 1.0;
    t *= normalized ? 1.0 : duration * duration;
    for (int i = TrajOrder - 2; i >= 0; i--)
    {
      accCoeffMat.col(i) = n * m * normalizedCoeffMat().col(i) / t;
      n++;
      m++;
      t *= normalized ? 1.0 : duration;
    }
    return accCoeffMat;
  }

  // Get the max velocity rate of the piece
  inline double getMaxVelRate()
  {
    // Compute normalized squared vel norm polynomial coefficient matrix
    Eigen::MatrixXd nVelCoeffMat = getVelCoeffMat(true);
    Eigen::VectorXd coeff = RootFinder::polySqr(nVelCoeffMat.row(0)) +
        RootFinder::polySqr(nVelCoeffMat.row(1)) +
        RootFinder::polySqr(nVelCoeffMat.row(2));
    int N = coeff.size();
    int n = N - 1;
    for (int i = 0; i < N; i++)
    {
      coeff(i) *= n;
      n--;
    }
    if (coeff.head(N - 1).squaredNorm() < DBL_EPSILON)
    {
      return getVel(0.0).norm();
    }
    else
    {
      // Search an open interval whose boundaries are not zeros
      double l = -0.0625;
      double r = 1.0625;
      while (fabs(RootFinder::polyVal(coeff.head(N - 1), l)) < DBL_EPSILON)
      {
        l = 0.5 * l;
      }
      while (fabs(RootFinder::polyVal(coeff.head(N - 1), r)) < DBL_EPSILON)
      {
        r = 0.5 * (r + 1.0);
      }
      // Find all stationaries
      std::set<double> candidates = RootFinder::solvePolynomial(coeff.head(N - 1), l, r,
                                                                FLT_EPSILON / duration);
      // Check boundary points and stationaries within duration
      candidates.insert(0.0);
      candidates.insert(1.0);
      double maxVelRateSqr = -INFINITY;
      double tempNormSqr;
      for (std::set<double>::const_iterator it = candidates.begin();
           it != candidates.end();
           it++)
      {
        if (0.0 <= *it && 1.0 >= *it)
        {
          // Recover the actual time then get the vel squared norm
          tempNormSqr = getVel((*it) * duration).squaredNorm();
          maxVelRateSqr = maxVelRateSqr < tempNormSqr ? tempNormSqr : maxVelRateSqr;
        }
      }
      return sqrt(maxVelRateSqr);
    }
  }

  // Get the max acceleration rate of the piece
  inline double getMaxAccRate()
  {
    // Compute normalized squared acc norm polynomial coefficient matrix
    Eigen::MatrixXd nAccCoeffMat = getAccCoeffMat(true);
    Eigen::VectorXd coeff = RootFinder::polySqr(nAccCoeffMat.row(0)) +
        RootFinder::polySqr(nAccCoeffMat.row(1)) +
        RootFinder::polySqr(nAccCoeffMat.row(2));
    int N = coeff.size();
    int n = N - 1;
    for (int i = 0; i < N; i++)
    {
      coeff(i) *= n;
      n--;
    }
    if (coeff.head(N - 1).squaredNorm() < DBL_EPSILON)
    {
      return getAcc(0.0).norm();
    }
    else
    {
      // Search an open interval whose boundaries are not zeros
      double l = -0.0625;
      double r = 1.0625;
      while (fabs(RootFinder::polyVal(coeff.head(N - 1), l)) < DBL_EPSILON)
      {
        l = 0.5 * l;
      }
      while (fabs(RootFinder::polyVal(coeff.head(N - 1), r)) < DBL_EPSILON)
      {
        r = 0.5 * (r + 1.0);
      }
      // Find all stationaries
      std::set<double> candidates = RootFinder::solvePolynomial(coeff.head(N - 1), l, r,
                                                                FLT_EPSILON / duration);
      // Check boundary points and stationaries within duration
      candidates.insert(0.0);
      candidates.insert(1.0);
      double maxAccRateSqr = -INFINITY;
      double tempNormSqr;
      for (std::set<double>::const_iterator it = candidates.begin();
           it != candidates.end();
           it++)
      {
        if (0.0 <= *it && 1.0 >= *it)
        {
          // Recover the actual time then get the acc squared norm
          tempNormSqr = getAcc((*it) * duration).squaredNorm();
          maxAccRateSqr = maxAccRateSqr < tempNormSqr ? tempNormSqr : maxAccRateSqr;
        }
      }
      return sqrt(maxAccRateSqr);
    }
  }

  // Check whether velocity rate of the piece is always less than maxVelRate
  inline bool checkMaxVelRate(double maxVelRate)
  {
    double sqrMaxVelRate = maxVelRate * maxVelRate;
    if (getVel(0.0).squaredNorm() >= sqrMaxVelRate ||
        getVel(duration).squaredNorm() >= sqrMaxVelRate)
    {
      return false;
    }
    else
    {
      Eigen::MatrixXd nVelCoeffMat = getVelCoeffMat(true);
      Eigen::VectorXd coeff = RootFinder::polySqr(nVelCoeffMat.row(0)) +
          RootFinder::polySqr(nVelCoeffMat.row(1)) +
          RootFinder::polySqr(nVelCoeffMat.row(2));
      // Convert the actual squared maxVelRate to a normalized one
      double t2 = duration * duration;
      coeff.tail<1>()(0) -= sqrMaxVelRate * t2;
      // Directly check the root existence in the normalized interval
      return RootFinder::countRoots(coeff, 0.0, 1.0) == 0;
    }
  }

  // Check whether accleration rate of the piece is always less than maxAccRate
  inline bool checkMaxAccRate(double maxAccRate)
  {
    double sqrMaxAccRate = maxAccRate * maxAccRate;
    if (getAcc(0.0).squaredNorm() >= sqrMaxAccRate ||
        getAcc(duration).squaredNorm() >= sqrMaxAccRate)
    {
      return false;
    }
    else
    {
      Eigen::MatrixXd nAccCoeffMat = getAccCoeffMat(true);
      Eigen::VectorXd coeff = RootFinder::polySqr(nAccCoeffMat.row(0)) +
          RootFinder::polySqr(nAccCoeffMat.row(1)) +
          RootFinder::polySqr(nAccCoeffMat.row(2));
      // Convert the actual squared maxAccRate to a normalized one
      double t2 = duration * duration;
      double t4 = t2 * t2;
      coeff.tail<1>()(0) -= sqrMaxAccRate * t4;
      // Directly check the root existence in the normalized interval
      return RootFinder::countRoots(coeff, 0.0, 1.0) == 0;
    }
  }
};

// A whole trajectory which contains multiple pieces
class Trajectory
{
 private:
  typedef std::vector<Piece> Pieces;
  Pieces pieces;

 public:
  Trajectory() = default;

  // Constructor from boundary conditions and durations
  Trajectory(const std::vector<BoundaryCond> &bdConds,
             const std::vector<double> &durs)
  {
    int N = std::min(durs.size(), bdConds.size());
    pieces.reserve(N);
    for (int i = 0; i < N; i++)
    {
      pieces.emplace_back(bdConds[i], durs[i]);
    }
  }

  inline int getPieceNum() const
  {
    return pieces.size();
  }

  // Get durations vector of all pieces
  inline Eigen::VectorXd getDurations() const
  {
    int N = getPieceNum();
    Eigen::VectorXd durations(N);
    for (int i = 0; i < N; i++)
    {
      durations(i) = pieces[i].getDuration();
    }
    return durations;
  }

  // Get total duration of the trajectory
  inline double getTotalDuration() const
  {
    int N = getPieceNum();
    double totalDuration = 0.0;
    for (int i = 0; i < N; i++)
    {
      totalDuration += pieces[i].getDuration();
    }
    return totalDuration;
  }

  inline Eigen::MatrixXd getPositions() const
  {
    int N = getPieceNum();
    Eigen::MatrixXd positions(3, N + 1);
    for (int i = 0; i < N; i++)
    {
      positions.col(i) = pieces[i].getBoundCond().col(0);
    }
    positions.col(N) = pieces[N - 1].getBoundCond().col((TrajOrder + 1) / 2);
    return positions;
  }

  // Reload the operator[] to access the i-th piece
  inline const Piece &operator[](int i) const
  {
    return pieces[i];
  }

  inline Piece &operator[](int i)
  {
    return pieces[i];
  }

  inline void clear(void)
  {
    pieces.clear();
    return;
  }

  inline Pieces::const_iterator begin() const
  {
    return pieces.begin();
  }

  inline Pieces::const_iterator end() const
  {
    return pieces.end();
  }

  inline Pieces::iterator begin()
  {
    return pieces.begin();
  }

  inline Pieces::iterator end()
  {
    return pieces.end();
  }

  inline void reserve(const int &n)
  {
    pieces.reserve(n);
    return;
  }

  // Put another piece at the tail of this trajectory
  inline void emplace_back(const Piece &piece)
  {
    pieces.emplace_back(piece);
    return;
  }

  inline void emplace_back(const BoundaryCond &bdCond, const double &dur)
  {
    pieces.emplace_back(bdCond, dur);
    return;
  }

  // Append another Trajectory at the tail of this trajectory
  inline void append(const Trajectory &traj)
  {
    pieces.insert(pieces.end(), traj.begin(), traj.end());
    return;
  }

  // Find the piece at which the time t is located
  // The index is returned and the offset in t is removed
  inline int locatePieceIdx(double &t) const
  {
    int N = getPieceNum();
    int idx;
    double dur;
    for (idx = 0;
         idx < N &&
             t > (dur = pieces[idx].getDuration());
         idx++)
    {
      t -= dur;
    }
    if (idx == N)
    {
      idx--;
      t += pieces[idx].getDuration();
    }
    return idx;
  }

  // Get the position at time t of the trajectory
  inline Eigen::Vector3d getPos(double t)
  {
    int pieceIdx = locatePieceIdx(t);
    return pieces[pieceIdx].getPos(t);
  }

  // Get the velocity at time t of the trajectory
  inline Eigen::Vector3d getVel(double t)
  {
    int pieceIdx = locatePieceIdx(t);
    return pieces[pieceIdx].getVel(t);
  }

  // Get the acceleration at time t of the trajectory
  inline Eigen::Vector3d getAcc(double t)
  {
    int pieceIdx = locatePieceIdx(t);
    return pieces[pieceIdx].getAcc(t);
  }

  // Get the position at the juncIdx-th waypoint
  inline Eigen::Vector3d getJuncPos(int juncIdx) const
  {
    if (juncIdx != getPieceNum())
    {
      return pieces[juncIdx].getBoundCond().col(0);
    }
    else
    {
      return pieces[juncIdx - 1].getBoundCond().col((TrajOrder + 1) / 2);
    }
  }

  // Get the velocity at the juncIdx-th waypoint
  inline Eigen::Vector3d getJuncVel(int juncIdx) const
  {
    if (juncIdx != getPieceNum())
    {
      return pieces[juncIdx].getBoundCond().col(1);
    }
    else
    {
      return pieces[juncIdx - 1].getBoundCond().col((TrajOrder + 1) / 2 + 1);
    }
  }

  // Get the acceleration at the juncIdx-th waypoint
  inline Eigen::Vector3d getJuncAcc(int juncIdx) const
  {
    if (juncIdx != getPieceNum())
    {
      return pieces[juncIdx].getBoundCond().col(2);
    }
    else
    {
      return pieces[juncIdx - 1].getBoundCond().col((TrajOrder + 1) / 2 + 2);
    }
  }

  // Get the max velocity rate of the trajectory
  inline double getMaxVelRate()
  {
    int N = getPieceNum();
    double maxVelRate = -INFINITY;
    double tempNorm;
    for (int i = 0; i < N; i++)
    {
      tempNorm = pieces[i].getMaxVelRate();
      maxVelRate = maxVelRate < tempNorm ? tempNorm : maxVelRate;
    }
    return maxVelRate;
  }

  // Get the max acceleration rate of the trajectory
  inline double getMaxAccRate()
  {
    int N = getPieceNum();
    double maxAccRate = -INFINITY;
    double tempNorm;
    for (int i = 0; i < N; i++)
    {
      tempNorm = pieces[i].getMaxAccRate();
      maxAccRate = maxAccRate < tempNorm ? tempNorm : maxAccRate;
    }
    return maxAccRate;
  }

  // Check whether the velocity rate of this trajectory exceeds the threshold
  inline bool checkMaxVelRate(double maxVelRate)
  {
    int N = getPieceNum();
    bool feasible = true;
    for (int i = 0; i < N && feasible; i++)
    {
      feasible = feasible && pieces[i].checkMaxVelRate(maxVelRate);
    }
    return feasible;
  }

  // Check whether the acceleration rate of this trajectory exceeds the threshold
  inline bool checkMaxAccRate(double maxAccRate)
  {
    int N = getPieceNum();
    bool feasible = true;
    for (int i = 0; i < N && feasible; i++)
    {
      feasible = feasible && pieces[i].checkMaxAccRate(maxAccRate);
    }
    return feasible;
  }
};

// The banded system class is used for solving
// banded linear system Ax=b efficiently.
// A is an N*N band matrix with lower band width lowerBw
// and upper band width upperBw.
// Banded LU factorization has O(N) time complexity.
class BandedSystem
{
 public:
  // The size of A, as well as the lower/upper
  // banded width p/q are needed
  inline void create(const int &n, const int &p, const int &q)
  {
    // In case of re-creating before destroying
    destroy();
    N = n;
    lowerBw = p;
    upperBw = q;
    int rows = lowerBw + upperBw + 1;
    int actualSize = N * rows;
    ptrData = new double[actualSize];
    std::fill_n(ptrData, actualSize, 0.0);
    offset = new double *[rows];
    double *ptrRow = ptrData;
    for (int i = 0; i < rows; i++)
    {
      offset[i] = ptrRow;
      ptrRow += N;
    }
    return;
  }

  inline void destroy()
  {
    if (ptrData != nullptr)
    {
      delete[] ptrData;
      ptrData = nullptr;
    }
    if (offset != nullptr)
    {
      delete[] offset;
      offset = nullptr;
    }
    return;
  }

 private:
  int N;
  int lowerBw;
  int upperBw;
  // Compulsory nullptr initialization here
  double *ptrData = nullptr;
  double **offset = nullptr;

 public:
  // Reset the matrix to zero
  inline void reset(void)
  {
    std::fill_n(ptrData, N * (lowerBw + upperBw + 1), 0.0);
    return;
  }

  // The band matrix is stored as suggested in "Matrix Computation"
  inline const double &operator()(const int &i, const int &j) const
  {
    return offset[i - j + upperBw][j];
  }

  inline double &operator()(const int &i, const int &j)
  {
    return offset[i - j + upperBw][j];
  }

  // This function conducts banded LU factorization in place
  // Note that the matrix "A" MUST NOT HAVE ZERO PIVOTS !!!
  inline void factorizeLU()
  {
    int iM, jM;
    for (int k = 0; k <= N - 2; k++)
    {
      iM = std::min(k + lowerBw, N - 1);
      for (int i = k + 1; i <= iM; i++)
      {
        operator()(i, k) /= operator()(k, k);
      }
      jM = std::min(k + upperBw, N - 1);
      for (int j = k + 1; j <= jM; j++)
      {
        for (int i = k + 1; i <= iM; i++)
        {
          operator()(i, j) -= operator()(i, k) * operator()(k, j);
        }
      }
    }
    return;
  }

  // This function solves Ax=b, then stores x in b
  // The input b is required to be N*m, i.e.,
  // m vectors to be solved.
  template <typename EIGENMAT>
  inline void solve(EIGENMAT &b) const
  {
    int iM;
    for (int j = 0; j <= N - 1; j++)
    {
      iM = std::min(j + lowerBw, N - 1);
      for (int i = j + 1; i <= iM; i++)
      {
        b.row(i) -= operator()(i, j) * b.row(j);
      }
    }
    for (int j = N - 1; j >= 0; j--)
    {
      b.row(j) /= operator()(j, j);
      iM = std::max(0, j - upperBw);
      for (int i = iM; i <= j - 1; i++)
      {
        b.row(i) -= operator()(i, j) * b.row(j);
      }
    }
    return;
  }
};

class SnapOpt
{
 public:
  SnapOpt() = default;
  ~SnapOpt() { A.destroy(); }

 private:
  int N;
  Eigen::Matrix3Xd Ps;
  Eigen::Matrix3Xd VAJs;
  Eigen::VectorXd T;
  BandedSystem A;
  Eigen::MatrixX3d b;

  // Temp variables
  Eigen::VectorXd t2;
  Eigen::VectorXd t3;
  Eigen::VectorXd t4;
  Eigen::VectorXd t5;
  Eigen::VectorXd t6;
  Eigen::VectorXd t7;
  Eigen::VectorXd cv00, cv01, cv02;
  Eigen::VectorXd cv10, cv11, cv12;
  Eigen::VectorXd cv20, cv21, cv22;
  Eigen::VectorXd cv30, cv31, cv32;
  Eigen::VectorXd ca00, ca01, ca02;
  Eigen::VectorXd ca10, ca11, ca12;
  Eigen::VectorXd ca20, ca21, ca22;
  Eigen::VectorXd ca30, ca31, ca32;
  Eigen::VectorXd cj00, cj01, cj02;
  Eigen::VectorXd cj10, cj11, cj12;
  Eigen::VectorXd cj20, cj21, cj22;
  Eigen::VectorXd cj30, cj31, cj32;

 private:
  inline double evalPieceObjective(const Eigen::Array3d &iP,
                                   const Eigen::Array3d &iV,
                                   const Eigen::Array3d &iA,
                                   const Eigen::Array3d &iJ,
                                   const Eigen::Array3d &fP,
                                   const Eigen::Array3d &fV,
                                   const Eigen::Array3d &fA,
                                   const Eigen::Array3d &fJ,
                                   const double &duration) const
  {
    Eigen::VectorXd coeffsSnpObjective(7);
    coeffsSnpObjective(0) = 8.0 * (2.0 * iJ.square() + iJ * fJ + 2.0 * fJ.square()).sum();
    coeffsSnpObjective(1) = 120.0 * (iA * (2.0 * iJ + fJ) - fA * (iJ + 2.0 * fJ)).sum();
    coeffsSnpObjective(2) = 240.0 * (5.0 * iA.square() - 7.0 * iA * fA + 5.0 * fA.square() + 4.0 * iJ * iV + 3.0 * fJ * iV + 3.0 * iJ * fV + 4.0 * fJ * fV).sum();
    coeffsSnpObjective(3) = 240.0 * (45.0 * iA * iV - 39.0 * fA * iV + 39.0 * iA * fV - 45.0 * fA * fV + 7.0 * (iJ + fJ) * (iP - fP)).sum();
    coeffsSnpObjective(4) = 2880.0 * (9.0 * iV.square() + 17.0 * iV * fV + 9.0 * fV.square() + 7.0 * (iA - fA) * (iP - fP)).sum();
    coeffsSnpObjective(5) = 100800.0 * ((iV + fV) * (iP - fP)).sum();
    coeffsSnpObjective(6) = 100800.0 * (iP - fP).square().sum();

    double t2 = duration * duration;
    double t6 = t2 * t2 * t2;
    double t7 = t6 * duration;

    return RootFinder::polyVal(coeffsSnpObjective, duration) / t7;
  }

 public:
  inline void reset(const Eigen::Matrix<double, 3, 4> &headState,
                    const Eigen::Matrix<double, 3, 4> &tailState,
                    const int &pieceNum)
  {
    N = pieceNum;
    Ps.resize(3, N + 1);
    VAJs.resize(3, 3 * N + 3);
    Ps.leftCols<1>() = headState.leftCols<1>();
    Ps.rightCols<1>() = tailState.leftCols<1>();
    VAJs.leftCols<3>() = headState.rightCols<3>();
    VAJs.rightCols<3>() = tailState.rightCols<3>();
    T.resize(N);
    A.create(3 * N - 3, 5, 5);
    b.resize(3 * N - 3, 3);

    cv00.resize(N - 1);
    cv01.resize(N - 1);
    cv02.resize(N - 1);
    cv10.resize(N - 1);
    cv11.resize(N - 1);
    cv12.resize(N - 1);
    cv20.resize(N - 1);
    cv21.resize(N - 1);
    cv22.resize(N - 1);
    cv30.resize(N - 1);
    cv31.resize(N - 1);
    cv32.resize(N - 1);
    ca00.resize(N - 1);
    ca01.resize(N - 1);
    ca02.resize(N - 1);
    ca10.resize(N - 1);
    ca11.resize(N - 1);
    ca12.resize(N - 1);
    ca20.resize(N - 1);
    ca21.resize(N - 1);
    ca22.resize(N - 1);
    ca30.resize(N - 1);
    ca31.resize(N - 1);
    ca32.resize(N - 1);
    cj00.resize(N - 1);
    cj01.resize(N - 1);
    cj02.resize(N - 1);
    cj10.resize(N - 1);
    cj11.resize(N - 1);
    cj12.resize(N - 1);
    cj20.resize(N - 1);
    cj21.resize(N - 1);
    cj22.resize(N - 1);
    cj30.resize(N - 1);
    cj31.resize(N - 1);
    cj32.resize(N - 1);

    return;
  }

  inline void generate(const Eigen::Matrix3Xd &inPs,
                       const Eigen::VectorXd &ts)
  {
    T = ts;
    const Eigen::VectorXd &t1 = T;
    t2 = t1.cwiseProduct(t1);
    t3 = t2.cwiseProduct(t1);
    t4 = t2.cwiseProduct(t2);
    t5 = t4.cwiseProduct(t1);
    t6 = t3.cwiseProduct(t3);
    t7 = t6.cwiseProduct(t1);

    // Computed nonzero entries in A and b for linear system Ax=b to be solved
    for (int i = 0; i < N - 1; i++)
    {
      cv00(i) = 100800.0 / t6(i);
      cv01(i) = 100800.0 * (1.0 / t6(i + 1) - 1.0 / t6(i));
      cv02(i) = -100800.0 / t6(i + 1);
      cv10(i) = 48960.0 / t5(i);
      cv11(i) = 51840.0 * (1.0 / t5(i + 1) + 1.0 / t5(i));
      cv12(i) = 48960.0 / t5(i + 1);
      cv20(i) = 9360.0 / t4(i);
      cv21(i) = 10800.0 * (1.0 / t4(i + 1) - 1.0 / t4(i));
      cv22(i) = -9360.0 / t4(i + 1);
      cv30(i) = 720.0 / t3(i);
      cv31(i) = 960.0 * (1.0 / t3(i + 1) + 1.0 / t3(i));
      cv32(i) = 720.0 / t3(i + 1);

      ca00(i) = -20160.0 / t5(i);
      ca01(i) = 20160.0 * (1.0 / t5(i + 1) + 1.0 / t5(i));
      ca02(i) = -20160.0 / t5(i + 1);
      ca10(i) = -9360.0 / t4(i);
      ca11(i) = 10800.0 * (1.0 / t4(i + 1) - 1.0 / t4(i));
      ca12(i) = 9360.0 / t4(i + 1);
      ca20(i) = -1680.0 / t3(i);
      ca21(i) = 2400.0 * (1.0 / t3(i + 1) + 1.0 / t3(i));
      ca22(i) = -1680.0 / t3(i + 1);
      ca30(i) = -120.0 / t2(i);
      ca31(i) = 240.0 * (1.0 / t2(i + 1) - 1.0 / t2(i));
      ca32(i) = 120.0 / t2(i + 1);

      cj00(i) = 1680.0 / t4(i);
      cj01(i) = 1680.0 * (1.0 / t4(i + 1) - 1.0 / t4(i));
      cj02(i) = -1680.0 / t4(i + 1);
      cj10(i) = 720.0 / t3(i);
      cj11(i) = 960.0 * (1.0 / t3(i + 1) + 1.0 / t3(i));
      cj12(i) = 720.0 / t3(i + 1);
      cj20(i) = 120.0 / t2(i);
      cj21(i) = 240.0 * (1.0 / t2(i + 1) - 1.0 / t2(i));
      cj22(i) = -120.0 / t2(i + 1);
      cj30(i) = 8.0 / t1(i);
      cj31(i) = 32.0 * (1.0 / t1(i + 1) + 1.0 / t1(i));
      cj32(i) = 8.0 / t1(i + 1);
    }

    Ps.block(0, 1, 3, N - 1) = inPs;

    if (N == 2)
    {
      Eigen::Matrix3d A33, invA, bl;

      A33(0, 0) = cv11(0);
      A33(0, 1) = cv21(0);
      A33(0, 2) = cv31(0);
      A33(1, 0) = ca11(0);
      A33(1, 1) = ca21(0);
      A33(1, 2) = ca31(0);
      A33(2, 0) = cj11(0);
      A33(2, 1) = cj21(0);
      A33(2, 2) = cj31(0);

      invA = A33.inverse();
      bl.row(0) = (-cv00(0) * Ps.col(0) - cv01(0) * Ps.col(1) - cv02(0) * Ps.col(2) - cv10(0) * VAJs.col(0) - cv20(0) * VAJs.col(1) - cv30(0) * VAJs.col(2) - cv12(0) * VAJs.col(6) - cv22(0) * VAJs.col(7) - cv32(0) * VAJs.col(8)).transpose();
      bl.row(1) = (-ca00(0) * Ps.col(0) - ca01(0) * Ps.col(1) - ca02(0) * Ps.col(2) - ca10(0) * VAJs.col(0) - ca20(0) * VAJs.col(1) - ca30(0) * VAJs.col(2) - ca12(0) * VAJs.col(6) - ca22(0) * VAJs.col(7) - ca32(0) * VAJs.col(8)).transpose();
      bl.row(2) = (-cj00(0) * Ps.col(0) - cj01(0) * Ps.col(1) - cj02(0) * Ps.col(2) - cj10(0) * VAJs.col(0) - cj20(0) * VAJs.col(1) - cj30(0) * VAJs.col(2) - cj12(0) * VAJs.col(6) - cj22(0) * VAJs.col(7) - cj32(0) * VAJs.col(8)).transpose();

      VAJs.block(0, 3, 3, 3) = (invA * bl).transpose();
    }
    else
    {
      A.reset();
      b.setZero();

      A(0, 0) = cv11(0);
      A(0, 1) = cv21(0);
      A(0, 2) = cv31(0);
      A(0, 3) = cv12(0);
      A(0, 4) = cv22(0);
      A(0, 5) = cv32(0);
      A(1, 0) = ca11(0);
      A(1, 1) = ca21(0);
      A(1, 2) = ca31(0);
      A(1, 3) = ca12(0);
      A(1, 4) = ca22(0);
      A(1, 5) = ca32(0);
      A(2, 0) = cj11(0);
      A(2, 1) = cj21(0);
      A(2, 2) = cj31(0);
      A(2, 3) = cj12(0);
      A(2, 4) = cj22(0);
      A(2, 5) = cj32(0);
      A(3 * N - 6, 3 * N - 9) = cv10(N - 2);
      A(3 * N - 6, 3 * N - 8) = cv20(N - 2);
      A(3 * N - 6, 3 * N - 7) = cv30(N - 2);
      A(3 * N - 6, 3 * N - 6) = cv11(N - 2);
      A(3 * N - 6, 3 * N - 5) = cv21(N - 2);
      A(3 * N - 6, 3 * N - 4) = cv31(N - 2);
      A(3 * N - 5, 3 * N - 9) = ca10(N - 2);
      A(3 * N - 5, 3 * N - 8) = ca20(N - 2);
      A(3 * N - 5, 3 * N - 7) = ca30(N - 2);
      A(3 * N - 5, 3 * N - 6) = ca11(N - 2);
      A(3 * N - 5, 3 * N - 5) = ca21(N - 2);
      A(3 * N - 5, 3 * N - 4) = ca31(N - 2);
      A(3 * N - 4, 3 * N - 9) = cj10(N - 2);
      A(3 * N - 4, 3 * N - 8) = cj20(N - 2);
      A(3 * N - 4, 3 * N - 7) = cj30(N - 2);
      A(3 * N - 4, 3 * N - 6) = cj11(N - 2);
      A(3 * N - 4, 3 * N - 5) = cj21(N - 2);
      A(3 * N - 4, 3 * N - 4) = cj31(N - 2);

      b.row(0) = (-cv00(0) * Ps.col(0) - cv01(0) * Ps.col(1) - cv02(0) * Ps.col(2) - cv10(0) * VAJs.col(0) - cv20(0) * VAJs.col(1) - cv30(0) * VAJs.col(2)).transpose();
      b.row(1) = (-ca00(0) * Ps.col(0) - ca01(0) * Ps.col(1) - ca02(0) * Ps.col(2) - ca10(0) * VAJs.col(0) - ca20(0) * VAJs.col(1) - ca30(0) * VAJs.col(2)).transpose();
      b.row(2) = (-cj00(0) * Ps.col(0) - cj01(0) * Ps.col(1) - cj02(0) * Ps.col(2) - cj10(0) * VAJs.col(0) - cj20(0) * VAJs.col(1) - cj30(0) * VAJs.col(2)).transpose();
      b.row(3 * N - 6) = (-cv00(N - 2) * Ps.col(N - 2) - cv01(N - 2) * Ps.col(N - 1) - cv02(N - 2) * Ps.col(N) - cv12(N - 2) * VAJs.col(3 * N) - cv22(N - 2) * VAJs.col(3 * N + 1) - cv32(N - 2) * VAJs.col(3 * N + 2)).transpose();
      b.row(3 * N - 5) = (-ca00(N - 2) * Ps.col(N - 2) - ca01(N - 2) * Ps.col(N - 1) - ca02(N - 2) * Ps.col(N) - ca12(N - 2) * VAJs.col(3 * N) - ca22(N - 2) * VAJs.col(3 * N + 1) - ca32(N - 2) * VAJs.col(3 * N + 2)).transpose();
      b.row(3 * N - 4) = (-cj00(N - 2) * Ps.col(N - 2) - cj01(N - 2) * Ps.col(N - 1) - cj02(N - 2) * Ps.col(N) - cj12(N - 2) * VAJs.col(3 * N) - cj22(N - 2) * VAJs.col(3 * N + 1) - cj32(N - 2) * VAJs.col(3 * N + 2)).transpose();

      for (int i = 1; i < N - 2; i++)
      {
        A(i * 3, i * 3 - 3) = cv10(i);
        A(i * 3, i * 3 - 2) = cv20(i);
        A(i * 3, i * 3 - 1) = cv30(i);
        A(i * 3, i * 3) = cv11(i);
        A(i * 3, i * 3 + 1) = cv21(i);
        A(i * 3, i * 3 + 2) = cv31(i);
        A(i * 3, i * 3 + 3) = cv12(i);
        A(i * 3, i * 3 + 4) = cv22(i);
        A(i * 3, i * 3 + 5) = cv32(i);
        A(i * 3 + 1, i * 3 - 3) = ca10(i);
        A(i * 3 + 1, i * 3 - 2) = ca20(i);
        A(i * 3 + 1, i * 3 - 1) = ca30(i);
        A(i * 3 + 1, i * 3) = ca11(i);
        A(i * 3 + 1, i * 3 + 1) = ca21(i);
        A(i * 3 + 1, i * 3 + 2) = ca31(i);
        A(i * 3 + 1, i * 3 + 3) = ca12(i);
        A(i * 3 + 1, i * 3 + 4) = ca22(i);
        A(i * 3 + 1, i * 3 + 5) = ca32(i);
        A(i * 3 + 2, i * 3 - 3) = cj10(i);
        A(i * 3 + 2, i * 3 - 2) = cj20(i);
        A(i * 3 + 2, i * 3 - 1) = cj30(i);
        A(i * 3 + 2, i * 3) = cj11(i);
        A(i * 3 + 2, i * 3 + 1) = cj21(i);
        A(i * 3 + 2, i * 3 + 2) = cj31(i);
        A(i * 3 + 2, i * 3 + 3) = cj12(i);
        A(i * 3 + 2, i * 3 + 4) = cj22(i);
        A(i * 3 + 2, i * 3 + 5) = cj32(i);

        b.row(i * 3) = (-cv00(i) * Ps.col(i) - cv01(i) * Ps.col(i + 1) - cv02(i) * Ps.col(i + 2)).transpose();
        b.row(i * 3 + 1) = (-ca00(i) * Ps.col(i) - ca01(i) * Ps.col(i + 1) - ca02(i) * Ps.col(i + 2)).transpose();
        b.row(i * 3 + 2) = (-cj00(i) * Ps.col(i) - cj01(i) * Ps.col(i + 1) - cj02(i) * Ps.col(i + 2)).transpose();
      }

      // Solve Ax=b using banded LU factorization
      A.factorizeLU();
      // The solution is computed in place.
      A.solve(b);

      VAJs.block(0, 3, 3, 3 * N - 3) = b.transpose();
    }

    return;
  }

  inline double getObjective() const
  {
    double objective = 0.0;

    for (int i = 0; i < N; i++)
    {
      objective += evalPieceObjective(Ps.col(i), VAJs.col(3 * i), VAJs.col(3 * i + 1), VAJs.col(3 * i + 2),
                                      Ps.col(i + 1), VAJs.col(3 * i + 3), VAJs.col(3 * i + 4), VAJs.col(3 * i + 5),
                                      T(i));
    }

    return objective;
  }

  inline Eigen::VectorXd getGradT(void) const
  {
    Eigen::VectorXd grad(N);

    double tempT, tempT8;
    Eigen::Array3d iP, iV, iA, iJ, fP, fV, fA, fJ;
    Eigen::VectorXd coeffsGradT(7);
    for (int i = 0; i < N; i++)
    {
      // Get the information of the piece
      tempT = T(i);
      tempT8 = tempT * tempT;
      tempT8 = tempT8 * tempT8;
      tempT8 = tempT8 * tempT8;

      // Calculate the numerator of dJi(T)/dT without time regularization
      iP = Ps.col(i);
      iV = VAJs.col(3 * i);
      iA = VAJs.col(3 * i + 1);
      iJ = VAJs.col(3 * i + 2);
      fP = Ps.col(i + 1);
      fV = VAJs.col(3 * i + 3);
      fA = VAJs.col(3 * i + 4);
      fJ = VAJs.col(3 * i + 5);

      coeffsGradT(0) = -8.0 * (2.0 * iJ.square() + iJ * fJ + 2.0 * fJ.square()).sum();
      coeffsGradT(1) = -240.0 * (iA * (2.0 * iJ + fJ) - fA * (iJ + 2.0 * fJ)).sum();
      coeffsGradT(2) = -720.0 * (5.0 * iA.square() - 7.0 * iA * fA + 5.0 * fA.square() + 4.0 * iJ * iV + 3.0 * fJ * iV + 3.0 * iJ * fV + 4.0 * fJ * fV).sum();
      coeffsGradT(3) = -960.0 * (45.0 * iA * iV - 39.0 * fA * iV + 39.0 * iA * fV - 45.0 * fA * fV + 7.0 * (iJ + fJ) * (iP - fP)).sum();
      coeffsGradT(4) = -14400.0 * (9.0 * iV.square() + 17.0 * iV * fV + 9.0 * fV.square() + 7.0 * (iA - fA) * (iP - fP)).sum();
      coeffsGradT(5) = -604800.0 * ((iV + fV) * (iP - fP)).sum();
      coeffsGradT(6) = -705600.0 * (iP - fP).square().sum();

      // Calculate the gradient
      grad(i) = RootFinder::polyVal(coeffsGradT, tempT) / tempT8;
    }

    return grad;
  }

  inline Eigen::Matrix3Xd getGradInnerP(void) const
  {
    Eigen::Matrix3Xd grad(3, N - 1);

    for (int i = 1; i < N; i++)
    {
      grad.col(i - 1) = 1680.0 * (-VAJs.col(3 * i - 1) / t4(i - 1) - VAJs.col(3 * i + 2) * (1.0 / t4(i - 1) - 1.0 / t4(i)) + VAJs.col(3 * i + 5) / t4(i)) +
          20160.0 * (-VAJs.col(3 * i - 2) / t5(i - 1) + VAJs.col(3 * i + 1) * (1.0 / t5(i - 1) + 1.0 / t5(i)) - VAJs.col(3 * i + 4) / t5(i)) +
          100800.0 * (-VAJs.col(3 * i - 3) / t6(i - 1) - VAJs.col(3 * i) * (1.0 / t6(i - 1) - 1.0 / t6(i)) + VAJs.col(3 * i + 3) / t6(i)) +
          201600.0 * (-Ps.col(i - 1) / t7(i - 1) + Ps.col(i) * (1.0 / t7(i - 1) + 1.0 / t7(i)) - Ps.col(i + 1) / t7(i));
    }

    return grad;
  }

  inline void getTraj(Trajectory &traj) const
  {
    traj.clear();
    traj.reserve(N);
    BoundaryCond boundCond;
    for (int i = 0; i < N; i++)
    {
      boundCond.col(0) = Ps.col(i);
      boundCond.col(1) = VAJs.col(3 * i);
      boundCond.col(2) = VAJs.col(3 * i + 1);
      boundCond.col(3) = VAJs.col(3 * i + 2);
      boundCond.col(4) = Ps.col(i + 1);
      boundCond.col(5) = VAJs.col(3 * i + 3);
      boundCond.col(6) = VAJs.col(3 * i + 4);
      boundCond.col(7) = VAJs.col(3 * i + 5);

      traj.emplace_back(boundCond, T(i));
    }
    return;
  }
};

inline drake::trajectories::PiecewisePolynomial<double>
MakeMinSnapTrajFromWaypoints(
    const Eigen::Matrix3Xd& waypoints,
    const std::vector<double>& breaks,
    const Eigen::Vector3d& initial_velocity,
    const Eigen::Vector3d& final_velocity) {

  int N = waypoints.cols();

  Trajectory traj;
  SnapOpt opt;
  Eigen::Matrix<double, 3, 4> init_state = Eigen::Matrix<double, 3, 4>::Zero();
  Eigen::Matrix<double, 3, 4> final_state = Eigen::Matrix<double, 3, 4>::Zero();
  init_state.col(0) = waypoints.col(0);
  init_state.col(1) = initial_velocity;
  final_state.col(0) = waypoints.rightCols<1>();
  final_state.col(1) = final_velocity;
  Eigen::VectorXd durations = Eigen::VectorXd::Zero(N-1);
  for (int i = 0; i < N-1; i++) {
    durations(i) = breaks.at(i+1) - breaks.at(i);
  }

  final_state.col(2) = -2 * (waypoints.col(N-1) - waypoints.col(N-2)) /
                            (durations(N-2) * durations(N-2));
  final_state.col(3) =  3 * final_state.col(2) / durations(0);

  opt.reset(init_state, final_state, N-1);
  opt.generate(waypoints.block(0, 1, 3, N-2), durations);
  opt.getTraj(traj);

  std::vector<drake::MatrixX<drake::Polynomial<double>>> polys;
  for (auto& segment : traj) {
    Eigen::Matrix3Xd coeffs = segment.getCoeffMat().rowwise().reverse();
    auto polymat = drake::Vector<drake::Polynomial<double>, 3>();
    for (int i = 0; i < 3; i++) {
      polymat(i) = drake::Polynomiald(coeffs.row(i).transpose());
    }
    polys.emplace_back(polymat);
  }
  return {polys, breaks};
}

} // namespace min_snap

