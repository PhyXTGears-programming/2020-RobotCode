#pragma once

#include <cmath>
#include <ostream>

namespace Point {

struct Point {
    double x, y;
};

// TODO const?
inline Point operator+ (const Point &a, const Point &b) { return {a.x + b.x, a.y + b.y}; }
inline Point operator- (const Point &a, const Point &b) { return {a.x - b.x, a.y - b.y}; }

inline void operator+= (Point &lhs, const Point &rhs) { lhs.x += rhs.x; lhs.y += rhs.y; }
inline void operator-= (Point &lhs, const Point &rhs) { lhs.x -= rhs.x; lhs.y -= rhs.y; }

inline Point operator* (const Point &a, double b) { return {a.x * b, a.y * b}; }
inline Point operator* (double a, const Point &b) { return {a * b.x, a * b.y}; }
inline void operator*= (Point &lhs, double rhs) { lhs.x *= rhs; lhs.y *= rhs; }

inline Point operator/ (const Point &a, double b) { return {a.x / b, a.y / b}; }
inline Point operator/ (double a, const Point &b) { return {a / b.x, a / b.y}; }
inline void operator/= (Point &lhs, double rhs) { lhs.x /= rhs; lhs.y /= rhs; }

inline double magnitude (const Point &p) { return std::sqrt(p.x*p.x + p.y*p.y); }
inline double distance (const Point &p1, const Point &p2) { return magnitude(p2 - p1); }

inline std::ostream& operator<< (std::ostream &os, const Point &p) { os << "(" << p.x << ", " << p.y << ")"; return os; }

}
