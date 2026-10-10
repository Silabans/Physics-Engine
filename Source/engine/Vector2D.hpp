#pragma once
#include <iosfwd> // forward declaration for ostream (output stream) only (no input stream)
#include <cmath>

struct Vector2D {
    // original coordinates
    float x{0.0f};
    float y{0.0f};

    static inline float squaring(float value) { 
        return value * value;
    }

    // Math operations
    inline Vector2D operator+(const Vector2D& a) { return {x + a.x, y + a.y}; }
    inline Vector2D operator-(const Vector2D& a) { return {x - a.x, y - a.y}; }
    inline Vector2D operator*(float scalar) { return {scalar * x, scalar * y}; }
    inline Vector2D operator/(float scalar) { float inv = 1.0f / scalar; return {x * inv, y * inv}; }
    inline Vector2D& operator+=(const Vector2D& a) { x += a.x; y += a.y; return *this; } // inplace addition (returns the given argument)
    inline Vector2D& operator-=(const Vector2D& a) { x -= a.x; y -= a.y; return *this; } // inplace addition (returns the given argument)

};


inline Vector2D normalized(Vector2D a) {
    float dist = std::sqrt(a.x * a.x + a.y * a.y);
    return a / dist;
}
inline float dot(const Vector2D& a, const Vector2D& b) { return a.x * b.x + a.y * b.y; }
// perpendicular (cross product)
inline Vector2D cross(const Vector2D& a, float scalar) { return {-a.y * scalar, a.x * scalar}; }
inline Vector2D cross(float scalar, const Vector2D& a) { return {a.y * scalar, -a.x * scalar}; }
// gives the magnitude and direction of rotation (about the z-axis / out from or into the screen)
inline float cross(const Vector2D& a, const Vector2D& b) { return a.x * b.y - a.y * b.x; }

// lightweight initialisation of ostream (prevents the overhead of full-blown iostream)
inline std::ostream& operator<<(std::ostream& os, const Vector2D& vec) {
    return os << '(' << vec.x << ", " << vec.y << ')'; 
};