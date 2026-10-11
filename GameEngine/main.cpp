#include "raylib.h"

#define RAYGUI_IMPLEMENTATION
#include "raygui.h"

#include <vector>
#include <cmath>
#include <cstdlib>
#include <algorithm>
#include <limits>
#include <cstring>
#include <cctype>
#include <cstdio>

// ---------------------------------------------------------
// Minimal 2D vector math
// ---------------------------------------------------------
struct Vec2 {
    float x = 0.0f, y = 0.0f;

    Vec2 operator+(const Vec2& o) const { return { x + o.x, y + o.y }; }
    Vec2 operator-(const Vec2& o) const { return { x - o.x, y - o.y }; }
    Vec2 operator*(float s) const { return { x * s, y * s }; }
    Vec2& operator+=(const Vec2& o) { x += o.x; y += o.y; return *this; }
    Vec2& operator-=(const Vec2& o) { x -= o.x; y -= o.y; return *this; }

    float dot(const Vec2& o) const { return x * o.x + y * o.y; }
    float length() const { return std::sqrt(x * x + y * y); }

    Vec2 normalized() const {
        float len = length();
        if (len < 1e-6f) return { 0, 0 };
        return { x / len, y / len };
    }
};

// ---------------------------------------------------------
// Shapes
// ---------------------------------------------------------
enum class ShapeType { CIRCLE, POLYGON };

// Regular N-sided polygon (triangle, square, pentagon, hexagon, ...),
// vertices in local space centered at the origin.
std::vector<Vec2> makeRegularPolygon(int sides, float circumradius) {
    std::vector<Vec2> verts;
    verts.reserve(sides);
    for (int i = 0; i < sides; ++i) {
        float angle = -PI / 2.0f + (2.0f * PI * i) / sides; // start pointing up
        verts.push_back({ circumradius * cosf(angle), circumradius * sinf(angle) });
    }
    return verts;
}

// Rectangle with independent width/height, centered at the origin.
std::vector<Vec2> makeRectangle(float width, float height) {
    float hw = width * 0.5f, hh = height * 0.5f;
    return { {-hw, -hh}, {hw, -hh}, {hw, hh}, {-hw, hh} };
}

// ---------------------------------------------------------
// A rigid body: either a circle or a convex polygon
// ---------------------------------------------------------
struct Body {
    ShapeType shape;
    Vec2 position;
    Vec2 velocity;
    float rotation = 0.0f;            // fixed at spawn for visual variety; not simulated (no torque)
    std::vector<Vec2> localVertices;  // empty for circles, convex polygon verts otherwise
    float radius;                     // circle radius, OR bounding radius for polygons (spawn/UI use)
    float mass;
    float invMass;                    // 0 = infinite mass (static/immovable)
    Color color;
    float gravityScale = 1.0f;        // multiplier on global gravity (set per shape in the sandbox editor)

    // Circle constructor
    Body(Vec2 pos, float r, float m, Color c)
        : shape(ShapeType::CIRCLE), position(pos), velocity{ 0, 0 },
          radius(r), mass(m), color(c) {
        invMass = (m <= 0.0f) ? 0.0f : 1.0f / m;
    }

    // Polygon constructor
    Body(Vec2 pos, std::vector<Vec2> verts, float boundingRadius, float rot, float m, Color c)
        : shape(ShapeType::POLYGON), position(pos), velocity{ 0, 0 }, rotation(rot),
          localVertices(std::move(verts)), radius(boundingRadius), mass(m), color(c) {
        invMass = (m <= 0.0f) ? 0.0f : 1.0f / m;
    }

    // World-space vertex positions (rotated by `rotation`, then translated to `position`)
    std::vector<Vec2> worldVertices() const {
        std::vector<Vec2> out;
        out.reserve(localVertices.size());
        float c = cosf(rotation), s = sinf(rotation);
        for (const auto& v : localVertices) {
            Vec2 rotated{ v.x * c - v.y * s, v.x * s + v.y * c };
            out.push_back(rotated + position);
        }
        return out;
    }

    // Uniformly resize the body by `f` (mass scales with area)
    void scaleBy(float f) {
        radius *= f;
        for (auto& v : localVertices) v = v * f;
        mass *= f * f;
        invMass = (mass <= 0.0f) ? 0.0f : 1.0f / mass;
    }
};

// ---------------------------------------------------------
// Simulation parameters (tweakable live via raygui)
// ---------------------------------------------------------
struct SimParams {
    float gravity = 500.0f;      // pixels/sec^2
    float restitution = 0.8f;    // 0 = no bounce, 1 = perfectly elastic
    bool paused = false;
};

// ---------------------------------------------------------
// Collision manifold: result of a narrow-phase test.
// normal always points from body A toward body B.
// ---------------------------------------------------------
struct Manifold {
    bool colliding = false;
    Vec2 normal;
    float penetration = 0.0f;
};

// Outward-facing unit normal for each edge of a world-space convex polygon.
// Orientation-agnostic: we verify "outward" against the centroid rather
// than assuming vertex winding order.
std::vector<Vec2> polygonWorldNormals(const std::vector<Vec2>& worldVerts) {
    size_t n = worldVerts.size();
    Vec2 centroid{ 0, 0 };
    for (const auto& v : worldVerts) centroid += v;
    centroid = centroid * (1.0f / n);

    std::vector<Vec2> normals;
    normals.reserve(n);
    for (size_t i = 0; i < n; ++i) {
        Vec2 edge = worldVerts[(i + 1) % n] - worldVerts[i];
        Vec2 normal = Vec2{ edge.y, -edge.x }.normalized();
        Vec2 midpoint = (worldVerts[i] + worldVerts[(i + 1) % n]) * 0.5f;
        if (normal.dot(midpoint - centroid) < 0.0f) normal = normal * -1.0f;
        normals.push_back(normal);
    }
    return normals;
}

// ---------------------------------------------------------
// Narrow-phase: circle vs circle
// ---------------------------------------------------------
Manifold collideCircleCircle(const Body& a, const Body& b) {
    Manifold m;
    Vec2 delta = b.position - a.position;
    float dist = delta.length();
    float minDist = a.radius + b.radius;
    if (dist >= minDist || dist < 1e-6f) return m;

    m.colliding = true;
    m.normal = delta.normalized();
    m.penetration = minDist - dist;
    return m;
}

// ---------------------------------------------------------
// Narrow-phase: polygon vs polygon (SAT)
// ---------------------------------------------------------
Manifold collidePolygonPolygon(const Body& a, const Body& b) {
    Manifold m;
    std::vector<Vec2> vertsA = a.worldVertices();
    std::vector<Vec2> vertsB = b.worldVertices();
    std::vector<Vec2> normalsA = polygonWorldNormals(vertsA);
    std::vector<Vec2> normalsB = polygonWorldNormals(vertsB);

    float minOverlap = std::numeric_limits<float>::max();
    Vec2 smallestAxis{ 0, 0 };

    auto testAxes = [&](const std::vector<Vec2>& normals) -> bool {
        for (const auto& axis : normals) {
            float minA = std::numeric_limits<float>::max(), maxA = -std::numeric_limits<float>::max();
            for (const auto& v : vertsA) { float p = v.dot(axis); minA = std::min(minA, p); maxA = std::max(maxA, p); }
            float minB = std::numeric_limits<float>::max(), maxB = -std::numeric_limits<float>::max();
            for (const auto& v : vertsB) { float p = v.dot(axis); minB = std::min(minB, p); maxB = std::max(maxB, p); }

            float overlap = std::min(maxA, maxB) - std::max(minA, minB);
            if (overlap <= 0.0f) return false; // separating axis found -> no collision
            if (overlap < minOverlap) { minOverlap = overlap; smallestAxis = axis; }
        }
        return true;
    };

    if (!testAxes(normalsA)) return m;
    if (!testAxes(normalsB)) return m;

    // Make sure the chosen axis points from A toward B
    Vec2 centerDelta = b.position - a.position;
    if (smallestAxis.dot(centerDelta) < 0.0f) smallestAxis = smallestAxis * -1.0f;

    m.colliding = true;
    m.normal = smallestAxis;
    m.penetration = minOverlap;
    return m;
}

// ---------------------------------------------------------
// Narrow-phase: circle vs polygon.
// `poly` is treated as body A, `circle` as body B: the returned
// normal points from the polygon toward the circle.
// ---------------------------------------------------------
Manifold collideCirclePolygon(const Body& poly, const Body& circle) {
    Manifold m;
    std::vector<Vec2> verts = poly.worldVertices();
    std::vector<Vec2> normals = polygonWorldNormals(verts);
    size_t n = verts.size();

    float bestSeparation = -std::numeric_limits<float>::max();
    size_t bestEdge = 0;
    for (size_t i = 0; i < n; ++i) {
        float separation = normals[i].dot(circle.position - verts[i]);
        if (separation > circle.radius) return m; // separating axis found
        if (separation > bestSeparation) { bestSeparation = separation; bestEdge = i; }
    }

    Vec2 v1 = verts[bestEdge];
    Vec2 v2 = verts[(bestEdge + 1) % n];

    if (bestSeparation < 1e-6f) {
        // Circle's center is inside the polygon
        m.colliding = true;
        m.normal = normals[bestEdge];
        m.penetration = circle.radius - bestSeparation;
        return m;
    }

    // Circle center is outside: figure out if it's closest to v1, v2, or the edge face
    float d1 = (circle.position - v1).dot(v2 - v1);
    float d2 = (circle.position - v2).dot(v1 - v2);

    if (d1 <= 0.0f) {
        float dist = (circle.position - v1).length();
        if (dist > circle.radius) return m;
        m.colliding = true;
        m.normal = (circle.position - v1).normalized();
        m.penetration = circle.radius - dist;
    } else if (d2 <= 0.0f) {
        float dist = (circle.position - v2).length();
        if (dist > circle.radius) return m;
        m.colliding = true;
        m.normal = (circle.position - v2).normalized();
        m.penetration = circle.radius - dist;
    } else {
        float dist = bestSeparation;
        if (dist > circle.radius) return m;
        m.colliding = true;
        m.normal = normals[bestEdge];
        m.penetration = circle.radius - dist;
    }
    return m;
}

// Dispatches to the right narrow-phase test based on each body's shape.
// Always returns a manifold whose normal points from `a` toward `b`.
Manifold collide(const Body& a, const Body& b) {
    if (a.shape == ShapeType::CIRCLE && b.shape == ShapeType::CIRCLE) {
        return collideCircleCircle(a, b);
    }
    if (a.shape == ShapeType::POLYGON && b.shape == ShapeType::POLYGON) {
        return collidePolygonPolygon(a, b);
    }
    if (a.shape == ShapeType::POLYGON && b.shape == ShapeType::CIRCLE) {
        return collideCirclePolygon(a, b); // already poly(a) -> circle(b)
    }
    // a = CIRCLE, b = POLYGON
    Manifold m = collideCirclePolygon(b, a); // poly(b) -> circle(a)
    m.normal = m.normal * -1.0f;             // flip to a -> b
    return m;
}

// ---------------------------------------------------------
// Step 1: Integrate forces (gravity) -> velocity -> position
// Each body can scale gravity with gravityScale (used by the sandbox editor).
// ---------------------------------------------------------
void integrate(std::vector<Body>& bodies, const SimParams& params, float dt) {
    for (auto& b : bodies) {
        if (b.invMass == 0.0f) continue; // static body, skip
        b.velocity.y += params.gravity * b.gravityScale * dt;
        b.position += b.velocity * dt;
    }
}

// ---------------------------------------------------------
// Step 2: Resolve collisions against a rectangular play area.
// `bottom` is the floor (the ground), `top` the ceiling, etc.
// ---------------------------------------------------------
struct Bounds {
    float left, top, right, bottom;
};

void resolveWallCollisions(std::vector<Body>& bodies, const Bounds& bd, const SimParams& params) {
    for (auto& b : bodies) {
        // Below this speed a floor hit just stops the body, which prevents
        // endless micro-bouncing while something is resting on the ground.
        float restSpeed = params.gravity * b.gravityScale * (2.0f / 60.0f);

        // Axis-aligned extents of the body (exact for circles, bounding box for polygons)
        float minX, maxX, minY, maxY;
        if (b.shape == ShapeType::CIRCLE) {
            minX = b.position.x - b.radius; maxX = b.position.x + b.radius;
            minY = b.position.y - b.radius; maxY = b.position.y + b.radius;
        } else {
            std::vector<Vec2> verts = b.worldVertices();
            minX = minY = std::numeric_limits<float>::max();
            maxX = maxY = -std::numeric_limits<float>::max();
            for (const auto& v : verts) {
                minX = std::min(minX, v.x); maxX = std::max(maxX, v.x);
                minY = std::min(minY, v.y); maxY = std::max(maxY, v.y);
            }
        }

        // Left / right
        if (minX < bd.left) {
            b.position.x += bd.left - minX;
            if (b.velocity.x < 0.0f) b.velocity.x = -b.velocity.x * params.restitution;
        } else if (maxX > bd.right) {
            b.position.x -= maxX - bd.right;
            if (b.velocity.x > 0.0f) b.velocity.x = -b.velocity.x * params.restitution;
        }

        // Ceiling / floor
        if (minY < bd.top) {
            b.position.y += bd.top - minY;
            if (b.velocity.y < 0.0f) b.velocity.y = -b.velocity.y * params.restitution;
        } else if (maxY > bd.bottom) {
            b.position.y -= maxY - bd.bottom;
            if (b.velocity.y > 0.0f) {
                b.velocity.y = (b.velocity.y < restSpeed) ? 0.0f : -b.velocity.y * params.restitution;
            }
        }
    }
}

// ---------------------------------------------------------
// Step 3: Narrow-phase + resolution for every body pair.
// (Naive O(n^2) check -- fine for a few hundred bodies;
//  swap in a spatial grid later if you need more.)
// ---------------------------------------------------------
void resolveCollisions(std::vector<Body>& bodies, const SimParams& params) {
    for (size_t i = 0; i < bodies.size(); ++i) {
        for (size_t j = i + 1; j < bodies.size(); ++j) {
            Body& a = bodies[i];
            Body& b = bodies[j];

            Manifold m = collide(a, b);
            if (!m.colliding) continue;

            float totalInvMass = a.invMass + b.invMass;
            if (totalInvMass <= 0.0f) continue; // both static

            // --- Positional correction (stop sinking into each other) ---
            const float percent = 0.8f;
            Vec2 correction = m.normal * (m.penetration / totalInvMass * percent);
            a.position -= correction * a.invMass;
            b.position += correction * b.invMass;

            // --- Impulse resolution ---
            Vec2 relativeVelocity = b.velocity - a.velocity;
            float velAlongNormal = relativeVelocity.dot(m.normal);
            if (velAlongNormal > 0) continue; // already separating

            float impulseMag = -(1.0f + params.restitution) * velAlongNormal / totalInvMass;
            Vec2 impulse = m.normal * impulseMag;

            a.velocity -= impulse * a.invMass;
            b.velocity += impulse * b.invMass;
        }
    }
}

// =========================================================
// Drawing, picking and shape-building helpers
// =========================================================

// Draws a body. `selected` adds a highlight outline, `ghost` makes it translucent
// (used for the drag-and-drop preview).
void drawBody(const Body& b, bool selected = false, bool ghost = false) {
    Color fill = ghost ? Fade(b.color, 0.55f) : b.color;
    Color line = ghost ? Fade(DARKGRAY, 0.55f) : DARKGRAY;

    if (b.shape == ShapeType::CIRCLE) {
        Vector2 c = { b.position.x, b.position.y };
        if (selected) DrawRing(c, b.radius, b.radius + 4.0f, 0.0f, 360.0f, 48, BLUE);
        DrawCircleV(c, b.radius, fill);
        DrawCircleLines((int)b.position.x, (int)b.position.y, b.radius, line);
        return;
    }

    std::vector<Vec2> verts = b.worldVertices();
    size_t n = verts.size();
    Vector2 center = { b.position.x, b.position.y };

    // Fill: one triangle per edge, fanned out from the center. raylib culls
    // triangles by winding order, so each triangle is drawn in both orders -
    // that way the fill shows up no matter which way the vertices were listed.
    for (size_t k = 0; k < n; ++k) {
        Vector2 p1 = { verts[k].x, verts[k].y };
        Vector2 p2 = { verts[(k + 1) % n].x, verts[(k + 1) % n].y };
        DrawTriangle(center, p1, p2, fill);
        DrawTriangle(center, p2, p1, fill);
    }

    if (selected) {
        for (size_t k = 0; k < n; ++k) {
            Vec2 p1 = verts[k], p2 = verts[(k + 1) % n];
            DrawLineEx({ p1.x, p1.y }, { p2.x, p2.y }, 6.0f, BLUE);
        }
    }
    for (size_t k = 0; k < n; ++k) {
        Vec2 p1 = verts[k], p2 = verts[(k + 1) % n];
        DrawLineEx({ p1.x, p1.y }, { p2.x, p2.y }, 2.0f, line);
    }
}

// Is point p inside the body? (used for mouse picking)
bool containsPoint(const Body& b, Vec2 p) {
    if (b.shape == ShapeType::CIRCLE) return (p - b.position).length() <= b.radius;

    std::vector<Vec2> verts = b.worldVertices();
    std::vector<Vec2> normals = polygonWorldNormals(verts);
    for (size_t i = 0; i < verts.size(); ++i) {
        if (normals[i].dot(p - verts[i]) > 0.0f) return false; // outside this edge
    }
    return true;
}

// shapeSel: 0=Circle, 1=Triangle, 2=Square, 3=Rectangle, 4=Polygon(sides)
// Polygons are rotated so they sit flat on their bottom edge.
Body makeShape(int shapeSel, Vec2 pos, float size, int sides, float rot, Color c) {
    switch (shapeSel) {
    case 0:
        return Body(pos, size, size * size * 0.05f, c);
    case 1:
        return Body(pos, makeRegularPolygon(3, size), size, rot, size * size * 0.05f, c);
    case 2:
        return Body(pos, makeRegularPolygon(4, size), size, rot + PI / 4.0f, size * size * 0.08f, c);
    case 3: {
        float w = size * 1.8f, h = size * 0.9f;
        float bounding = std::sqrt((w * 0.5f) * (w * 0.5f) + (h * 0.5f) * (h * 0.5f));
        return Body(pos, makeRectangle(w, h), bounding, rot, w * h * 0.002f, c);
    }
    default: {
        float extra = (sides % 2 == 0) ? PI / sides : 0.0f; // even N: flat edge on the bottom
        return Body(pos, makeRegularPolygon(sides, size), size, rot + extra, size * size * 0.05f, c);
    }
    }
}

Color randomBrightColor() {
    return Color{ (unsigned char)GetRandomValue(80, 255),
                  (unsigned char)GetRandomValue(80, 255),
                  (unsigned char)GetRandomValue(80, 255), 255 };
}

// Accepts "#RRGGBB" or "RRGGBB". Returns false if the text isn't a complete hex color yet.
bool parseHexColor(const char* s, Color& out) {
    if (*s == '#') ++s;
    if (std::strlen(s) != 6) return false;
    for (int i = 0; i < 6; ++i) {
        if (!std::isxdigit((unsigned char)s[i])) return false;
    }
    unsigned long v = std::strtoul(s, nullptr, 16);
    out = Color{ (unsigned char)((v >> 16) & 0xFF), (unsigned char)((v >> 8) & 0xFF),
                 (unsigned char)(v & 0xFF), 255 };
    return true;
}

const char* shapeName(const Body& b) {
    if (b.shape == ShapeType::CIRCLE) return "Circle";
    return TextFormat("Polygon (%d sides)", (int)b.localVertices.size());
}

// =========================================================
// App state
// =========================================================
constexpr int   SCREEN_W = 1200;
constexpr int   SCREEN_H = 750;
constexpr float PIXELS_PER_METER = 50.0f;
constexpr float EARTH_G = 9.8f;          // m/s^2
constexpr float FIXED_DT = 1.0f / 60.0f; // physics timestep
constexpr int   TOP_H = 50;              // editor: top bar height
constexpr int   PAL_W = 160;             // editor: palette width (left)
constexpr int   INS_W = 270;             // editor: inspector width (right)
constexpr int   PALETTE_COUNT = 5;

enum class AppMode { SPAWNER, EDITOR };

struct SpawnerState {
    std::vector<Body> bodies;
    SimParams params;
    int shapeSelection = 0;       // 0=Circle, 1=Triangle, 2=Square, 3=Rectangle, 4=Polygon
    float polygonSidesF = 5.0f;
    float sizeSlider = 20.0f;
    float rectWidthSlider = 60.0f;
    float rectHeightSlider = 30.0f;
    Color spawnColor = RED;
    bool randomColor = true;
};

struct Editor {
    std::vector<Body> bodies;
    SimParams params;             // gravity is fixed at 1g; each body scales it individually
    bool paused = false;

    int selected = -1;            // index of the selected body, -1 = none
    int dragging = -1;            // index of the body being dragged, -1 = none
    Vec2 dragOffset{ 0, 0 };
    Vec2 dragVelocity{ 0, 0 };    // smoothed mouse velocity, applied when you let go (throw)

    int paletteDrag = -1;         // palette item being dragged into the world, -1 = none
    Color paletteColor = SKYBLUE;
    float paletteSides = 5.0f;

    float groundY = 640.0f;
    bool draggingGround = false;

    char hexBuf[16] = "#FFFFFF";
    bool hexEdit = false;
    int hexOwner = -2;            // which selection hexBuf currently belongs to
};

Rectangle paletteCell(int i) {
    return { 20.0f, (float)(TOP_H + 55 + i * 85), 120.0f, 75.0f };
}

// =========================================================
// Spawner mode (click-to-spawn playground)
// =========================================================
void spawnerFrame(SpawnerState& s, float& accumulator, AppMode& mode) {
    const float PANEL_HEIGHT = 175.0f;

    // Click to spawn the selected shape
    if (IsMouseButtonPressed(MOUSE_BUTTON_LEFT) && GetMouseY() > PANEL_HEIGHT) {
        Vec2 spawnPos{ (float)GetMouseX(), (float)GetMouseY() };
        Color c = s.randomColor ? randomBrightColor() : s.spawnColor;
        float randRot = (float)GetRandomValue(0, 628) / 100.0f; // cosmetic spawn rotation, 0-2pi

        if (s.shapeSelection == 0) {
            float r = s.sizeSlider;
            s.bodies.emplace_back(spawnPos, r, r * r * 0.05f, c);
        } else if (s.shapeSelection == 1) {
            float r = s.sizeSlider;
            s.bodies.emplace_back(spawnPos, makeRegularPolygon(3, r), r, randRot, r * r * 0.05f, c);
        } else if (s.shapeSelection == 2) {
            float r = s.sizeSlider;
            s.bodies.emplace_back(spawnPos, makeRegularPolygon(4, r), r, randRot, r * r * 0.08f, c);
        } else if (s.shapeSelection == 3) {
            float w = s.rectWidthSlider, h = s.rectHeightSlider;
            float bounding = std::sqrt((w * 0.5f) * (w * 0.5f) + (h * 0.5f) * (h * 0.5f));
            s.bodies.emplace_back(spawnPos, makeRectangle(w, h), bounding, randRot, w * h * 0.002f, c);
        } else {
            float r = s.sizeSlider;
            s.bodies.emplace_back(spawnPos, makeRegularPolygon((int)s.polygonSidesF, r), r, randRot, r * r * 0.05f, c);
        }
    }

    // Fixed-timestep physics
    if (!s.params.paused) {
        accumulator = std::min(accumulator + GetFrameTime(), 0.25f);
        Bounds bounds{ 0.0f, 0.0f, (float)SCREEN_W, (float)SCREEN_H };
        while (accumulator >= FIXED_DT) {
            integrate(s.bodies, s.params, FIXED_DT);
            resolveCollisions(s.bodies, s.params);
            resolveWallCollisions(s.bodies, bounds, s.params);
            accumulator -= FIXED_DT;
        }
    }

    // Render
    BeginDrawing();
    ClearBackground(RAYWHITE);

    for (const auto& b : s.bodies) drawBody(b);

    DrawRectangle(0, 0, SCREEN_W, (int)PANEL_HEIGHT, Fade(LIGHTGRAY, 0.9f));

    GuiSlider({ 120, 15, 200, 20 }, "Gravity", TextFormat("%.2f m/s^2", s.params.gravity / PIXELS_PER_METER),
              &s.params.gravity, 0.0f, 2000.0f);
    GuiCheckBox({ 400, 15, 20, 20 }, "Paused", &s.params.paused);

    GuiSlider({ 120, 45, 200, 20 }, "Bounciness", TextFormat("%.2f", s.params.restitution),
              &s.params.restitution, 0.0f, 1.0f);
    if (GuiButton({ 350, 45, 100, 25 }, "Clear all")) s.bodies.clear();

    DrawText("Shape:", 20, 85, 16, DARKGRAY);
    GuiToggleGroup({ 80, 80, 70, 25 }, "Circle;Triangle;Square;Rectangle;Polygon", &s.shapeSelection);
    if (s.shapeSelection == 4) {
        GuiSlider({ 440, 80, 100, 25 }, "Sides", TextFormat("%d", (int)s.polygonSidesF),
                  &s.polygonSidesF, 3.0f, 12.0f);
    }

    if (s.shapeSelection == 3) {
        GuiSlider({ 120, 115, 140, 20 }, "Width", TextFormat("%.0f", s.rectWidthSlider),
                  &s.rectWidthSlider, 20.0f, 150.0f);
        GuiSlider({ 300, 115, 140, 20 }, "Height", TextFormat("%.0f", s.rectHeightSlider),
                  &s.rectHeightSlider, 10.0f, 100.0f);
    } else {
        GuiSlider({ 120, 115, 200, 20 }, "Size", TextFormat("%.0f", s.sizeSlider),
                  &s.sizeSlider, 8.0f, 60.0f);
    }

    DrawText(TextFormat("Bodies: %d", (int)s.bodies.size()), 20, 145, 18, DARKGRAY);
    DrawText("Click below to spawn", 160, 147, 16, GRAY);

    GuiColorPicker({ 600, 15, 100, 100 }, "Spawn Color", &s.spawnColor);
    GuiCheckBox({ 600, 125, 20, 20 }, "Random Color", &s.randomColor);

    if (GuiButton({ 980, 15, 190, 30 }, "Open Sandbox Editor")) {
        mode = AppMode::EDITOR;
        accumulator = 0.0f;
        SetMouseCursor(MOUSE_CURSOR_DEFAULT);
    }

    EndDrawing();
}

// =========================================================
// Sandbox editor mode (drag & drop, per-shape settings, movable ground)
// =========================================================
void editorFrame(Editor& ed, float& accumulator, AppMode& mode) {
    Vector2 mouse = GetMousePosition();
    Vec2 m{ mouse.x, mouse.y };

    const int worldW = SCREEN_W - PAL_W - INS_W;
    const int insX = SCREEN_W - INS_W;
    Rectangle worldRect = { (float)PAL_W, (float)TOP_H, (float)worldW, (float)(SCREEN_H - TOP_H) };
    bool mouseInWorld = CheckCollisionPointRec(mouse, worldRect);
    Bounds bounds{ (float)PAL_W, (float)TOP_H, (float)insX, ed.groundY };

    if (ed.selected >= (int)ed.bodies.size()) ed.selected = -1;
    if (ed.dragging >= (int)ed.bodies.size()) ed.dragging = -1;

    // ---- What is the mouse over? ----
    int hoverBody = -1;
    if (mouseInWorld && ed.paletteDrag < 0) {
        for (int i = (int)ed.bodies.size() - 1; i >= 0; --i) { // topmost first
            if (containsPoint(ed.bodies[i], m)) { hoverBody = i; break; }
        }
    }
    bool nearGround = mouseInWorld && hoverBody < 0 && std::fabs(m.y - ed.groundY) <= 10.0f;

    int cursor = MOUSE_CURSOR_DEFAULT;
    if (ed.draggingGround || nearGround) cursor = MOUSE_CURSOR_RESIZE_NS;
    else if (ed.dragging >= 0) cursor = MOUSE_CURSOR_RESIZE_ALL;
    else if (hoverBody >= 0) cursor = MOUSE_CURSOR_POINTING_HAND;
    SetMouseCursor(cursor);

    // ---- Mouse press: start a palette drag, pick a body, or grab the ground ----
    if (IsMouseButtonPressed(MOUSE_BUTTON_LEFT)) {
        for (int i = 0; i < PALETTE_COUNT; ++i) {
            if (CheckCollisionPointRec(mouse, paletteCell(i))) {
                ed.paletteDrag = i;
                ed.paletteColor = randomBrightColor();
            }
        }
        if (mouseInWorld) {
            if (hoverBody >= 0) {
                ed.selected = hoverBody;
                ed.dragging = hoverBody;
                ed.dragOffset = ed.bodies[hoverBody].position - m;
                ed.dragVelocity = Vec2{ 0, 0 };
            } else if (nearGround) {
                ed.draggingGround = true;
            } else {
                ed.selected = -1;
            }
        }
    }

    // ---- Palette drop: create the shape where the mouse was released ----
    if (ed.paletteDrag >= 0 && !IsMouseButtonDown(MOUSE_BUTTON_LEFT)) {
        if (mouseInWorld) {
            ed.bodies.push_back(makeShape(ed.paletteDrag, m, 28.0f, (int)ed.paletteSides, 0.0f, ed.paletteColor));
            ed.selected = (int)ed.bodies.size() - 1;
        }
        ed.paletteDrag = -1;
    }

    // ---- Dragging a body (follows the mouse; lets go with a throw) ----
    if (ed.dragging >= 0) {
        Body& d = ed.bodies[ed.dragging];
        if (IsMouseButtonDown(MOUSE_BUTTON_LEFT)) {
            Vec2 target = m + ed.dragOffset;
            float dt = GetFrameTime();
            if (dt > 0.0f) ed.dragVelocity = ed.dragVelocity * 0.6f + ((target - d.position) * (1.0f / dt)) * 0.4f;
            d.position = target;
            d.velocity = Vec2{ 0, 0 };
        } else {
            float speed = ed.dragVelocity.length();
            if (speed > 1500.0f) ed.dragVelocity = ed.dragVelocity * (1500.0f / speed);
            d.velocity = ed.dragVelocity;
            ed.dragging = -1;
        }
    }

    // ---- Dragging the ground line ----
    if (ed.draggingGround) {
        if (IsMouseButtonDown(MOUSE_BUTTON_LEFT)) {
            ed.groundY = std::clamp(m.y, (float)(TOP_H + 100), (float)(SCREEN_H - 10));
            bounds.bottom = ed.groundY;
        } else {
            ed.draggingGround = false;
        }
    }

    // ---- Delete key (not while typing in the hex box) ----
    bool deleteSelected = false;
    if (IsKeyPressed(KEY_DELETE) && !ed.hexEdit && ed.selected >= 0) deleteSelected = true;

    // ---- Physics ----
    // A body being dragged is made temporarily immovable so it pushes others out of the way.
    float savedInvMass = 0.0f;
    bool pinned = ed.dragging >= 0;
    if (pinned) {
        savedInvMass = ed.bodies[ed.dragging].invMass;
        ed.bodies[ed.dragging].invMass = 0.0f;
    }
    if (!ed.paused) {
        accumulator = std::min(accumulator + GetFrameTime(), 0.25f);
        while (accumulator >= FIXED_DT) {
            integrate(ed.bodies, ed.params, FIXED_DT);
            resolveCollisions(ed.bodies, ed.params);
            resolveWallCollisions(ed.bodies, bounds, ed.params);
            accumulator -= FIXED_DT;
        }
    } else {
        accumulator = 0.0f;
        resolveWallCollisions(ed.bodies, bounds, ed.params); // still keep things inside the area
    }
    if (pinned) ed.bodies[ed.dragging].invMass = savedInvMass;

    // The hex box must restart whenever the selection changes
    if (ed.selected != ed.hexOwner) {
        ed.hexOwner = ed.selected;
        ed.hexEdit = false;
    }

    // =====================================================
    // Render
    // =====================================================
    BeginDrawing();
    ClearBackground(RAYWHITE);

    // World + ground
    DrawRectangleRec(worldRect, Color{ 240, 244, 250, 255 });
    DrawRectangle(PAL_W, (int)ed.groundY, worldW, SCREEN_H - (int)ed.groundY, Color{ 139, 115, 85, 255 });
    DrawRectangle(PAL_W, (int)ed.groundY, worldW, 4, Color{ 90, 70, 50, 255 });
    {
        float gx = PAL_W + worldW / 2.0f;
        bool hot = nearGround || ed.draggingGround;
        DrawRectangleRounded({ gx - 26, ed.groundY - 5, 52, 10 }, 0.8f, 8, hot ? ORANGE : Color{ 90, 70, 50, 255 });
        if (ed.groundY + 30 < SCREEN_H) {
            DrawText("Ground - drag the line to move it", PAL_W + 12, (int)ed.groundY + 14, 14, Fade(WHITE, 0.85f));
        }
    }

    for (int i = 0; i < (int)ed.bodies.size(); ++i) drawBody(ed.bodies[i], i == ed.selected);

    // ---- Top bar ----
    DrawRectangle(0, 0, SCREEN_W, TOP_H, Fade(LIGHTGRAY, 0.95f));
    DrawText("Sandbox Editor", 20, 13, 22, DARKGRAY);
    if (GuiButton({ 260, 10, 150, 30 }, "Back to Spawner")) {
        mode = AppMode::SPAWNER;
        accumulator = 0.0f;
        ed.dragging = -1;
        ed.paletteDrag = -1;
        ed.draggingGround = false;
        SetMouseCursor(MOUSE_CURSOR_DEFAULT);
    }
    GuiCheckBox({ 440, 15, 20, 20 }, "Paused", &ed.paused);
    if (GuiButton({ 560, 10, 100, 30 }, "Clear all")) {
        ed.bodies.clear();
        ed.selected = -1;
        ed.dragging = -1;
    }
    DrawText(TextFormat("Bodies: %d", (int)ed.bodies.size()), 690, 16, 18, DARKGRAY);
    DrawText("Drag shapes in from the left, click one to edit it", 800, 18, 14, GRAY);

    // ---- Palette (left) ----
    DrawRectangle(0, TOP_H, PAL_W, SCREEN_H - TOP_H, Fade(LIGHTGRAY, 0.95f));
    DrawText("Palette", 20, TOP_H + 10, 18, DARKGRAY);
    DrawText("drag into the world", 20, TOP_H + 32, 12, GRAY);
    static const char* paletteNames[PALETTE_COUNT] = { "Circle", "Triangle", "Square", "Rectangle", "Polygon" };
    for (int i = 0; i < PALETTE_COUNT; ++i) {
        Rectangle cell = paletteCell(i);
        bool hovered = CheckCollisionPointRec(mouse, cell);
        DrawRectangleRec(cell, hovered ? Fade(SKYBLUE, 0.35f) : WHITE);
        DrawRectangleLinesEx(cell, 1.0f, GRAY);
        Body icon = makeShape(i, Vec2{ cell.x + cell.width / 2.0f, cell.y + 30.0f }, 20.0f,
                              (int)ed.paletteSides, 0.0f, SKYBLUE);
        drawBody(icon);
        DrawText(paletteNames[i], (int)cell.x + 8, (int)cell.y + 58, 14, DARKGRAY);
    }
    GuiSlider({ 55, (float)(TOP_H + 55 + PALETTE_COUNT * 85 + 10), 70, 20 }, "Sides",
              TextFormat("%d", (int)ed.paletteSides), &ed.paletteSides, 3.0f, 12.0f);

    // ---- Inspector (right) ----
    DrawRectangle(insX, TOP_H, INS_W, SCREEN_H - TOP_H, Fade(LIGHTGRAY, 0.95f));
    DrawText("Inspector", insX + 15, TOP_H + 10, 18, DARKGRAY);

    if (ed.selected >= 0) {
        Body& s = ed.bodies[ed.selected];
        DrawText(shapeName(s), insX + 15, 95, 16, DARKGRAY);

        // Size (uniform scale of the whole shape)
        float newSize = s.radius;
        GuiSlider({ (float)insX + 70, 125, 100, 20 }, "Size", TextFormat("%.0f", newSize), &newSize, 8.0f, 120.0f);
        if (std::fabs(newSize - s.radius) > 0.01f) s.scaleBy(newSize / s.radius);

        // Gravity acting on this shape, in m/s^2
        float gMs2 = s.gravityScale * EARTH_G;
        float newG = gMs2;
        GuiSlider({ (float)insX + 70, 155, 100, 20 }, "Gravity", TextFormat("%.1f m/s^2", newG), &newG, 0.0f, 30.0f);
        if (std::fabs(newG - gMs2) > 0.001f) s.gravityScale = newG / EARTH_G;

        // Hex color: text box + live swatch
        if (!ed.hexEdit) {
            std::snprintf(ed.hexBuf, sizeof(ed.hexBuf), "#%02X%02X%02X", s.color.r, s.color.g, s.color.b);
        }
        DrawText("Hex", insX + 15, 196, 14, DARKGRAY);
        if (GuiTextBox({ (float)insX + 50, 190, 100, 26 }, ed.hexBuf, 8, ed.hexEdit)) ed.hexEdit = !ed.hexEdit;
        if (ed.hexEdit) {
            Color parsed;
            if (parseHexColor(ed.hexBuf, parsed)) s.color = parsed;
        }
        DrawRectangle(insX + 160, 190, 26, 26, s.color);
        DrawRectangleLines(insX + 160, 190, 26, 26, DARKGRAY);

        // Color picker (only adopt its value while the user is actually dragging in it)
        Rectangle pickerArea = { (float)insX + 10, 230, 215, 160 };
        Color picked = s.color;
        GuiColorPicker({ (float)insX + 15, 235, 150, 150 }, "", &picked);
        if (IsMouseButtonDown(MOUSE_BUTTON_LEFT) && CheckCollisionPointRec(mouse, pickerArea) &&
            (picked.r != s.color.r || picked.g != s.color.g || picked.b != s.color.b)) {
            s.color = Color{ picked.r, picked.g, picked.b, 255 };
        }

        if (GuiButton({ (float)insX + 15, 405, 130, 30 }, "Delete shape")) deleteSelected = true;
        DrawText("or press Delete", insX + 155, 413, 12, GRAY);
    } else {
        DrawText("Click a shape to edit it,", insX + 15, 95, 16, DARKGRAY);
        DrawText("or drag one in from the palette.", insX + 15, 115, 16, DARKGRAY);
        DrawText("Size, hex color and gravity", insX + 15, 150, 14, GRAY);
        DrawText("are set per shape.", insX + 15, 168, 14, GRAY);
    }

    // World settings
    DrawText("World", insX + 15, 470, 18, DARKGRAY);
    GuiSlider({ (float)insX + 70, 500, 100, 20 }, "Ground Y", TextFormat("%.0f px", ed.groundY),
              &ed.groundY, (float)(TOP_H + 100), (float)(SCREEN_H - 10));
    GuiSlider({ (float)insX + 70, 530, 100, 20 }, "Bounce", TextFormat("%.2f", ed.params.restitution),
              &ed.params.restitution, 0.0f, 1.0f);

    // Drag-and-drop ghost (on top of everything)
    if (ed.paletteDrag >= 0) {
        Body ghost = makeShape(ed.paletteDrag, m, 28.0f, (int)ed.paletteSides, 0.0f, ed.paletteColor);
        drawBody(ghost, false, true);
    }

    EndDrawing();

    if (deleteSelected && ed.selected >= 0 && ed.selected < (int)ed.bodies.size()) {
        ed.bodies.erase(ed.bodies.begin() + ed.selected);
        ed.selected = -1;
        ed.dragging = -1;
    }
}

int main() {
    InitWindow(SCREEN_W, SCREEN_H, "C++ Physics Engine - raylib + raygui");
    SetTargetFPS(60);

    AppMode mode = AppMode::SPAWNER;
    SpawnerState spawner;
    Editor editor;
    editor.params.gravity = EARTH_G * PIXELS_PER_METER; // 1g; per-shape gravity scales this
    editor.params.restitution = 0.4f;

    float accumulator = 0.0f;

    while (!WindowShouldClose()) {
        if (mode == AppMode::SPAWNER) spawnerFrame(spawner, accumulator, mode);
        else editorFrame(editor, accumulator, mode);
    }

    CloseWindow();
    return 0;
}
