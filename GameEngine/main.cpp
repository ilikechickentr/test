#include "raylib.h"

#define RAYGUI_IMPLEMENTATION
#include "raygui.h"

#include <vector>
#include <cmath>
#include <cstdlib>
#include <algorithm>
#include <limits>

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
// ---------------------------------------------------------
void integrate(std::vector<Body>& bodies, const SimParams& params, float dt) {
    for (auto& b : bodies) {
        if (b.invMass == 0.0f) continue; // static body, skip
        b.velocity.y += params.gravity * dt;
        b.position += b.velocity * dt;
    }
}

// ---------------------------------------------------------
// Step 2: Resolve collisions against screen bounds (walls)
// ---------------------------------------------------------
void resolveWallCollisions(std::vector<Body>& bodies, int screenW, int screenH,
                            const SimParams& params) {
    for (auto& b : bodies) {
        if (b.shape == ShapeType::CIRCLE) {
            if (b.position.x - b.radius < 0.0f) {
                b.position.x = b.radius;
                b.velocity.x = -b.velocity.x * params.restitution;
            } else if (b.position.x + b.radius > screenW) {
                b.position.x = screenW - b.radius;
                b.velocity.x = -b.velocity.x * params.restitution;
            }
            if (b.position.y - b.radius < 0.0f) {
                b.position.y = b.radius;
                b.velocity.y = -b.velocity.y * params.restitution;
            } else if (b.position.y + b.radius > screenH) {
                b.position.y = screenH - b.radius;
                b.velocity.y = -b.velocity.y * params.restitution;
            }
        } else {
            // Polygon: push back based on its world-space bounding box.
            // (Simplification: treats the polygon's axis-aligned bounds as
            // the collider against the walls, rather than full edge SAT.)
            std::vector<Vec2> verts = b.worldVertices();
            float minX = std::numeric_limits<float>::max(), maxX = -std::numeric_limits<float>::max();
            float minY = std::numeric_limits<float>::max(), maxY = -std::numeric_limits<float>::max();
            for (const auto& v : verts) {
                minX = std::min(minX, v.x); maxX = std::max(maxX, v.x);
                minY = std::min(minY, v.y); maxY = std::max(maxY, v.y);
            }

            if (minX < 0.0f) {
                b.position.x -= minX;
                b.velocity.x = -b.velocity.x * params.restitution;
            } else if (maxX > screenW) {
                b.position.x -= (maxX - screenW);
                b.velocity.x = -b.velocity.x * params.restitution;
            }

            if (minY < 0.0f) {
                b.position.y -= minY;
                b.velocity.y = -b.velocity.y * params.restitution;
            } else if (maxY > screenH) {
                b.position.y -= (maxY - screenH);
                b.velocity.y = -b.velocity.y * params.restitution;
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

int main() {
    const int screenWidth = 1000;
    const int screenHeight = 700;
    const float PIXELS_PER_METER = 50.0f;
    const float PANEL_HEIGHT = 125.0f;

    InitWindow(screenWidth, screenHeight, "C++ Physics Engine - raylib + raygui");
    SetTargetFPS(60);

    std::vector<Body> bodies;
    SimParams params;

    // Shape-spawning UI state
    int shapeSelection = 0;     // 0=Circle, 1=Triangle, 2=Square, 3=Rectangle, 4=Polygon
    float polygonSidesF = 5.0f; // only used when shapeSelection == 4

    // Fixed timestep accumulator (decouples physics from render framerate)
    const float fixedDt = 1.0f / 60.0f;
    float accumulator = 0.0f;

    while (!WindowShouldClose()) {
        // -----------------------------------------------------
        // Input: click to spawn a new body at mouse position,
        // using whichever shape is selected in the panel
        // -----------------------------------------------------
        if (IsMouseButtonPressed(MOUSE_BUTTON_LEFT) && GetMouseY() > PANEL_HEIGHT) {
            Vec2 spawnPos{ (float)GetMouseX(), (float)GetMouseY() };
            Color c = { (unsigned char)GetRandomValue(80, 255),
                        (unsigned char)GetRandomValue(80, 255),
                        (unsigned char)GetRandomValue(80, 255), 255 };
            float randRot = (float)GetRandomValue(0, 628) / 100.0f; // cosmetic spawn rotation, 0-2pi

            if (shapeSelection == 0) {
                float r = 12.0f + (float)GetRandomValue(0, 15);
                bodies.emplace_back(spawnPos, r, r * r * 0.05f, c);
            } else if (shapeSelection == 1) {
                float r = 18.0f + (float)GetRandomValue(0, 10);
                auto verts = makeRegularPolygon(3, r);
                bodies.emplace_back(spawnPos, verts, r, randRot, r * r * 0.05f, c);
            } else if (shapeSelection == 2) {
                float r = 16.0f + (float)GetRandomValue(0, 10);
                auto verts = makeRegularPolygon(4, r);
                bodies.emplace_back(spawnPos, verts, r, randRot, r * r * 0.08f, c);
            } else if (shapeSelection == 3) {
                float w = 40.0f + (float)GetRandomValue(0, 20);
                float h = 20.0f + (float)GetRandomValue(0, 15);
                auto verts = makeRectangle(w, h);
                float boundingRadius = sqrtf((w * 0.5f) * (w * 0.5f) + (h * 0.5f) * (h * 0.5f));
                bodies.emplace_back(spawnPos, verts, boundingRadius, randRot, (w * h) * 0.002f, c);
            } else {
                int sides = (int)polygonSidesF;
                float r = 16.0f + (float)GetRandomValue(0, 10);
                auto verts = makeRegularPolygon(sides, r);
                bodies.emplace_back(spawnPos, verts, r, randRot, r * r * 0.05f, c);
            }
        }

        // -----------------------------------------------------
        // Fixed-timestep physics loop
        // -----------------------------------------------------
        if (!params.paused) {
            accumulator += GetFrameTime();
            while (accumulator >= fixedDt) {
                integrate(bodies, params, fixedDt);
                resolveCollisions(bodies, params);
                resolveWallCollisions(bodies, screenWidth, screenHeight, params);
                accumulator -= fixedDt;
            }
        }

        // -----------------------------------------------------
        // Render
        // -----------------------------------------------------
        BeginDrawing();
        ClearBackground(RAYWHITE);

        for (const auto& b : bodies) {
            if (b.shape == ShapeType::CIRCLE) {
                DrawCircleV({ b.position.x, b.position.y }, b.radius, b.color);
                DrawCircleLines((int)b.position.x, (int)b.position.y, b.radius, DARKGRAY);
            } else {
                std::vector<Vec2> verts = b.worldVertices();

                std::vector<Vector2> fan;
                fan.reserve(verts.size() + 2);
                fan.push_back({ b.position.x, b.position.y }); // fan center
                for (const auto& v : verts) fan.push_back({ v.x, v.y });
                fan.push_back({ verts[0].x, verts[0].y }); // close the fan
                DrawTriangleFan(fan.data(), (int)fan.size(), b.color);

                for (size_t k = 0; k < verts.size(); ++k) {
                    Vec2 p1 = verts[k];
                    Vec2 p2 = verts[(k + 1) % verts.size()];
                    DrawLineEx({ p1.x, p1.y }, { p2.x, p2.y }, 2.0f, DARKGRAY);
                }
            }
        }

        // --- raygui control panel ---
        DrawRectangle(0, 0, screenWidth, (int)PANEL_HEIGHT, Fade(LIGHTGRAY, 0.9f));

        GuiSlider({ 120, 15, 200, 20 }, "Gravity", TextFormat("%.2f m/s^2", params.gravity / PIXELS_PER_METER),
                  &params.gravity, 0.0f, 2000.0f);
        GuiSlider({ 120, 45, 200, 20 }, "Bounciness", TextFormat("%.2f", params.restitution),
                  &params.restitution, 0.0f, 1.0f);
        GuiCheckBox({ 400, 15, 20, 20 }, "Paused", &params.paused);

        if (GuiButton({ 350, 45, 100, 25 }, "Clear all")) {
            bodies.clear();
        }

        DrawText("Shape to spawn:", 480, 18, 16, DARKGRAY);
        GuiToggleGroup({ 480, 80, 70, 25 }, "Circle;Triangle;Square;Rectangle;Polygon", &shapeSelection);

        if (shapeSelection == 4) {
            GuiSlider({ 850, 85, 130, 20 }, "Sides", TextFormat("%d", (int)polygonSidesF),
                      &polygonSidesF, 3.0f, 12.0f);
        }

        DrawText(TextFormat("Bodies: %d", (int)bodies.size()), 480, 45, 18, DARKGRAY);
        DrawText("Click below the panel to spawn", 650, 18, 16, GRAY);

        EndDrawing();
    }

    CloseWindow();
    return 0;
}
