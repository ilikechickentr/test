#include "raylib.h"

#define RAYGUI_IMPLEMENTATION
#include "raygui.h"

#include <vector>
#include <cmath>
#include <cstdlib>

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
// A rigid circle body
// ---------------------------------------------------------
struct Body {
    Vec2 position;
    Vec2 velocity;
    float radius;
    float mass;
    float invMass;   // 0 = infinite mass (static/immovable)
    Color color;

    Body(Vec2 pos, float r, float m, Color c)
        : position(pos), velocity{ 0, 0 }, radius(r), mass(m), color(c) {
        invMass = (m <= 0.0f) ? 0.0f : 1.0f / m;
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
        // Left / right walls
        if (b.position.x - b.radius < 0.0f) {
            b.position.x = b.radius;
            b.velocity.x = -b.velocity.x * params.restitution;
        } else if (b.position.x + b.radius > screenW) {
            b.position.x = screenW - b.radius;
            b.velocity.x = -b.velocity.x * params.restitution;
        }

        // Floor / ceiling
        if (b.position.y - b.radius < 0.0f) {
            b.position.y = b.radius;
            b.velocity.y = -b.velocity.y * params.restitution;
        } else if (b.position.y + b.radius > screenH) {
            b.position.y = screenH - b.radius;
            b.velocity.y = -b.velocity.y * params.restitution;
        }
    }
}

// ---------------------------------------------------------
// Step 3: Narrow-phase + resolution for circle-circle pairs
// (Naive O(n^2) check -- fine for a few hundred bodies;
//  swap in a spatial grid later if you need more.)
// ---------------------------------------------------------
void resolveCircleCollisions(std::vector<Body>& bodies, const SimParams& params) {
    for (size_t i = 0; i < bodies.size(); ++i) {
        for (size_t j = i + 1; j < bodies.size(); ++j) {
            Body& a = bodies[i];
            Body& b = bodies[j];

            Vec2 delta = b.position - a.position;
            float dist = delta.length();
            float minDist = a.radius + b.radius;

            if (dist >= minDist || dist < 1e-6f) continue; // not colliding

            Vec2 normal = delta.normalized();
            float penetration = minDist - dist;

            // --- Positional correction (stop sinking into each other) ---
            float totalInvMass = a.invMass + b.invMass;
            if (totalInvMass <= 0.0f) continue; // both static

            const float percent = 0.8f; // how much penetration to correct per step
            Vec2 correction = normal * (penetration / totalInvMass * percent);
            a.position -= correction * a.invMass;
            b.position += correction * b.invMass;

            // --- Impulse resolution ---
            Vec2 relativeVelocity = b.velocity - a.velocity;
            float velAlongNormal = relativeVelocity.dot(normal);

            if (velAlongNormal > 0) continue; // already separating

            float impulseMag = -(1.0f + params.restitution) * velAlongNormal / totalInvMass;
            Vec2 impulse = normal * impulseMag;

            a.velocity -= impulse * a.invMass;
            b.velocity += impulse * b.invMass;
        }
    }
}

int main() {
    const int screenWidth = 1000;
    const int screenHeight = 700;
    const float PIXELS_PER_METER = 50.0f;
    InitWindow(screenWidth, screenHeight, "C++ Physics Engine - raylib + raygui");
    SetTargetFPS(60);

    std::vector<Body> bodies;
    SimParams params;

    // Fixed timestep accumulator (decouples physics from render framerate)
    const float fixedDt = 1.0f / 60.0f;
    float accumulator = 0.0f;

    while (!WindowShouldClose()) {
        // -----------------------------------------------------
        // Input: click to spawn a new circle at mouse position
        // -----------------------------------------------------
        if (IsMouseButtonPressed(MOUSE_BUTTON_LEFT) && GetMouseY() > 90) {
            float r = 12.0f + static_cast<float>(GetRandomValue(0, 15));
            Color c = { (unsigned char)GetRandomValue(80, 255),
                        (unsigned char)GetRandomValue(80, 255),
                        (unsigned char)GetRandomValue(80, 255), 255 };
            bodies.emplace_back(Vec2{ (float)GetMouseX(), (float)GetMouseY() }, r, r * r * 0.05f, c);
        }

        // -----------------------------------------------------
        // Fixed-timestep physics loop
        // -----------------------------------------------------
        if (!params.paused) {
            accumulator += GetFrameTime();
            while (accumulator >= fixedDt) {
                integrate(bodies, params, fixedDt);
                resolveCircleCollisions(bodies, params);
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
            DrawCircleV({ b.position.x, b.position.y }, b.radius, b.color);
            DrawCircleLines((int)b.position.x, (int)b.position.y, b.radius, DARKGRAY);
        }

        // --- raygui control panel ---
        DrawRectangle(0, 0, screenWidth, 90, Fade(LIGHTGRAY, 0.9f));
        GuiSlider({ 120, 15, 200, 20 }, "Gravity", TextFormat("%.2f m/s^2", params.gravity / PIXELS_PER_METER),
                  &params.gravity, 0.0f, 2000.0f);
        GuiSlider({ 120, 45, 200, 20 }, "Bounciness", TextFormat("%.2f", params.restitution),
                  &params.restitution, 0.0f, 1.0f);
        GuiCheckBox({ 400, 15, 20, 20 }, "Paused", &params.paused);

        if (GuiButton({ 350, 45, 100, 25 }, "Clear all")) {
            bodies.clear();
        }

        DrawText(TextFormat("Bodies: %d", (int)bodies.size()), 480, 15, 18, DARKGRAY);
        DrawText("Click below the panel to spawn a circle", 480, 45, 16, GRAY);

        EndDrawing();
    }

    CloseWindow();
    return 0;
}
