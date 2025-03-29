#include "framework.h"
#include <vector>
#include <cmath>

const char* vertSource = R"(
    #version 330                
    precision highp float;

    layout(location = 0) in vec2 cP;    // vertex position

    void main() {
        gl_Position = vec4(cP.x, cP.y, 0, 1);
    }
)";

const char* fragSource = R"(
    #version 330
    precision highp float;

    uniform vec3 color;            // constant color
    out vec4 fragmentColor;        // pixel color

    void main() {
        fragmentColor = vec4(color, 1);
    }
)";

const int winWidth = 600, winHeight = 600;
const float g = 10.0f; // gravity (40 m/s^2)
const float wheelRadius = 1.0f / 20.0f; // 1m in normalized coordinates (20m world)
const float dt = 0.01f; // time step for simulation

class VasutApp : public glApp {
    Geometry<vec2>* controlPoints;
    Geometry<vec2>* splinePoints;
    Geometry<vec2>* wheel;
    std::vector<vec2> pointList;
    GPUProgram* gpuProgram;

    // Simulation variables
    bool isSimulating = false;
    bool normalFlipped = false;  // Add this to track normal orientation
    float t = 0.0f; // spline parameter
    float speed = 0.0f;
    float wheelAngle = 0.0f;
    const float minSpeed = 0.0f;      // Lower minimum speed
    const float maxSpeed = 2.0f;       // Reduced maximum speed
    const float friction = 0.3f;       // Increased friction
    const float speedFactor = 0.3f;    // Global speed reduction factor
    vec2 wheelPosition = vec2(0, 0);
    vec2 wheelTangent = vec2(1, 0);
    vec2 wheelNormal = vec2(1, 0);  // Add this line to store the normal vector

public:
    VasutApp() : glApp("Roller Coaster Simulation") {}

    void onInitialization() {
        controlPoints = new Geometry<vec2>();
        splinePoints = new Geometry<vec2>();
        wheel = new Geometry<vec2>();
        gpuProgram = new GPUProgram(vertSource, fragSource);
        glPointSize(10.0f);
        glLineWidth(3.0f);

        // Initialize wheel geometry (circle with spokes)
        createWheelGeometry();
    }

    void createWheelGeometry() {
        wheel->Vtx().clear();
        const int segments = 36;
        const int spokes = 4;

        // Circle outline
        for (int i = 0; i <= segments; i++) {
            float angle = 2.0f * M_PI * i / segments;
            vec2 point(wheelRadius * cos(angle), wheelRadius * sin(angle));
            wheel->Vtx().push_back(point);
            printf("Circle point %d: (%.3f, %.3f)\n", i, point.x, point.y); // DEBUG
        }

        // Spokes
        for (int i = 0; i < spokes; i++) {
            float angle = 2.0f * M_PI * i / spokes;
            wheel->Vtx().push_back(vec2(0, 0));
            vec2 spokeEnd(wheelRadius * cos(angle), wheelRadius * sin(angle));
            wheel->Vtx().push_back(spokeEnd);
            printf("Spoke %d: center to (%.3f, %.3f)\n", i, spokeEnd.x, spokeEnd.y); // DEBUG
        }

        printf("Total wheel vertices: %zu\n", wheel->Vtx().size()); // DEBUG
        wheel->updateGPU();
    }

    vec2 calculateSplinePoint(float t) {
        if (pointList.size() < 2) return vec2(0, 0);

        // Clamp t to [0,1]
        t = std::max(0.0f, std::min(1.0f, t));

        int n = pointList.size();
        float segment = floor(t * (n - 1));
        float localT = t * (n - 1) - segment;

        int i0 = std::max(0, (int)segment - 1);
        int i1 = (int)segment;
        int i2 = std::min(n - 1, (int)segment + 1);
        int i3 = std::min(n - 1, (int)segment + 2);

        vec2 p0 = pointList[i0];
        vec2 p1 = pointList[i1];
        vec2 p2 = pointList[i2];
        vec2 p3 = pointList[i3];

        // Catmull-Rom spline calculation (optimized)
        float t2 = localT * localT;
        float t3 = t2 * localT;

        return 0.5f * (
            (2.0f * p1) +
            (-p0 + p2) * localT +
            (2.0f * p0 - 5.0f * p1 + 4.0f * p2 - p3) * t2 +
            (-p0 + 3.0f * p1 - 3.0f * p2 + p3) * t3
            );
    }

    vec2 calculateSplineDerivative(float t) {
        if (pointList.size() < 2) return vec2(0, 0);

        t = std::max(0.0f, std::min(1.0f, t));

        int n = pointList.size();
        float segment = floor(t * (n - 1));
        float localT = t * (n - 1) - segment;

        int i0 = std::max(0, (int)segment - 1);
        int i1 = (int)segment;
        int i2 = std::min(n - 1, (int)segment + 1);
        int i3 = std::min(n - 1, (int)segment + 2);

        vec2 p0 = pointList[i0];
        vec2 p1 = pointList[i1];
        vec2 p2 = pointList[i2];
        vec2 p3 = pointList[i3];

        // Properly typed derivative calculation
        return 0.5f * (
            (-p0 + p2) +
            (4.0f * p0 - 10.0f * p1 + 8.0f * p2 - 2.0f * p3) * localT +
            (-3.0f * p0 + 9.0f * p1 - 9.0f * p2 + 3.0f * p3) * localT * localT
            );
    }

    vec2 calculateSplineSecondDerivative(float t) {
        if (pointList.size() < 2) return vec2(0, 0);

        t = std::max(0.0f, std::min(1.0f, t));

        int n = pointList.size();
        float segment = floor(t * (n - 1));
        float localT = t * (n - 1) - segment;

        int i0 = std::max(0, (int)segment - 1);
        int i1 = (int)segment;
        int i2 = std::min(n - 1, (int)segment + 1);
        int i3 = std::min(n - 1, (int)segment + 2);

        vec2 p0 = pointList[i0];
        vec2 p1 = pointList[i1];
        vec2 p2 = pointList[i2];
        vec2 p3 = pointList[i3];

        // Second derivative of Catmull-Rom spline
        return 0.5f * (
            (4.0f * p0 - 10.0f * p1 + 8.0f * p2 - 2.0f * p3) +
            2.0f * (-3.0f * p0 + 9.0f * p1 - 9.0f * p2 + 3.0f * p3) * localT
            );
    }

    void updateSplineGeometry() {
        printf("Updating spline with %d points\n", (int)pointList.size());
        splinePoints->Vtx().clear();
        if (pointList.size() < 2) return;

        const int segments = 100;
        for (int i = 0; i <= segments; i++) {
            float t = (float)i / segments;
            splinePoints->Vtx().push_back(calculateSplinePoint(t));
        }
        splinePoints->updateGPU();

        refreshScreen();
    }

    void drawWheel() {
        if (!isSimulating || speed <= 0.001f) return;

        // 1. Draw filled blue wheel (using triangle fan)
        Geometry<vec2> wheelFill;
        wheelFill.Vtx().push_back(wheelPosition); // Center point

        const int segments = 36;
        for (int i = 0; i <= segments; i++) {
            float angle = 2.0f * M_PI * i / segments;
            vec2 point(
                wheelRadius * cos(angle) + wheelPosition.x,
                wheelRadius * sin(angle) + wheelPosition.y
            );
            wheelFill.Vtx().push_back(point);
        }
        wheelFill.updateGPU();
        wheelFill.Draw(gpuProgram, GL_TRIANGLE_FAN, vec3(0, 0, 1)); // Blue fill

        // 2. Draw white outline (using line loop)
        Geometry<vec2> wheelOutline;
        for (int i = 0; i <= segments; i++) {
            float angle = 2.0f * M_PI * i / segments;
            vec2 point(
                wheelRadius * cos(angle) + wheelPosition.x,
                wheelRadius * sin(angle) + wheelPosition.y
            );
            wheelOutline.Vtx().push_back(point);
        }
        wheelOutline.updateGPU();
        glLineWidth(1.5f); // Slightly thicker outline
        wheelOutline.Draw(gpuProgram, GL_LINE_LOOP, vec3(1, 1, 1)); // White outline
        glLineWidth(1.0f); // Reset to default

        // 3. Draw white spokes (using lines)
        Geometry<vec2> wheelSpokes;
        const int spokeCount = 4;
        for (int i = 0; i < spokeCount; i++) {
            float spokeAngle = wheelAngle + 2.0f * M_PI * i / spokeCount;
            wheelSpokes.Vtx().push_back(wheelPosition); // Center
            wheelSpokes.Vtx().push_back(vec2(
                wheelRadius * cos(spokeAngle) + wheelPosition.x,
                wheelRadius * sin(spokeAngle) + wheelPosition.y
            ));
        }
        wheelSpokes.updateGPU();
        wheelSpokes.Draw(gpuProgram, GL_LINES, vec3(1, 1, 1)); // White spokes
    }

    void onDisplay() {
        glClearColor(0, 0, 0, 0);
        glClear(GL_COLOR_BUFFER_BIT);

        // Draw spline
        if (pointList.size() > 1) {
            splinePoints->Draw(gpuProgram, GL_LINE_STRIP, vec3(1, 1, 0));
        }

        // Draw control points
        controlPoints->Draw(gpuProgram, GL_POINTS, vec3(1, 0, 0));

        // Draw wheel if simulating
        if (isSimulating) {
            drawWheel();
        }
    }

    void onMousePressed(MouseButton button, int pX, int pY) {
        if (button == MOUSE_LEFT && !isSimulating) {
            float x = 2.0f * pX / winWidth - 1;
            float y = 1 - 2.0f * pY / winHeight;
            vec2 clickedPoint(x, y);

            pointList.push_back(clickedPoint);
            controlPoints->Vtx().push_back(clickedPoint);
            controlPoints->updateGPU();

            updateSplineGeometry();
            refreshScreen();
        }
    }
    void onKeyboard(int key) {
        if (key == ' ' && pointList.size() >= 2 && !isSimulating) {
            isSimulating = true;
            t = 0.0f;
            speed = 0.1f;
            wheelAngle = 0.0f;
            wheelPosition = calculateSplinePoint(t);
            vec2 derivative = calculateSplineDerivative(t);

            if (length(derivative) > 0) {
                wheelTangent = normalize(derivative);
                wheelNormal = vec2(-wheelTangent.y, wheelTangent.x);

                // Determine initial normal orientation once
                normalFlipped = (wheelNormal.y < 0);
                if (normalFlipped) {
                    wheelNormal = -wheelNormal;
                }
            }
            refreshScreen();
        }
    }

    void onTimeElapsed(float startTime, float endTime) {
        if (!isSimulating) return;

        float deltaTime = endTime - startTime;
        if (deltaTime <= 0) return;

        // 1. Get current spline position and derivatives
        vec2 splinePoint = calculateSplinePoint(t);
        vec2 derivative = calculateSplineDerivative(t);
        float tangentLength = length(derivative);

        if (tangentLength < 0.0001f) {
            isSimulating = false;
            return;
        }

        // 2. Calculate tangent and maintain consistent normal orientation
        wheelTangent = normalize(derivative);

        // Calculate base normal (90� rotation of tangent)
        vec2 baseNormal = vec2(-wheelTangent.y, wheelTangent.x);

        // Apply initial flip decision consistently
        wheelNormal = normalFlipped ? -baseNormal : baseNormal;

        // 3. Position wheel on its original side of track
        wheelPosition = splinePoint + wheelNormal * wheelRadius;

        // 4. Calculate gravitational acceleration along the track
        vec2 gravity(0, -g);
        float acceleration = dot(gravity, wheelTangent) - friction;

        // 5. Update speed with controlled acceleration
        speed += acceleration * deltaTime;
        speed = fmax(speed, 0.0f);  // Allow speed to reach 0
        speed = fmin(speed, maxSpeed);

        // 6. Check if wheel has stopped
        if (speed <= 0.001f) {
            isSimulating = false;
            refreshScreen();
            return;
        }

        // 7. Update position parameter
        t += speed * deltaTime / tangentLength;

        // 8. Handle spline boundaries
        if (t < 0) {
            t = 0;
            speed = 0;
            isSimulating = false;
        }
        else if (t > 1) {
            t = 1;
            speed = 0;
            isSimulating = false;
        }

        // 9. Update wheel rotation
        float distanceMoved = speed * deltaTime;
        wheelAngle -= distanceMoved / wheelRadius;

        refreshScreen();
    }
};
VasutApp app;