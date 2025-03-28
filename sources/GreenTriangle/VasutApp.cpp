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
const float g = 40.0f; // gravity (40 m/s^2)
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
    float t = 0.0f; // spline parameter
    float speed = 0.0f;
    vec2 wheelPosition;
    vec2 wheelTangent;
    float wheelAngle = 0.0f;

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

    void drawWheel() {
        // 1. Draw wheel outline (blue)
        Geometry<vec2> wheelOutline;
        for (int i = 0; i <= 36; i++) { // First 37 points are the circle
            vec2 v = wheel->Vtx()[i];
            vec2 transformed = vec2(
                v.x * cos(wheelAngle) - v.y * sin(wheelAngle) + wheelPosition.x,
                v.x * sin(wheelAngle) + v.y * cos(wheelAngle) + wheelPosition.y
            );
            wheelOutline.Vtx().push_back(transformed);
        }
        wheelOutline.updateGPU();
        wheelOutline.Draw(gpuProgram, GL_LINE_STRIP, vec3(0, 0, 1));

        // 2. Draw spokes (white)
        Geometry<vec2> wheelSpokes;
        for (int i = 37; i < wheel->Vtx().size(); i++) {
            vec2 v = wheel->Vtx()[i];
            vec2 transformed = vec2(
                v.x * cos(wheelAngle) - v.y * sin(wheelAngle) + wheelPosition.x,
                v.x * sin(wheelAngle) + v.y * cos(wheelAngle) + wheelPosition.y
            );
            wheelSpokes.Vtx().push_back(transformed);
        }
        wheelSpokes.updateGPU();
        wheelSpokes.Draw(gpuProgram, GL_LINES, vec3(1, 1, 1));
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
        printf("Updating spline with %d points\n", pointList.size());
        splinePoints->Vtx().clear();
        if (pointList.size() < 2) return;

        const int segments = 100;
        for (int i = 0; i <= segments; i++) {
            float t = (float)i / segments;
            splinePoints->Vtx().push_back(calculateSplinePoint(t));
        }
        splinePoints->updateGPU();
    }

    void onDisplay() {
        glClearColor(0, 0, 0, 0);
        glClear(GL_COLOR_BUFFER_BIT);

        // Draw spline
        if (pointList.size() >= 2) {
            splinePoints->Draw(gpuProgram, GL_LINE_STRIP, vec3(1, 1, 0));
        }

        // Draw control points
        controlPoints->Draw(gpuProgram, GL_POINTS, vec3(1, 0, 0));

        // Draw wheel
        if (isSimulating) {
            drawWheel();
        }

        if (isSimulating) {
            printf("Drawing wheel at (%.2f, %.2f), angle: %.1f°\n",
                wheelPosition.x, wheelPosition.y, wheelAngle * 180 / M_PI);

            // 1. Draw transformed wheel points (red)
            Geometry<vec2> debugWheel;
            for (auto& v : wheel->Vtx()) {
                vec2 transformed(
                    v.x * cos(wheelAngle) - v.y * sin(wheelAngle) + wheelPosition.x,
                    v.x * sin(wheelAngle) + v.y * cos(wheelAngle) + wheelPosition.y
                );
                debugWheel.Vtx().push_back(transformed);
            }
            debugWheel.updateGPU();
            debugWheel.Draw(gpuProgram, GL_POINTS, vec3(1, 0, 0));

            // 2. Draw wheel center (big green point)
            Geometry<vec2> center;
            center.Vtx().push_back(wheelPosition);
            center.updateGPU();
            glPointSize(20.0f);
            center.Draw(gpuProgram, GL_POINTS, vec3(0, 1, 0));
            glPointSize(10.0f);
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
            printf("=== STARTING SIMULATION ===\n");
            printf("Control points: %d\n", pointList.size());

            // Initialize simulation state
            isSimulating = true;
            t = 0.0f;
            speed = 0.0f;
            wheelAngle = 0.0f;
            wheelPosition = calculateSplinePoint(t); // Only set position once

            printf("Initial wheel position: (%.2f, %.2f)\n", wheelPosition.x, wheelPosition.y);
            refreshScreen();
        }
    }

    void onTimeElapsed(float startTime, float endTime) {
        if (!isSimulating) return;

        float deltaTime = endTime - startTime;
        if (deltaTime <= 0) return;

        // Get current position and derivatives
        wheelPosition = calculateSplinePoint(t);
        vec2 derivative = calculateSplineDerivative(t);
        float tangentLength = length(derivative);

        if (tangentLength < 0.0001f) {
            isSimulating = false;
            return;
        }

        // Calculate tangent and normal vectors
        vec2 tangent = derivative / tangentLength;
        vec2 normal(-tangent.y, tangent.x);

        // Calculate curvature (dT/ds)
        vec2 dT = calculateSplineSecondDerivative(t) - tangent * dot(calculateSplineSecondDerivative(t), tangent);
        float curvature = length(dT) / (tangentLength * tangentLength);

        // Calculate required centripetal force
        float requiredForce = speed * speed * curvature;

        // Calculate available normal force (gravity component)
        float gravityComponent = dot(vec2(0, -g), normal);

        // Check if wheel stays on track
        if (gravityComponent >= requiredForce) {
            // On track - calculate new speed using energy conservation
            float deltaHeight = pointList[0].y - wheelPosition.y;
            speed = sqrt(2.0f * g * deltaHeight / (1.0f + 1.0f));

            // Update position
            t += speed * deltaTime / tangentLength;
            t = fmod(t, 1.0f); // Wrap around if needed

            // Update rotation
            wheelAngle += speed * deltaTime / wheelRadius;
        }
        else {
            // Wheel falls off
            isSimulating = false;
        }

        refreshScreen();
    }
};
VasutApp app;