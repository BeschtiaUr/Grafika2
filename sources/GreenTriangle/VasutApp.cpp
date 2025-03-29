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
    bool isFalling = false;           // Track falling state
    float t = 0.0f; // spline parameter
    float speed = 0.0f;
    float wheelAngle = 0.0f;
    float fallStartTime;
    const float minSpeed = 0.0f;      // Lower minimum speed
    const float maxSpeed = 2.0f;       // Reduced maximum speed
    const float friction = 0.3f;       // Increased friction
    const float speedFactor = 0.3f;    // Global speed reduction factor
    vec2 fallVelocity;
    vec2 fallStartPosition;
    vec2 wheelPosition = vec2(0, 0);
    vec2 wheelTangent = vec2(1, 0);
    vec2 wheelNormal = vec2(1, 0);  // Add this line to store the normal vector
    vec2 velocity = vec2(0, 0);       // Track velocity when falling
    vec3 wheelColor = vec3(0, 0, 1);  // Blue normally, red when falling

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
        if (!isSimulating) return;

        // 1. Draw filled blue wheel (red if falling)
        Geometry<vec2> wheelFill;
        wheelFill.Vtx().push_back(wheelPosition); // Center

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
        wheelFill.Draw(gpuProgram, GL_TRIANGLE_FAN, wheelColor);

        // 2. Draw white outline
        Geometry<vec2> wheelOutline;
        for (const auto& v : wheelFill.Vtx()) {
            if (v != wheelFill.Vtx()[0]) // Skip center point
                wheelOutline.Vtx().push_back(v);
        }
        wheelOutline.updateGPU();
        glLineWidth(1.5f);
        wheelOutline.Draw(gpuProgram, GL_LINE_LOOP, vec3(1, 1, 1));
        glLineWidth(1.0f);

        // 3. Draw spokes
        Geometry<vec2> wheelSpokes;
        const int spokeCount = 4;
        for (int i = 0; i < spokeCount; i++) {
            float spokeAngle = wheelAngle + 2.0f * M_PI * i / spokeCount;
            wheelSpokes.Vtx().push_back(wheelPosition);
            wheelSpokes.Vtx().push_back(vec2(
                wheelRadius * cos(spokeAngle) + wheelPosition.x,
                wheelRadius * sin(spokeAngle) + wheelPosition.y
            ));
        }
        wheelSpokes.updateGPU();
        glLineWidth(2.0f);
        wheelSpokes.Draw(gpuProgram, GL_LINES, vec3(1, 1, 1));
        glLineWidth(1.0f);
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

    // Helper function to calculate reflection manually
    vec2 reflectVector(vec2 incident, vec2 normal) {
        return incident - 2.0f * dot(incident, normal) * normal;
    }

    void onTimeElapsed(float startTime, float endTime) {
        if (!isSimulating) return;

        float deltaTime = endTime - startTime;
        if (deltaTime <= 0) return;

        // 1. Get current track information
        vec2 splinePoint = calculateSplinePoint(t);
        vec2 derivative = calculateSplineDerivative(t);
        float tangentLength = length(derivative);

        // 2. Calculate track orientation
        wheelTangent = tangentLength > 0 ? normalize(derivative) : vec2(1, 0);
        vec2 normal(-wheelTangent.y, wheelTangent.x);

        // 3. Physics calculations
        float curvature = length(calculateSplineSecondDerivative(t)) / (tangentLength * tangentLength);
        float requiredNormalForce = speed * speed * curvature;
        float availableNormalForce = dot(vec2(0, -g), normal);

        // 4. Check for falling condition
        if (!isFalling && availableNormalForce < requiredNormalForce) {
            isFalling = true;
            fallVelocity = speed * wheelTangent; // Initial falling velocity
            fallStartTime = endTime;
            wheelColor = vec3(1, 0, 0); // Red when falling
        }

        // 5. Handle movement based on state
        if (!isFalling) {
            // On-track movement
            float acceleration = dot(vec2(0, -g), wheelTangent) - friction;
            speed += acceleration * deltaTime;
            speed = fmax(speed, 0);

            t += speed * deltaTime / tangentLength;
            wheelPosition = splinePoint + normal * wheelRadius;
            wheelAngle -= speed * deltaTime / wheelRadius;

            if (speed <= 0.001f) isSimulating = false;
        }
        else {
            // Falling movement with collision detection
            vec2 gravity(0, -g);
            vec2 newPosition = wheelPosition + fallVelocity * deltaTime +
                0.5f * gravity * deltaTime * deltaTime;

            // Collision detection against track
            bool collided = false;
            vec2 collisionNormal;
            float closestDistance = wheelRadius * 1.1f; // 10% tolerance

            // Only check nearby points for performance
            int startIdx = max(0, (int)(t * splinePoints->Vtx().size()) - 10);
            int endIdx = min((int)splinePoints->Vtx().size(), startIdx + 20);

            for (int i = startIdx; i < endIdx; i++) {
                float dist = length(newPosition - splinePoints->Vtx()[i]);
                if (dist < closestDistance) {
                    closestDistance = dist;
                    collisionNormal = normalize(newPosition - splinePoints->Vtx()[i]);
                }
            }

            if (closestDistance < wheelRadius * 1.1f) {
                // Collision response
                wheelPosition = splinePoints->Vtx()[startIdx] + collisionNormal * wheelRadius * 1.1f;
                fallVelocity = reflectVector(fallVelocity, collisionNormal) * 0.7f; // 30% energy loss
            }
            else {
                // Normal falling motion
                wheelPosition = newPosition;
                fallVelocity += gravity * deltaTime;
            }

            // Continue rotation during fall
            wheelAngle -= length(fallVelocity) * deltaTime / wheelRadius;

            // Ground collision (y = -1 is ground level in normalized coords)
            if (wheelPosition.y - wheelRadius <= -1.0f) {
                wheelPosition.y = -1.0f + wheelRadius;
                isSimulating = false;
            }
        }

        refreshScreen();
    }
};
VasutApp app;