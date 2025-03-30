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
const float g = 10.0f; // Reduced gravity (40 -> 10)
const float wheelRadius = 0.05f;
const float dt = 0.01f;
const float lambda = 1.0f;
const float minSpeed = 0.1f; // Reduced minimum speed (0.5 -> 0.1)
const float initialPush = 0.3f; // Reduced initial push (1.0 -> 0.3)
const float friction = 0.2f; // Increased friction (0.1 -> 0.2)
const float speedFactor = 0.5f; // Global speed reduction

class VasutApp : public glApp {
    Geometry<vec2>* controlPoints;
    Geometry<vec2>* splinePoints;
    Geometry<vec2>* wheel;
    std::vector<vec2> pointList;
    GPUProgram* gpuProgram;

    // Simulation variables
    bool isSimulating = false;
    bool isFalling = false;
    float t = 0.0f; // spline parameter
    float speed = 0.0f;
    float wheelAngle = 0.0f;
    vec2 fallVelocity;
    vec2 wheelPosition = vec2(0, 0);
    vec2 wheelTangent = vec2(1, 0);
    vec2 wheelNormal = vec2(0, 1);
    float initialHeight = 0.0f;

public:
    VasutApp() : glApp("Roller Coaster Simulation") {}

    void onInitialization() {
        controlPoints = new Geometry<vec2>();
        splinePoints = new Geometry<vec2>();
        wheel = new Geometry<vec2>();
        gpuProgram = new GPUProgram(vertSource, fragSource);
        glPointSize(10.0f);
        glLineWidth(3.0f);

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
        }

        // Spokes
        for (int i = 0; i < spokes; i++) {
            float angle = 2.0f * M_PI * i / spokes;
            wheel->Vtx().push_back(vec2(0, 0));
            vec2 spokeEnd(wheelRadius * cos(angle), wheelRadius * sin(angle));
            wheel->Vtx().push_back(spokeEnd);
        }

        wheel->updateGPU();
    }

    vec2 calculateSplinePoint(float t) {
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

        return 0.5f * (
            (4.0f * p0 - 10.0f * p1 + 8.0f * p2 - 2.0f * p3) +
            2.0f * (-3.0f * p0 + 9.0f * p1 - 9.0f * p2 + 3.0f * p3) * localT
            );
    }

    float calculateCurvature(float t) {
        vec2 r_t = calculateSplineDerivative(t);
        vec2 r_tt = calculateSplineSecondDerivative(t);
        float r_t_length = length(r_t);

        if (r_t_length < 0.0001f) return 0.0f;

        float numerator = fabs(r_t.x * r_tt.y - r_t.y * r_tt.x);
        float denominator = pow(r_t_length, 3);
        return numerator / denominator;
    }

    void updateSplineGeometry() {
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

        // Draw filled blue wheel
        Geometry<vec2> wheelFill;
        wheelFill.Vtx().push_back(wheelPosition);
        const int segments = 36;
        for (int i = 0; i <= segments; i++) {
            float angle = 2.0f * M_PI * i / segments;
            wheelFill.Vtx().push_back(vec2(
                wheelRadius * cos(angle) + wheelPosition.x,
                wheelRadius * sin(angle) + wheelPosition.y
            ));
        }
        wheelFill.updateGPU();
        wheelFill.Draw(gpuProgram, GL_TRIANGLE_FAN, vec3(0, 0, 1));

        // Draw white outline
        Geometry<vec2> wheelOutline;
        for (int i = 0; i <= segments; i++) {
            float angle = 2.0f * M_PI * i / segments;
            wheelOutline.Vtx().push_back(vec2(
                wheelRadius * cos(angle) + wheelPosition.x,
                wheelRadius * sin(angle) + wheelPosition.y
            ));
        }
        wheelOutline.updateGPU();
        glLineWidth(1.5f);
        wheelOutline.Draw(gpuProgram, GL_LINE_LOOP, vec3(1, 1, 1));
        glLineWidth(1.0f);

        // Draw spokes
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
        wheelSpokes.Draw(gpuProgram, GL_LINES, vec3(1, 1, 1));
    }

    void onDisplay() {
        glClearColor(0, 0, 0, 0);
        glClear(GL_COLOR_BUFFER_BIT);

        if (pointList.size() > 1) {
            splinePoints->Draw(gpuProgram, GL_LINE_STRIP, vec3(1, 1, 0));
        }

        if (isSimulating) {
            drawWheel();
        }

        controlPoints->Draw(gpuProgram, GL_POINTS, vec3(1, 0, 0));
    }

    void onMousePressed(MouseButton button, int pX, int pY) {
        if (button == MOUSE_LEFT && !isSimulating) {
            float x = 2.0f * pX / winWidth - 1;
            float y = 1 - 2.0f * pY / winHeight;
            pointList.push_back(vec2(x, y));
            controlPoints->Vtx().push_back(vec2(x, y));
            controlPoints->updateGPU();
            updateSplineGeometry();
        }
    }

    void onKeyboard(int key) {
        if (key == ' ' && pointList.size() >= 2 && !isSimulating) {
            isSimulating = true;
            isFalling = false;
            t = 0.0f;
            speed = initialPush;
            wheelAngle = 0.0f;

            vec2 splinePoint = calculateSplinePoint(t);
            initialHeight = splinePoint.y;
            vec2 derivative = calculateSplineDerivative(t);

            if (length(derivative) > 0) {
                wheelTangent = normalize(derivative);
                // Calculate initial normal - always point "up" relative to movement direction
                wheelNormal = vec2(-wheelTangent.y, wheelTangent.x);
                if (wheelNormal.y < 0) {
                    wheelNormal = -wheelNormal;
                }

                wheelPosition = splinePoint + wheelNormal * wheelRadius;
            }
        }
    }

    void onTimeElapsed(float startTime, float endTime) {
        if (!isSimulating) return;

        float deltaTime = endTime - startTime;
        if (deltaTime <= 0) return;

        const float physicsStep = 0.01f;
        int steps = 0;

        while (deltaTime > 0 && steps++ < 100) {
            float dt = std::min(physicsStep, deltaTime);
            deltaTime -= dt;

            if (isFalling) {
                // Free fall physics
                fallVelocity += vec2(0, -g) * dt;
                wheelPosition += fallVelocity * dt;
                wheelAngle -= 0.5f * length(fallVelocity) * dt / wheelRadius;

                // Check if out of bounds
                if (wheelPosition.x < -1 || wheelPosition.x > 1 || wheelPosition.y < -1) {
                    isSimulating = false;
                    break;
                }

                // Only check for collision if moving downward (prevent sticking at crossings)
                if (fallVelocity.y < 0) {
                    // Find closest point on track
                    float closestDistSq = std::numeric_limits<float>::max();
                    float closestT = t; // Start searching near current t
                    const int searchSteps = 20;
                    const float searchRange = 0.1f; // Search ±10% of track length

                    for (int i = 0; i <= searchSteps; i++) {
                        float testT = t + searchRange * (2.0f * i / searchSteps - 1.0f);
                        testT = std::max(0.0f, std::min(1.0f, testT));
                        vec2 point = calculateSplinePoint(testT);
                        float distSq = dot(wheelPosition - point, wheelPosition - point);

                        if (distSq < closestDistSq) {
                            closestDistSq = distSq;
                            closestT = testT;
                        }
                    }

                    // Only reattach if close enough and moving toward track
                    if (closestDistSq < wheelRadius * wheelRadius * 1.1f) {
                        vec2 closestPoint = calculateSplinePoint(closestT);
                        vec2 toWheel = wheelPosition - closestPoint;
                        if (dot(toWheel, fallVelocity) < 0) { // Moving toward track
                            t = closestT;
                            vec2 derivative = calculateSplineDerivative(t);

                            if (length(derivative) > 0) {
                                wheelTangent = normalize(derivative);
                                wheelNormal = vec2(-wheelTangent.y, wheelTangent.x);
                                if (wheelNormal.y < 0) wheelNormal = -wheelNormal;

                                // Only reattach if we have enough momentum
                                if (length(fallVelocity) > minSpeed * 0.5f) {
                                    speed = length(fallVelocity);
                                    isFalling = false;
                                    wheelPosition = closestPoint + wheelNormal * wheelRadius;
                                }
                            }
                        }
                    }
                }
            }
            else {
                // On-track physics (same as before)
                vec2 splinePoint = calculateSplinePoint(t);
                vec2 derivative = calculateSplineDerivative(t);
                float tangentLength = length(derivative);

                if (tangentLength > 0.0001f) {
                    // Calculate tangent and maintain consistent normal
                    vec2 newTangent = normalize(derivative);
                    vec2 potentialNormal = vec2(-newTangent.y, newTangent.x);

                    if (dot(potentialNormal, wheelNormal) < 0) {
                        potentialNormal = -potentialNormal;
                    }

                    if (length(newTangent - wheelTangent) > 0.01f) {
                        wheelTangent = newTangent;
                        wheelNormal = potentialNormal;
                    }

                    // PROPER SLOPE PHYSICS - FIXED HERE
                    // Calculate slope angle (angle between tangent and horizontal)
                    float slopeAngle = atan2(wheelTangent.y, wheelTangent.x);
                    // Acceleration due to gravity along the slope
                    float slopeAcceleration = -g * sin(slopeAngle); // Negative when going uphill

                    // Apply friction (always opposes motion)
                    float frictionAcceleration = -friction * (speed > 0 ? 1 : -1);

                    // Combined acceleration
                    float totalAcceleration = slopeAcceleration + frictionAcceleration;

                    // Update speed
                    speed += totalAcceleration * dt;
                    speed = std::max(speed, minSpeed);

                    // Calculate curvature and required centripetal force
                    float curvature = calculateCurvature(t);
                    float requiredCentripetal = speed * speed * curvature;
                    float gDotN = dot(vec2(0, -g), wheelNormal);
                    float availableForce = gDotN + requiredCentripetal;

                    // Only detach if physics requires AND we have some speed
                    if (availableForce < 0 && speed > minSpeed * 0.5f) {
                        isFalling = true;
                        fallVelocity = speed * wheelTangent;
                        continue;
                    }

                    // Update position parameter
                    float deltaT = (speed * dt) / tangentLength;
                    t += deltaT;

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
                    if (speed == 0)
                    {
                        isSimulating = false;
                        return;
                    }

                    // Update wheel position and rotation
                    splinePoint = calculateSplinePoint(t);
                    wheelPosition = splinePoint + wheelNormal * wheelRadius;
                    wheelAngle -= 0.5f * speed * dt / wheelRadius;
                }
                else {
                    isSimulating = false;
                }
            }
        }

        refreshScreen();
    }
};

VasutApp app;