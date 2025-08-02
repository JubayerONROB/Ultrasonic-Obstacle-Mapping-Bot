#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <Servo.h>
#include <ArduinoJson.h>
#include <math.h>

// Motor pins
#define ENA D8    // Enable PWM for Motor A
#define IN1 D3    // Motor A direction 1
#define IN2 D4    // Motor A direction 2
#define IN3 D1    // Motor B direction 1
#define IN4 D2    // Motor B direction 2
#define ENB D7    // Enable PWM for Motor B

// Encoder pins
#define LEFT_ENCODER_PIN D10   // GPIO10 (SD3)
#define RIGHT_ENCODER_PIN D9   // GPIO9 (SD2)

// Ultrasonic sensor pins
#define TRIG_PIN D5
#define ECHO_PIN D6

// Servo pin
#define SERVO_PIN D0

// Access Point credentials
const char* ap_ssid = "RobotAP";
const char* ap_password = "robot1234";

// Create servo and web server objects
Servo servo;
ESP8266WebServer server(80);

// Variables for distance measurement
int frontDistance = 0;
int leftDistance = 0;
int rightDistance = 0;
String motorState = "Initializing";

// Encoder variables
volatile unsigned long leftPulseCount = 0;
volatile unsigned long rightPulseCount = 0;
unsigned long lastLeftPulseCount = 0;
unsigned long lastRightPulseCount = 0;
unsigned long lastEncoderTime = 0;
float leftVelocity = 0;
float rightVelocity = 0;

// Robot physical parameters (calibrate these for your robot)
const float wheelDiameter = 6.5;        // cm
const float wheelCircumference = PI * wheelDiameter;
const int pulsesPerRevolution = 20;     // Encoder pulses per wheel revolution
const float cm_per_pulse = wheelCircumference / pulsesPerRevolution;
const float wheelBase = 15.0;           // Distance between wheels (cm)

// Speed control
const int baseSpeed = 100;              // Base PWM speed (0-255)
const int turnSpeed = 80;              // Turning speed
const int reverseSpeed = 80;           // Reverse speed

// Mapping variables
float robotX = 0;           // Current X position (cm)
float robotY = 0;           // Current Y position (cm)
float robotTheta = 0;       // Current orientation (radians)

// Map storage
const int maxMapPoints = 500;
float mapPointsX[maxMapPoints];
float mapPointsY[maxMapPoints];
int mapPointCount = 0;

// Threshold distances (cm)
const int obstacleDistance = 15;        // Stop and avoid if closer than this
const int safeDistance = 40;            // Comfortable distance to maintain

// Scan parameters
const int numAngles = 7;
int angles[numAngles] = {0, 30, 60, 90, 120, 150, 180};
float distances[numAngles];

// Scan state machine
enum ScanState { SCAN_IDLE, SCAN_MOVE_SERVO, SCAN_SETTLE, SCAN_TAKE_READING, SCAN_WAIT_BETWEEN };
ScanState scanState = SCAN_IDLE;
int currentAngleIndex = 0;
int currentReading = 0;
long totalDistance = 0;
unsigned long lastActionTime = 0;

// Movement state machine
enum MoveState { MOVE_NONE, MOVE_FORWARD, MOVE_STOP, MOVE_BACK, MOVE_TURN_LEFT, MOVE_TURN_RIGHT };
MoveState currentMoveState = MOVE_STOP;
MoveState nextMoveState = MOVE_NONE;
MoveState afterNextMoveState = MOVE_NONE;
unsigned long moveStateStart = 0;
unsigned long moveStateDuration = 0;
unsigned long nextMoveDuration = 0;
unsigned long afterNextMoveDuration = 0;

// HTML and JavaScript for the web interface with zoomable map
const char* htmlContent = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
    <title>Robot Control Panel</title>
    <meta name="viewport" content="width=device-width, initial-scale=1">
    <style>
        body {
            font-family: Arial, sans-serif;
            text-align: center;
            margin: 0;
            padding: 20px;
            background-color: #f5f5f5;
        }
        .container {
            max-width: 1000px;
            margin: 0 auto;
            background: white;
            padding: 20px;
            border-radius: 10px;
            box-shadow: 0 0 10px rgba(0,0,0,0.1);
        }
        .dashboard {
            display: flex;
            justify-content: space-around;
            flex-wrap: wrap;
            margin: 20px 0;
        }
        .sensor-box {
            background: #e3f2fd;
            padding: 15px;
            border-radius: 8px;
            margin: 10px;
            min-width: 120px;
        }
        .sensor-value {
            font-size: 24px;
            font-weight: bold;
            color: #0d47a1;
        }
        .status {
            padding: 15px;
            margin: 20px 0;
            border-radius: 5px;
            font-size: 18px;
            background: #e8f5e9;
        }
        .status.avoiding {
            background: #ffebee;
        }
        .status.moving {
            background: #e8f5e9;
        }
        .status.turning {
            background: #fff8e1;
        }
        .map-container {
            margin: 20px 0;
            border: 1px solid #ddd;
            border-radius: 5px;
            overflow: hidden;
            position: relative;
        }
        #environmentMap {
            background-color: #f9f9f9;
            cursor: move;
        }
        .zoom-controls {
            position: absolute;
            top: 10px;
            right: 10px;
            z-index: 100;
        }
        .zoom-btn {
            background: white;
            border: 1px solid #ddd;
            border-radius: 4px;
            width: 30px;
            height: 30px;
            font-size: 20px;
            cursor: pointer;
            margin-bottom: 5px;
            display: block;
        }
        @media (max-width: 600px) {
            .dashboard {
                flex-direction: column;
            }
            #environmentMap {
                width: 100%;
                height: auto;
            }
        }
    </style>
</head>
<body>
    <div class="container">
        <h1>Robot Control Panel</h1>
        
        <div class="dashboard">
            <div class="sensor-box">
                <h3>Front Distance</h3>
                <div class="sensor-value" id="frontDist">-- cm</div>
            </div>
            <div class="sensor-box">
                <h3>Left Distance</h3>
                <div class="sensor-value" id="leftDist">-- cm</div>
            </div>
            <div class="sensor-box">
                <h3>Right Distance</h3>
                <div class="sensor-value" id="rightDist">-- cm</div>
            </div>
        </div>
        
        <div class="dashboard">
            <div class="sensor-box">
                <h3>Left Wheel Speed</h3>
                <div class="sensor-value" id="leftSpeed">-- cm/s</div>
            </div>
            <div class="sensor-box">
                <h3>Right Wheel Speed</h3>
                <div class="sensor-value" id="rightSpeed">-- cm/s</div>
            </div>
            <div class="sensor-box">
                <h3>Position (X,Y)</h3>
                <div class="sensor-value" id="position">(0, 0)</div>
            </div>
        </div>
        
        <div class="status" id="statusDisplay">
            Connecting to robot...
        </div>
        
        <div class="map-container">
            <h3>Environment Map</h3>
            <canvas id="environmentMap" width="800" height="600"></canvas>
            <div class="zoom-controls">
                <button class="zoom-btn" id="zoomIn">+</button>
                <button class="zoom-btn" id="zoomOut">-</button>
                <button class="zoom-btn" id="resetView">⟲</button>
            </div>
        </div>
        
        <div class="connection-info">
            <p>Connected to: <span id="ipAddress">%IP%</span></p>
            <p>Signal strength: <span id="rssi">--</span> dBm</p>
        </div>
    </div>

    <script>
        const canvas = document.getElementById('environmentMap');
        const ctx = canvas.getContext('2d');
        
        // Map view state
        let viewState = {
            scale: 2,           // Pixels per cm
            offsetX: 0,         // View offset in cm
            offsetY: 0,         // View offset in cm
            isDragging: false,
            lastX: 0,
            lastY: 0
        };
        
        // Tracked robot state
        let robotX = 0, robotY = 0, robotTheta = 0;
        let mapPoints = [];
        
        // Convert world coordinates to canvas coordinates
        function worldToCanvas(x, y) {
            return {
                x: canvas.width/2 + (x - viewState.offsetX) * viewState.scale,
                y: canvas.height/2 - (y - viewState.offsetY) * viewState.scale
            };
        }
        
        // Center view on robot
        function centerOnRobot() {
            viewState.offsetX = robotX;
            viewState.offsetY = robotY;
            drawMap();
        }
        
        // Draw the map
        function drawMap() {
            // Clear canvas
            ctx.clearRect(0, 0, canvas.width, canvas.height);
            
            // Draw grid (adjust spacing based on zoom level)
            const gridSpacing = Math.max(10, Math.floor(50 / viewState.scale)) * viewState.scale;
            const gridSize = gridSpacing * Math.ceil(1 / viewState.scale) * 5;
            
            ctx.strokeStyle = '#ddd';
            ctx.lineWidth = 1;
            
            // Calculate visible area
            const visibleLeft = viewState.offsetX - canvas.width/(2*viewState.scale);
            const visibleRight = viewState.offsetX + canvas.width/(2*viewState.scale);
            const visibleTop = viewState.offsetY + canvas.height/(2*viewState.scale);
            const visibleBottom = viewState.offsetY - canvas.height/(2*viewState.scale);
            
            // Draw vertical grid lines
            const startX = Math.floor(visibleLeft / gridSize) * gridSize;
            for (let x = startX; x <= visibleRight; x += gridSize) {
                const canvasX = worldToCanvas(x, 0).x;
                ctx.beginPath();
                ctx.moveTo(canvasX, 0);
                ctx.lineTo(canvasX, canvas.height);
                ctx.stroke();
            }
            
            // Draw horizontal grid lines
            const startY = Math.floor(visibleBottom / gridSize) * gridSize;
            for (let y = startY; y <= visibleTop; y += gridSize) {
                const canvasY = worldToCanvas(0, y).y;
                ctx.beginPath();
                ctx.moveTo(0, canvasY);
                ctx.lineTo(canvas.width, canvasY);
                ctx.stroke();
            }
            
            // Draw axes
            ctx.strokeStyle = '#999';
            ctx.lineWidth = 2;
            
            // X axis
            let origin = worldToCanvas(0, 0);
            ctx.beginPath();
            ctx.moveTo(0, origin.y);
            ctx.lineTo(canvas.width, origin.y);
            
            // Y axis
            ctx.moveTo(origin.x, 0);
            ctx.lineTo(origin.x, canvas.height);
            ctx.stroke();
            
            // Draw obstacles (map points)
            ctx.fillStyle = 'rgba(255, 0, 0, 0.7)';
            for (let i = 0; i < mapPoints.length; i++) {
                const point = worldToCanvas(mapPoints[i].x, mapPoints[i].y);
                ctx.beginPath();
                ctx.arc(point.x, point.y, 3, 0, Math.PI * 2);
                ctx.fill();
            }
            
            // Draw robot position
            const robotPos = worldToCanvas(robotX, robotY);
            
            // Draw robot body
            ctx.fillStyle = 'blue';
            ctx.beginPath();
            ctx.arc(robotPos.x, robotPos.y, 8, 0, Math.PI * 2);
            ctx.fill();
            
            // Draw robot orientation
            ctx.strokeStyle = 'blue';
            ctx.lineWidth = 3;
            ctx.beginPath();
            ctx.moveTo(robotPos.x, robotPos.y);
            ctx.lineTo(
                robotPos.x + 20 * Math.cos(robotTheta) * viewState.scale,
                robotPos.y - 20 * Math.sin(robotTheta) * viewState.scale
            );
            ctx.stroke();
            
            // Draw robot center marker
            ctx.strokeStyle = 'darkblue';
            ctx.lineWidth = 2;
            ctx.beginPath();
            ctx.arc(robotPos.x, robotPos.y, 6, 0, Math.PI * 2);
            ctx.stroke();
        }
        
        // Handle mouse events for panning
        canvas.addEventListener('mousedown', (e) => {
            viewState.isDragging = true;
            viewState.lastX = e.clientX;
            viewState.lastY = e.clientY;
        });
        
        canvas.addEventListener('mousemove', (e) => {
            if (viewState.isDragging) {
                const dx = (e.clientX - viewState.lastX) / viewState.scale;
                const dy = (e.clientY - viewState.lastY) / viewState.scale;
                
                viewState.offsetX -= dx;
                viewState.offsetY += dy;
                viewState.lastX = e.clientX;
                viewState.lastY = e.clientY;
                
                drawMap();
            }
        });
        
        canvas.addEventListener('mouseup', () => {
            viewState.isDragging = false;
        });
        
        canvas.addEventListener('mouseleave', () => {
            viewState.isDragging = false;
        });
        
        // Handle zoom controls
        document.getElementById('zoomIn').addEventListener('click', () => {
            viewState.scale *= 1.2;
            drawMap();
        });
        
        document.getElementById('zoomOut').addEventListener('click', () => {
            viewState.scale /= 1.2;
            drawMap();
        });
        
        document.getElementById('resetView').addEventListener('click', () => {
            centerOnRobot();
        });
        
        function updateData() {
            fetch('/data')
                .then(response => response.json())
                .then(data => {
                    // Update sensor displays
                    document.getElementById('frontDist').textContent = data.frontDistance + ' cm';
                    document.getElementById('leftDist').textContent = data.leftDistance + ' cm';
                    document.getElementById('rightDist').textContent = data.rightDistance + ' cm';
                    document.getElementById('leftSpeed').textContent = data.leftVelocity.toFixed(2) + ' cm/s';
                    document.getElementById('rightSpeed').textContent = data.rightVelocity.toFixed(2) + ' cm/s';
                    document.getElementById('position').textContent = '(' + data.robotX.toFixed(1) + ', ' + data.robotY.toFixed(1) + ')';
                    
                    document.getElementById('statusDisplay').textContent = 'Status: ' + data.motorState;
                    
                    // Update status display class based on state
                    const statusDiv = document.getElementById('statusDisplay');
                    statusDiv.className = 'status';
                    if(data.motorState.includes('Avoiding')) {
                        statusDiv.classList.add('avoiding');
                    } else if(data.motorState.includes('Moving')) {
                        statusDiv.classList.add('moving');
                    } else if(data.motorState.includes('Turning')) {
                        statusDiv.classList.add('turning');
                    }
                    
                    document.getElementById('ipAddress').textContent = data.ip;
                    document.getElementById('rssi').textContent = data.rssi;
                    
                    // Update map data
                    if (data.mapPoints) {
                        mapPoints = data.mapPoints;
                        robotX = data.robotX;
                        robotY = data.robotY;
                        robotTheta = data.robotTheta;
                        
                        // Auto-center if robot is near edge of view
                        const robotCanvasPos = worldToCanvas(robotX, robotY);
                        const margin = 100; // pixels from edge to trigger recenter
                        
                        if (robotCanvasPos.x < margin || 
                            robotCanvasPos.x > canvas.width - margin ||
                            robotCanvasPos.y < margin || 
                            robotCanvasPos.y > canvas.height - margin) {
                            centerOnRobot();
                        } else {
                            drawMap();
                        }
                    }
                })
                .catch(error => {
                    console.error('Error fetching data:', error);
                });
        }
        
        // Initialize
        setInterval(updateData, 1000);
        updateData();
        centerOnRobot(); // Start centered on robot
    </script>
</body>
</html>
)rawliteral";

// Encoder interrupt service routines
void ICACHE_RAM_ATTR countLeftPulse() {
  leftPulseCount++;
}

void ICACHE_RAM_ATTR countRightPulse() {
  rightPulseCount++;
}

void setup() {
  // Initialize serial communication
  Serial.begin(115200);
  delay(1000);
  
  // Initialize motor control pins
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENB, OUTPUT);
  
  // Initialize ultrasonic sensor pins
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  // Initialize encoder pins
  pinMode(LEFT_ENCODER_PIN, INPUT_PULLUP);
  pinMode(RIGHT_ENCODER_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(LEFT_ENCODER_PIN), countLeftPulse, RISING);
  attachInterrupt(digitalPinToInterrupt(RIGHT_ENCODER_PIN), countRightPulse, RISING);
  
  // Attach servo
  servo.attach(SERVO_PIN);
  servo.write(90); // Center position
  delay(1000);
  
  // Create Access Point
  WiFi.softAP(ap_ssid, ap_password);
  
  Serial.println();
  Serial.print("Access Point \"");
  Serial.print(ap_ssid);
  Serial.println("\" started");
  
  Serial.print("IP address: ");
  Serial.println(WiFi.softAPIP());
  
  // Set up web server routes
  server.on("/", handleRoot);
  server.on("/data", handleData);
  server.begin();
  Serial.println("HTTP server started");

  // Initialize encoder timing
  lastEncoderTime = millis();

  // Start with an initial scan
  scanState = SCAN_MOVE_SERVO;
  motorState = "Initial scan";
  currentMoveState = MOVE_STOP;
}

void loop() {
  server.handleClient(); // Handle web requests

  // Update odometry
  updateOdometry();

  // Handle scanning
  handleScan();

  // Handle movement state
  handleMoveState();

  // Simple front obstacle detection when not scanning
  if (scanState == SCAN_IDLE && currentMoveState == MOVE_FORWARD) {
    checkFrontObstacle();
  }
}

void updateOdometry() {
  unsigned long currentTime = millis();
  if (currentTime - lastEncoderTime >= 100) { // Update every 100ms
    unsigned long currentLeftPulses = leftPulseCount;
    unsigned long currentRightPulses = rightPulseCount;
    
    unsigned long deltaLeftPulses = currentLeftPulses - lastLeftPulseCount;
    unsigned long deltaRightPulses = currentRightPulses - lastRightPulseCount;
    
    float timeElapsed = (currentTime - lastEncoderTime) / 1000.0; // in seconds
    
    // Calculate velocities (cm/s)
    leftVelocity = deltaLeftPulses * cm_per_pulse / timeElapsed;
    rightVelocity = deltaRightPulses * cm_per_pulse / timeElapsed;
    
    // Update position estimate (odometry)
    float deltaLeft = deltaLeftPulses * cm_per_pulse;
    float deltaRight = deltaRightPulses * cm_per_pulse;
    float deltaDistance = (deltaLeft + deltaRight) / 2.0;
    float deltaTheta = (deltaRight - deltaLeft) / wheelBase;
    
    robotTheta += deltaTheta;
    robotX += deltaDistance * cos(robotTheta);
    robotY += deltaDistance * sin(robotTheta);
    
    // Normalize theta to -PI..PI range
    while (robotTheta > PI) robotTheta -= 2 * PI;
    while (robotTheta < -PI) robotTheta += 2 * PI;
    
    lastLeftPulseCount = currentLeftPulses;
    lastRightPulseCount = currentRightPulses;
    lastEncoderTime = currentTime;
  }
}

void checkFrontObstacle() {
  // Simple front distance check without servo movement
  long durationSum = 0;
  int validReadings = 0;
  
  for (int i = 0; i < 3; i++) { // Take 3 readings and average
    digitalWrite(TRIG_PIN, LOW);
    delayMicroseconds(2);
    digitalWrite(TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG_PIN, LOW);
    
    long duration = pulseIn(ECHO_PIN, HIGH, 30000); // Timeout for ~500cm
    if (duration > 0) {
      durationSum += duration;
      validReadings++;
    }
    delay(10);
  }
  
  if (validReadings > 0) {
    long avgDuration = durationSum / validReadings;
    frontDistance = avgDuration * 0.034 / 2;
    
    if (frontDistance > 400) frontDistance = 400;
    if (frontDistance < 2) frontDistance = 2;
    
    if (frontDistance <= obstacleDistance) {
      // Obstacle detected - stop and start scanning
      currentMoveState = MOVE_STOP;
      moveStateStart = millis();
      moveStateDuration = 500; // Stop for 500ms
      nextMoveState = MOVE_NONE;
      scanState = SCAN_MOVE_SERVO; // Start scanning
      motorState = "Obstacle detected - scanning";
    }
  }
}

void handleScan() {
  unsigned long now = millis();

  switch (scanState) {
    case SCAN_IDLE:
      // Do nothing - waiting for obstacle detection
      break;

    case SCAN_MOVE_SERVO:
        // Stop the robot before moving servo
      currentMoveState = MOVE_STOP;
      moveStateStart = millis();
      moveStateDuration = 0; // Stop indefinitely while scanning
      nextMoveState = MOVE_NONE;
      servo.write(angles[currentAngleIndex]);
      lastActionTime = now;
      scanState = SCAN_SETTLE;
      motorState = "Scanning at " + String(angles[currentAngleIndex]) + "°";
      break;

    case SCAN_SETTLE:
      if (now - lastActionTime >= 300) { // Increased settling time for more accurate readings
        currentReading = 0;
        totalDistance = 0;
        scanState = SCAN_TAKE_READING;
      }
      break;

    case SCAN_TAKE_READING:
      if (currentReading < 5) {
        // Take distance reading with improved reliability
        long durationSum = 0;
        int validReadings = 0;
        
        for (int i = 0; i < 3; i++) { // Take 3 readings and average
          digitalWrite(TRIG_PIN, LOW);
          delayMicroseconds(2);
          digitalWrite(TRIG_PIN, HIGH);
          delayMicroseconds(10);
          digitalWrite(TRIG_PIN, LOW);
          
          long duration = pulseIn(ECHO_PIN, HIGH, 30000); // Timeout for ~500cm
          if (duration > 0) {
            durationSum += duration;
            validReadings++;
          }
          delay(10);
        }
        
        if (validReadings > 0) {
          long avgDuration = durationSum / validReadings;
          int distance = avgDuration * 0.034 / 2;
          
          if (distance > 400 || distance < 2) {
            distance = 400; // Limit to max range
          }
          
          totalDistance += distance;
        } else {
          totalDistance += 400; // No valid readings, assume max distance
        }
        
        currentReading++;
        lastActionTime = now;
        scanState = SCAN_WAIT_BETWEEN;
      } else {
        // Process average of 5 readings
        float average = totalDistance / 5.0;
        distances[currentAngleIndex] = average;

        // Add to map if valid (not max distance)
        if (average < 400 && mapPointCount < maxMapPoints) {
          float sensorAngle = (angles[currentAngleIndex] - 90) * PI / 180.0; // Convert to radians
          float globalAngle = robotTheta + sensorAngle;
          mapPointsX[mapPointCount] = robotX + average * cos(globalAngle);
          mapPointsY[mapPointCount] = robotY + average * sin(globalAngle);
          mapPointCount++;
        }

        currentAngleIndex++;
        if (currentAngleIndex < numAngles) {
          scanState = SCAN_MOVE_SERVO;
        } else {
          // Scan complete, return to center and perform avoidance
          servo.write(90);
          scanState = SCAN_IDLE;
          performAvoidance();
        }
      }
      break;

    case SCAN_WAIT_BETWEEN:
      if (now - lastActionTime >= 150) { // Increased delay between readings
        scanState = SCAN_TAKE_READING;
      }
      break;
  }
}

void performAvoidance() {
  // Calculate min distances for sectors with better angle coverage
  float front_min = fmin(fmin(distances[2], distances[3]), distances[4]); // 60°, 90°, 120°
  float left_min = fmin(fmin(distances[4], distances[5]), distances[6]);   // 120°, 150°, 180°
  float right_min = fmin(fmin(distances[0], distances[1]), distances[2]);  // 0°, 30°, 60°
  
  // Update display distances
  frontDistance = (int)distances[3];  // 90° for front display
  leftDistance = (int)distances[6];   // 180° for left display
  rightDistance = (int)distances[0];  // 0° for right display

  // Decide avoidance strategy
  if (left_min > right_min && left_min > safeDistance) {
    // More space on left - turn left
    currentMoveState = MOVE_TURN_LEFT;
    moveStateStart = millis();
    moveStateDuration = 500; // Turn for 500ms
    nextMoveState = MOVE_FORWARD; // Then continue forward
    motorState = "Avoiding - turning left";
  } 
  else if (right_min > left_min && right_min > safeDistance) {
    // More space on right - turn right
    currentMoveState = MOVE_TURN_RIGHT;
    moveStateStart = millis();
    moveStateDuration = 500; // Turn for 500ms
    nextMoveState = MOVE_FORWARD; // Then continue forward
    motorState = "Avoiding - turning right";
  }
  else {
    // Both sides blocked - back up and turn right
    currentMoveState = MOVE_BACK;
    moveStateStart = millis();
    moveStateDuration = 1000; // Back for 1s
    nextMoveState = MOVE_TURN_RIGHT;
    nextMoveDuration = 800; // Then turn for 800ms
    afterNextMoveState = MOVE_FORWARD; // Then continue forward
    motorState = "Avoiding - backing up";
  }
  
  // Reset scan angle index for next time
  currentAngleIndex = 0;
}

void handleMoveState() {
  if (scanState != SCAN_IDLE && currentMoveState != MOVE_STOP) {
    currentMoveState = MOVE_STOP;
    moveStateStart = millis();
    moveStateDuration = 0;
    nextMoveState = MOVE_NONE;
  }
  unsigned long now = millis();

  // Check if current state duration has ended
  if (moveStateDuration > 0 && now - moveStateStart >= moveStateDuration) {
    if (nextMoveState != MOVE_NONE) {
      // Transition to next state in sequence
      currentMoveState = nextMoveState;
      moveStateStart = now;
      moveStateDuration = nextMoveDuration;
      nextMoveState = afterNextMoveState;
      nextMoveDuration = afterNextMoveDuration;
      afterNextMoveState = MOVE_NONE;
      afterNextMoveDuration = 0;
    } else {
      // Default to forward movement if no next state
      currentMoveState = MOVE_FORWARD;
      moveStateDuration = 0;
    }
  }

  // Apply motor controls based on current state
  switch (currentMoveState) {
    case MOVE_FORWARD:
      moveForward();
      motorState = "Moving forward";
      break;
    case MOVE_STOP:
      stopRobot();
      break;
    case MOVE_BACK:
      moveBackward();
      motorState = "Moving backward";
      break;
    case MOVE_TURN_LEFT:
      turnLeft();
      motorState = "Turning left";
      break;
    case MOVE_TURN_RIGHT:
      turnRight();
      motorState = "Turning right";
      break;
    default:
      stopRobot();
      break;
  }
}

// Web server handlers
void handleRoot() {
  String html = htmlContent;
  html.replace("%IP%", WiFi.softAPIP().toString());
  server.send(200, "text/html", html);
}

void handleData() {
  // Create JSON response
  DynamicJsonDocument doc(4096);
  
  doc["frontDistance"] = frontDistance;
  doc["leftDistance"] = leftDistance;
  doc["rightDistance"] = rightDistance;
  doc["leftVelocity"] = leftVelocity;
  doc["rightVelocity"] = rightVelocity;
  doc["motorState"] = motorState;
  doc["robotX"] = robotX;
  doc["robotY"] = robotY;
  doc["robotTheta"] = robotTheta;
  doc["ip"] = WiFi.softAPIP().toString();
  doc["rssi"] = WiFi.softAPgetStationNum();  // Number of connected stations
  
  // Add map points
  JsonArray mapPoints = doc.createNestedArray("mapPoints");
  for (int i = 0; i < mapPointCount; i++) {
    JsonObject point = mapPoints.createNestedObject();
    point["x"] = mapPointsX[i];
    point["y"] = mapPointsY[i];
  }
  
  String json;
  serializeJson(doc, json);
  server.send(200, "application/json", json);
}

// Motor control functions
void moveForward() {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  analogWrite(ENA, baseSpeed);
  analogWrite(ENB, baseSpeed);
}

void moveBackward() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  analogWrite(ENA, reverseSpeed);
  analogWrite(ENB, reverseSpeed);
}

void turnLeft() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  analogWrite(ENA, turnSpeed);
  analogWrite(ENB, turnSpeed);
}

void turnRight() {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  analogWrite(ENA, turnSpeed);
  analogWrite(ENB, turnSpeed);
}

void stopRobot() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
}