// Cloth simulation based on the lecture slides of Prof. Dan
// written by: Zhou Zhuang

String windowTitle = "Cloth Simulation";

//Simulation Parameters
float floor = 500;
PVector gravity = new PVector(0,400,0);
float radius = 5;
PVector stringTop = new PVector(200,50,0);
float restLen = 10;
float mass = 1.0;
float ks = 2000;
float kd = 30;
float kfric = 0.5;
float ksphere = 2000;
float kdamp = 0.995;

float zoom = 0.75;

// Positions and velocities of masses
static int nx = 50;
static int ny = 42;
PVector pos[][] = new PVector[nx][ny];
PVector vel[][] = new PVector[nx][ny];
PVector acc[][] = new PVector[nx][ny];

// sphere position and radius
PVector spherePos = new PVector(250,375,235);
float sphereRadius = 100;

PImage img;

//Initialize the cloth
void setup() {
    size(400, 500, P3D);
    surface.setTitle(windowTitle);
    frameRate(30);
    img = loadImage("cloth.jpg");
    for (int i = 0; i < nx; i++) {
        for (int j = 0; j < ny; j++) {
            pos[i][j] = new PVector(i * restLen, j * restLen, j * restLen);
            vel[i][j] = new PVector(0, 0, 0);
            acc[i][j] = new PVector(0, 0, 0);
        }
    }
}

void update(float dt) {
    // Initialize the accelerations
    for (int i = 0; i < nx; i++) {
        for (int j = 0; j < ny; j++) {
            acc[i][j] = new PVector(0, 0, 0);
            acc[i][j].add(gravity);
        }
    }

    // Apply spring forces vertically
    for (int i = 0; i < nx; i++) {
        for (int j = 0; j < ny-1; j++) {
            PVector e = PVector.sub(pos[i][j + 1], pos[i][j]);
            float len = e.mag();
            e.normalize();
            float v1 = vel[i][j].dot(e);
            float v2 = vel[i][j + 1].dot(e);
            float f = -ks * (restLen - len) - kd * (v1 - v2);
            acc[i][j].add(PVector.mult(e, f));
            acc[i][j + 1].sub(PVector.mult(e, f));
        }
    }

    // Apply spring forces horizontally
    for (int i = 0; i < nx-1; i++) {
        for (int j = 0; j < ny; j++) {
            PVector e = PVector.sub(pos[i + 1][j], pos[i][j]);
            float len = e.mag();
            e.normalize();
            float v1 = vel[i][j].dot(e);
            float v2 = vel[i + 1][j].dot(e);
            float f = -ks * (restLen - len) - kd * (v1 - v2);
            acc[i][j].add(PVector.mult(e, f));
            acc[i + 1][j].sub(PVector.mult(e, f));
        }
    }


    // Separation forces is too time consuming! Removed.

    // Stop the cloth passing through the sphere
    for (int i = 0; i < nx; i++) {
        for (int j = 1; j < ny; j++) {
            PVector e = PVector.sub(spherePos, pos[i][j]);
            float len = e.mag();
            if (len < sphereRadius) {
                pos[i][j].sub(PVector.mult(e, (sphereRadius - len) / len));
                // reflect the velocity with the normal
                PVector n = PVector.mult(e, -1);
                n.normalize();
                vel[i][j].sub(PVector.mult(n, 2 * vel[i][j].dot(n)));
            }
        }
    }

    // Update velocities and positions
    for (int i = 0; i < nx; i++) {
        for (int j = 1; j < ny; j++) {
            vel[i][j].add(PVector.mult(acc[i][j], dt));
            pos[i][j].add(PVector.mult(vel[i][j], dt));
        }
    }

    // Collision with floor, simple bounce
    for (int i = 0; i < nx; i++) {
        for (int j = 0; j < ny; j++) {
            if (pos[i][j].y > floor) {
                pos[i][j].y = floor;
                vel[i][j].y *= -0.9;
            }
        }
    }

    // Apply damping
    for (int i = 0; i < nx; i++) {
        for (int j = 0; j < ny; j++) {
            vel[i][j].mult(kdamp);
        }
    }

}


void draw() {
    background(255, 255, 255);
    fill(100, 100, 100);
    stroke(0);
    strokeWeight(0);
    for (int i = 0; i < 30; i++) {
      update(1/(10*frameRate));
    }
    // Set camera position using mouse
    camera(mouseX*2 - width, mouseY*2 - height, (height / 2) / tan(PI / 6), width / 2, height / 2, 0, 0, 1, 0);
    
    // Set zoom
    scale(zoom);

    // Move the shpere using arrow keys
    if (keyPressed) {
        if (key == 'w') {
            spherePos.y -= 5;
        }
        if (key == 's') {
            spherePos.y += 5;
        }
        if (key == 'a') {
            spherePos.x -= 5;
        }
        if (key == 'd') {
            spherePos.x += 5;
        }
        if (key == 'q') {
            spherePos.z -= 5;
        }
        if (key == 'e') {
            spherePos.z += 5;
        }
        if (key == 'r') {
            setup();
        }
    }

    


    // Render the cloth using triangle strips
    for (int i = 0; i < nx - 1; i++) {
        beginShape(TRIANGLE_STRIP);
        // Set texture for the cloth
        texture(img);
        textureMode(NORMAL);
        for (int j = 0; j < ny; j++) {
            // set vertex with texture coordinates
            float texScale = 1.0f / nx;
            vertex(pos[i][j].x, pos[i][j].y, pos[i][j].z, i * texScale, j * texScale);
            vertex(pos[i + 1][j].x, pos[i + 1][j].y, pos[i + 1][j].z, (i + 1) * texScale, j * texScale);
        }
        endShape();
    }

    // Render a sphere
    pushMatrix();
    translate(spherePos.x, spherePos.y, spherePos.z);
    // Set the sphere to be slightly transparent
    fill(255, 255, 255, 100);
    strokeWeight(1);
    sphere(sphereRadius-10);
    popMatrix();

    // Print sphere position to terminal
    println("Sphere position:" + spherePos.x + " " + spherePos.y + " " + spherePos.z);

    // Add lights
    ambientLight(50, 50, 50);
    directionalLight(255, 255, 255, 0, 0, -1);
    pointLight(255, 255, 255, 0, 0, 0);

}

void mouseWheel(MouseEvent event) {
    float e = event.getCount();
    // clamp the zoom
    if (e > 0 && zoom < 1) {
        zoom += 0.01;
    } else if (e < 0 && zoom > 0.1) {
        zoom -= 0.01;
    }
}
