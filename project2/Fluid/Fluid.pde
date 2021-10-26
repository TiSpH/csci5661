// 2D SPH water simulation based on the lecture slides of Prof. Dan
// written by: Zhou Zhuang

// number of particles
int numParticles = 500;

// particle class
class Particle {
    public PVector pos;
    public PVector posOld;
    public PVector vel;
    public float dens;
    public float densN;
    public float press;
    public float pressN;
}

class Pair {
    public int i;
    public int j;
    public float q;
    public float q2;
    public float q3;
}

// array of particles
Particle[] particles;


// gravity
PVector gravity = new PVector(0, 50);

// stiffness
float k_stiff = 300;
float k_stiffN = 1000;

// rest density
float k_restDens = 20;

// smoothing radius
float smoothingRadius = 20;


void setup() {
    size(640, 360);
    smooth();
    frameRate(60);
    colorMode(RGB, 255, 255, 255, 100);
    noStroke();
    particles = new Particle[numParticles];
    for (int i = 0; i < numParticles; i++) {
        particles[i] = new Particle();
        particles[i].pos = new PVector(random(640), random(360));
        particles[i].posOld = particles[i].pos.copy();
        particles[i].vel = new PVector(0, 0);
        particles[i].dens = 0;
        particles[i].densN = 0;
        particles[i].press = 0;
        particles[i].pressN = 0;
    }
}


void update(float dt) {
    // init variables
    for (int i = 0; i < numParticles; i++) {
        particles[i].vel = PVector.sub(particles[i].pos, particles[i].posOld).mult(1 / dt);
        particles[i].posOld = particles[i].pos.copy();
        particles[i].vel.add(PVector.mult(gravity, dt));
        particles[i].pos.add(PVector.mult(particles[i].vel, dt));
        particles[i].dens = 0;
        particles[i].densN = 0;
    }

    // create a Pair List
    ArrayList<Pair> pairs = new ArrayList<Pair>();

    // add pairs to the list if the distance is smaller than the smoothing radius
    for (int i = 0; i < numParticles; i++) {
        for (int j = i + 1; j < numParticles; j++) {
            float dist = PVector.dist(particles[i].pos, particles[j].pos);
            if (dist < smoothingRadius) {
                Pair p = new Pair();
                p.i = i;
                p.j = j;
                p.q = 1 - dist / smoothingRadius;
                p.q2 = p.q * p.q;
                p.q3 = p.q2 * p.q;
                pairs.add(p);
                particles[i].dens += p.q2;
                particles[j].dens += p.q2;
                particles[i].densN += p.q3;
                particles[j].densN += p.q3;
            }
        }
    }

    // calculate pressure
    for (int i = 0; i < numParticles; i++) {
        particles[i].press = k_stiff * (particles[i].dens - k_restDens);
        particles[i].pressN = k_stiffN * particles[i].densN;
    }

    // move particles for each pair
    for (int i = 0; i < pairs.size(); i++) {
        Pair p = pairs.get(i);
        float press = particles[p.i].press + particles[p.j].press;
        float pressN = particles[p.i].pressN + particles[p.j].pressN;
        float displace = (press*p.q + pressN*p.q2) * (dt*dt);
        PVector dir = PVector.sub(particles[p.j].pos, particles[p.i].pos).normalize();
        particles[p.i].pos.add(PVector.mult(dir, displace));
        particles[p.j].pos.sub(PVector.mult(dir, displace));
    }

    // move particles randomly for a small amount
    for (int i = 0; i < numParticles; i++) {
        particles[i].pos.add(PVector.mult(PVector.random2D(), 0.01));
    }

    // bounce particles off the walls
    for (int i = 0; i < numParticles; i++) {
        if (particles[i].pos.x < 0) {
            particles[i].pos.x = -particles[i].pos.x;
            particles[i].vel.x = -particles[i].vel.x;
        }
        if (particles[i].pos.x > width) {
            particles[i].pos.x = 2 * width - particles[i].pos.x;
            particles[i].vel.x = -particles[i].vel.x;
        }
        if (particles[i].pos.y < 0) {
            particles[i].pos.y = -particles[i].pos.y;
            particles[i].vel.y = -particles[i].vel.y;
        }
        if (particles[i].pos.y > height) {
            particles[i].pos.y = 2 * height - particles[i].pos.y;
            particles[i].vel.y = -particles[i].vel.y;
        }
    }

}


void draw() {
    background(0);
    update(1 / 60.0);
    for (int i = 0; i < numParticles; i++) {
        // set color based on density
        int r = (int) (particles[i].dens * 255);
        int g = (int) (particles[i].dens * 255);
        int b = (int) (particles[i].dens * 255);
        fill(r, g, b, 100);
        ellipse(particles[i].pos.x, particles[i].pos.y, 10, 10);
    }

    // move particles to the mouse
    PVector mouse = new PVector(mouseX, mouseY);
    for (int i = 0; i < numParticles; i++) {
        PVector dir = PVector.sub(mouse, particles[i].pos).normalize();
        particles[i].pos.add(PVector.mult(dir, 0.1));
    }
}
