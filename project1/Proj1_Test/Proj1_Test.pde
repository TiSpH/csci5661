//CSCI 5611 - Graph Search & Planning
//PRM Sample Code [Proj 1]
//Instructor: Stephen J. Guy <sjguy@umn.edu>

//This is a test harness designed to help you test & debug your PRM.

//USAGE:
// On start-up your PRM will be tested on a random scene and the results printed
// Left clicking will set a red goal, right clicking the blue start
// The arrow keys will move the circular obstacle with the heavy outline
// Pressing 'r' will randomize the obstacles and re-run the tests

//Change the below parameters to change the scenario/roadmap size
int numObstacles = 100;
int numNodes  = 300;
float k_goal = 10;
float k_avoid = 1000;

//A list of circle obstacles
static int maxNumObstacles = 1000;
static int maxNumAgents = 20;
int numAgents = 8;
Vec2 circlePos[] = new Vec2[maxNumObstacles]; //Circle positions
float circleRad[] = new float[maxNumObstacles];  //Circle radii

Vec2 startPs[] = new Vec2[maxNumAgents];
Vec2 endPs[] = new Vec2[maxNumAgents];
Vec2 agentPs[] = new Vec2[maxNumAgents];
ArrayList<Integer>[] paths = new ArrayList[maxNumAgents];
int targets[] = new int[maxNumAgents];
Vec2 dirs[] = new Vec2[maxNumAgents];

Vec2 agentVel[] = new Vec2[maxNumAgents];
Vec2 agentAcc[] = new Vec2[maxNumAgents];

//Vec2 startPos = new Vec2(100, 500);
//Vec2 agentPos = new Vec2(100, 500);
float agentSpeed = 50;
//Vec2 agentDir = new Vec2(0,-1);
float agentSize = 6;
//int curTarget = 0;
//Vec2 goalPos = new Vec2(500, 200);
PShape rocket;
PShape planet;
PShape deathStar;
PImage space;

static int maxNumNodes = 1000;
Vec2[] nodePos = new Vec2[maxNumNodes];

//Return at what time agents 1 and 2 collide if they keep their current velocities
// or -1 if there is no collision.
float computeTTC(Vec2 pos1, Vec2 vel1, float radius1, Vec2 pos2, Vec2 vel2, float radius2){
  Vec2 w = pos1.minus(pos2);
  Vec2 v = vel1.minus(vel2);
  float a = dot(v,v);
  float b = 2*dot(w,v);
  float r = radius1+radius2;
  float c = dot(w,w) - r*r;
  float d = b*b - 4*a*c;
  if (d >=0 ){
    float t = (-b - sqrt(d))/(2*a);
    if (t >= 0) return t;
    return -1;
  }
  return -1;
}

// Compute attractive forces to draw agents to their goals,
// and avoidance forces to anticipatory avoid collisions
Vec2 computeAgentForces(int id){
  // add goal force
  nodePos[numNodes] = startPs[id];
  nodePos[numNodes+1] = endPs[id];
  Vec2 acc = new Vec2(0,0);
  if (targets[id] > paths[id].size()-1) {
      targets[id]=paths[id].size()-1;
  }
  Vec2 goalVel = dirs[id].times(agentSpeed);
  if (goalVel.length() > agentSpeed) goalVel.setToLength(agentSpeed);
  Vec2 goalForce = goalVel.minus(agentVel[id]);
  acc.add(goalForce.times(k_goal));

  
  // add avoidance force
  for (int j = 0; j < numAgents; j++){
    if (j != id){
      float ttc = computeTTC(agentPs[id], agentVel[id], agentSize+2,
                           agentPs[j], agentVel[j], agentSize+2);
      if (ttc > 0){
        // A_future = A_current + A_vel*ttc; B_future + B_current + B_vel*ttc
        Vec2 A_future = agentPs[id].plus(agentVel[id].times(ttc));
        Vec2 B_future = agentPs[j].plus(agentVel[j].times(ttc));
        // relative_future_direction = (A_future - B_future).normalized()
        Vec2 relative_future_direction = A_future.minus(B_future).normalized();
        // acc += k_avoid * (1/ttc) * relative_future_direction
        acc.add(relative_future_direction.times(k_avoid * (1/ttc)));
      }  
    }
  }
  
  for (int j = 0; j < numObstacles; j++){
      hitInfo hit = rayCircleListIntersect(circlePos, circleRad, numObstacles, agentPs[id], dirs[id], 5, agentSize);
      float ttc = hit.t;
      if (hit.hit){
        acc.add(dirs[id].plus(new Vec2(random(0.01)-0.005,random(0.01)-0.005)).times(-k_avoid * (1/ttc)));
      }  
  }
  
  
  acc.clampToLength(5000);
  return acc;
}


//Update agent positions & velocities based acceleration
void moveAgent(float dt){
  
  //Update position and velocity using (Eulerian) numerical integration
  for (int i = 0; i < numAgents; i++){
    //if (agentPs[i].distanceTo(endPs[i])<3) {
    //  continue;
    //}
    //Compute accelerations for every agents
    agentAcc[i] = computeAgentForces(i);
    nodePos[numNodes] = startPs[i];
    nodePos[numNodes+1] = endPs[i];
    if (nodePos[targets[i]].minus(agentPs[i]).length() > 3) { 
      agentVel[i].add(agentAcc[i].times(dt));
      agentPs[i].add(agentVel[i].times(dt));
    }
  }
}



//Generate non-colliding PRM nodes
void generateRandomNodes(int numNodes, Vec2[] circleCenters, float[] circleRadii) {
  for (int i = 0; i < numNodes; i++) {
    Vec2 randPos = new Vec2(random(width), random(height));
    boolean insideAnyCircle = pointInCircleList(circleCenters, circleRadii, numObstacles, randPos, 2);
    //boolean insideBox = pointInBox(boxTopLeft, boxW, boxH, randPos);
    while (insideAnyCircle) {
      randPos = new Vec2(random(width), random(height));
      insideAnyCircle = pointInCircleList(circleCenters, circleRadii, numObstacles, randPos, 2);
      //insideBox = pointInBox(boxTopLeft, boxW, boxH, randPos);
    }
    nodePos[i] = randPos;
  }
}

void placeRandomObstacles(int numObstacles) {
  //Initial obstacle position
  for (int i = 0; i < numObstacles; i++) {
    circlePos[i] = new Vec2(random(50, 950), random(50, 700));
    circleRad[i] = (10+40*pow(random(1), 3));
  }
  circleRad[0] = 30; //Make the first obstacle big
}

ArrayList<Integer> curPath;

int strokeWidth = 2;







void setup() {
  size(1024, 768);
  rocket = loadShape("rocket.svg");
  planet = loadShape("planet.svg");
  deathStar = loadShape("death-star-bold.svg");
  space = loadImage("space.jpg");
  testPRM();
}

int numCollisions;
float pathLength;
boolean reachedGoal;
/*
void pathQuality() {
  Vec2 dir;
  hitInfo hit;
  float segmentLength;
  numCollisions = 9999; 
  pathLength = 9999;
  if (curPath.size() == 1 && curPath.get(0) == -1) return; //No path found  

  pathLength = 0; 
  numCollisions = 0;

  if (curPath.size() == 0 ) { //Path found with no nodes (direct start-to-goal path)
    segmentLength = startPos.distanceTo(goalPos);
    pathLength += segmentLength;
    dir = goalPos.minus(startPos).normalized();
    hit = rayCircleListIntersect(circlePos, circleRad, numObstacles, startPos, dir, segmentLength,agentSize);
    if (hit.hit) numCollisions += 1;
    return;
  }

  segmentLength = startPos.distanceTo(nodePos[curPath.get(0)]);
  pathLength += segmentLength;
  dir = nodePos[curPath.get(0)].minus(startPos).normalized();
  hit = rayCircleListIntersect(circlePos, circleRad, numObstacles, startPos, dir, segmentLength, agentSize);
  if (hit.hit) numCollisions += 1;


  for (int i = 0; i < curPath.size()-1; i++) {
    int curNode = curPath.get(i);
    int nextNode = curPath.get(i+1);
    segmentLength = nodePos[curNode].distanceTo(nodePos[nextNode]);
    pathLength += segmentLength;

    dir = nodePos[nextNode].minus(nodePos[curNode]).normalized();
    hit = rayCircleListIntersect(circlePos, circleRad, numObstacles, nodePos[curNode], dir, segmentLength, agentSize);
    if (hit.hit) numCollisions += 1;
  }

  int lastNode = curPath.get(curPath.size()-1);
  segmentLength = nodePos[lastNode].distanceTo(goalPos);
  pathLength += segmentLength;
  dir = goalPos.minus(nodePos[lastNode]).normalized();
  hit = rayCircleListIntersect(circlePos, circleRad, numObstacles, nodePos[lastNode], dir, segmentLength, agentSize);
  if (hit.hit) numCollisions += 1;
}
*/

Vec2 sampleFreePos() {
  Vec2 randPos = new Vec2(random(width), random(height));
  boolean insideAnyCircle = pointInCircleList(circlePos, circleRad, numObstacles, randPos, 8);
  while (insideAnyCircle) {
    randPos = new Vec2(random(width), random(height));
    insideAnyCircle = pointInCircleList(circlePos, circleRad, numObstacles, randPos, 8);
  }
  return randPos;
}

void testPRM() {
  //long startTime, endTime;

  placeRandomObstacles(numObstacles);
  /*
  startPos = sampleFreePos();
  agentPos = startPos.clone();
  goalPos = sampleFreePos();
  curTarget = 1;
  */
  generateRandomNodes(numNodes, circlePos, circleRad);
  connectNeighbors(circlePos, circleRad, numObstacles, nodePos, numNodes);

  //startTime = System.nanoTime();
  /*
  curPath = planPath(startPos, goalPos, circlePos, circleRad, numObstacles, nodePos, numNodes);
  if (curPath.size() > 1) {
    nodePos[numNodes] = startPos;
    nodePos[numNodes+1] = goalPos;
    agentDir = nodePos[curPath.get(curTarget)].minus(agentPos).normalized();
  } 
  */
  for (int i = 0; i < numAgents; i++) {
    startPs[i] = sampleFreePos();
    agentPs[i] = startPs[i].clone();
    endPs[i] = sampleFreePos();
    paths[i] = planPath(startPs[i], endPs[i], circlePos, circleRad, numObstacles, nodePos, numNodes);
    if (paths[i].size() > 1) {
      nodePos[numNodes] = startPs[i];
      nodePos[numNodes+1] = endPs[i];
      targets[i] = 1;
      dirs[i] = nodePos[paths[i].get(1)].minus(startPs[i]).normalized();
      agentVel[i] = nodePos[paths[i].get(1)].minus(startPs[i]).times(0.1);
      agentVel[i].clampToLength(10);
    }
  }
  //endTime = System.nanoTime();
  //pathQuality();

  //println("Nodes:", numNodes, " Obstacles:", numObstacles, " Time (us):", int((endTime-startTime)/1000), 
  //  " Path Len:", pathLength, " Path Segment:", curPath.size()+1, " Num Collisions:", numCollisions);
}

void draw() {
  //println("FrameRate:",frameRate);
  strokeWeight(1);
  background(space); //Grey background
  stroke(0, 0, 0);
  fill(255, 255, 255);

  //Draw the circle obstacles
  for (int i = 0; i < numObstacles; i++) {
    Vec2 c = circlePos[i];
    float r = circleRad[i];
    //circle(c.x, c.y, r*2);
    shape(planet, c.x-r, c.y-r, 2*r, 2*r);
  }
  //Draw the first circle a little special b/c the user controls it
  fill(240);
  strokeWeight(2);
  //circle(circlePos[0].x, circlePos[0].y, circleRad[0]*2);
  float rad = circleRad[0];
  shape(deathStar, circlePos[0].x-rad, circlePos[0].y-rad, rad*2, rad*2);
  strokeWeight(1);

  /*
  //Draw PRM Nodes
  fill(0);
  for (int i = 0; i < numNodes; i++) {
    circle(nodePos[i].x, nodePos[i].y, 5);
  }
  */
  /*
  //Draw graph
  stroke(100, 100, 100);
  strokeWeight(1);
  for (int i = 0; i < numNodes; i++) {
    for (int j : neighbors[i]) {
      line(nodePos[i].x, nodePos[i].y, nodePos[j].x, nodePos[j].y);
    }
  }
  */
  //Draw Start and Goal
  fill(20, 60, 250);
  //circle(nodePos[startNode].x,nodePos[startNode].y,20);
  circle(startPs[0].x, startPs[0].y, 20);
  fill(250, 30, 50);
  //circle(nodePos[goalNode].x,nodePos[goalNode].y,20);
  circle(endPs[0].x, endPs[0].y, 20);

  //if (curPath.size() >0 && curPath.get(0) == -1) return; //No path found

  //Draw Planned Path
  /*
  stroke(20, 255, 40);
  strokeWeight(5);
  if (curPath.size() == 0) {
    line(startPos.x, startPos.y, goalPos.x, goalPos.y);
    return;
  }
  line(startPos.x, startPos.y, nodePos[curPath.get(0)].x, nodePos[curPath.get(0)].y);
  for (int i = 0; i < curPath.size()-1; i++) {
    int curNode = curPath.get(i);
    int nextNode = curPath.get(i+1);
    line(nodePos[curNode].x, nodePos[curNode].y, nodePos[nextNode].x, nodePos[nextNode].y);
  }
  line(goalPos.x, goalPos.y, nodePos[curPath.get(curPath.size()-1)].x, nodePos[curPath.get(curPath.size()-1)].y);
  */
  
  
  // Draw agent
  //circle(agentPos.x, agentPos.y, 20);
  float size = 20;
  Vec2 dir;
  Vec2 curTarPos;
  float dis;
  for (int n = 0; n < numAgents; n++) {
    if (paths[n].size()==1) {
      continue;
    }
    nodePos[numNodes] = startPs[n];
    nodePos[numNodes+1] = endPs[n];
    pushMatrix();
    translate(agentPs[n].x,agentPs[n].y);
    rotate(atan2(dirs[n].y,dirs[n].x)+PI/2);
    shape(rocket, -size/2, -size/2, size, size);
    popMatrix();
    // Update agent
    if (agentPs[n].distanceTo(endPs[n])<3) {
      continue;
    }
    if (targets[n] > paths[n].size()-1) {
      targets[n]= paths[n].size()-1;
    }
    if (targets[n] < paths[n].size()) {
      for (int i = paths[n].size()-1; i > targets[n] ; i--) {
        dir = nodePos[paths[n].get(i)].minus(agentPs[n]).normalized();
        curTarPos = nodePos[paths[n].get(i)].clone();
        dis = curTarPos.distanceTo(agentPs[n]);
        hitInfo hit = rayCircleListIntersect(circlePos, circleRad, numObstacles, agentPs[n], dir, dis, agentSize);
        if (!hit.hit) {
          //println(curTarget+"->"+i);
          targets[n] = i;
          break;
        }
      }
      curTarPos = nodePos[paths[n].get(targets[n])].clone();
      dis = curTarPos.distanceTo(agentPs[n]);
      if (dis < agentSpeed/frameRate) {
        agentPs[n] = curTarPos;
        targets[n]++;
      } else {
        Vec2 to = nodePos[paths[n].get(targets[n])].minus(agentPs[n]).normalized();
        dirs[n] = interpolate(dirs[n], to, 0.1);
        agentVel[n] = dirs[n].times(agentSpeed);
        //agentPs[n].add(dirs[n].times(agentSpeed/frameRate));
        moveAgent(1/frameRate);
      }
    }
  }
  
}

boolean shiftDown = false;
void keyPressed() {
  if (key == 'r') {
    testPRM();
    return;
  }

  if (keyCode == SHIFT) {
    shiftDown = true;
  }

  float speed = 10;
  if (shiftDown) speed = 30;
  if (keyCode == RIGHT) {
    circlePos[0].x += speed;
  }
  if (keyCode == LEFT) {
    circlePos[0].x -= speed;
  }
  if (keyCode == UP) {
    circlePos[0].y -= speed;
  }
  if (keyCode == DOWN) {
    circlePos[0].y += speed;
  }
  connectNeighbors(circlePos, circleRad, numObstacles, nodePos, numNodes);
  for (int i = 0; i < numAgents; i++) {
    paths[i] = planPath(startPs[i], endPs[i], circlePos, circleRad, numObstacles, nodePos, numNodes);
  }
}

void keyReleased() {
  if (keyCode == SHIFT) {
    shiftDown = false;
  }
}

void mousePressed() {
  if (mouseButton == RIGHT) {
    startPs[0] = new Vec2(mouseX, mouseY);
    targets[0] = 0;
    agentPs[0] = startPs[0].clone();
    //println("New Start is",startPos.x, startPos.y);
  } else if (mouseButton == LEFT) {
    endPs[0] = new Vec2(mouseX, mouseY);
    targets[0] = 0;
    agentPs[0] = startPs[0].clone();
    //println("New Goal is",goalPos.x, goalPos.y);
  } else if (mouseButton == CENTER) {
    circlePos[0] = new Vec2(mouseX, mouseY);
    connectNeighbors(circlePos, circleRad, numObstacles, nodePos, numNodes);
    for (int i = 0; i < numAgents; i++) {
    paths[i] = planPath(startPs[i], endPs[i], circlePos, circleRad, numObstacles, nodePos, numNodes);
    if (paths[i].size() > 1) {
      nodePos[numNodes] = startPs[i];
      nodePos[numNodes+1] = endPs[i];
      targets[i] = 1;
      dirs[i] = nodePos[paths[i].get(1)].minus(startPs[i]);
    }
  }
  }
  paths[0] = planPath(startPs[0], endPs[0], circlePos, circleRad, numObstacles, nodePos, numNodes);
}
