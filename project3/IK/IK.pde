//Inverse Kinematics
//CSCI 5611 IK [Solution]
// Stephen J. Guy <sjguy@umn.edu>

/*
INTRODUCTION:
Rather than making an artist control every aspect of a characters animation, we will often specify 
key points (e.g., center of mass and hand position) and let an optimizer find the right angles for 
all of the joins in the character’s skelton. This is called Inverse Kinematics (IK). Here, we start 
with some simple IK code and try to improve the results a bit to get better motion.

TODO:
Step 1. Change the joint lengths and colors to look more like a human arm. Try to match 
        the length ratios of your own arm/hand, and try to match your own skin tone in the rendering.

Step 2: Add an angle limit to the shoulder joint to limit the joint to be between 0 and 90 degrees, 
        this should stop the top of the arm from moving off screen.

Step 3: Add an angle limit to the wrist joint, and limit it to be within +/- 90 degrees relative
        to the lower arm.

Step 4: Cap the acceleration of each joint so the joints can only update slowly. Try to tweak the 
        acceleration cap to be different for each joint to get a good effect on the arm motion.

Step 5: Try adding a 4th limb to the IK chain.


CHALLENGE:

1. Go back to the 3-limb arm, can you make it look more human-like. Try adding a simple body to 
   the scene using circles and rectangles. Can you make a scene where the character picks up 
   something and moves it somewhere?
2. Create a more full skeleton. How do you handle the torso having two different arms?

*/

void setup(){
  size(640,480);
  surface.setTitle("Inverse Kinematics [CSCI 5611 Example]");
  fkLeft();
  solveLeft();
  fkRight();
  solveRight();
}

float armW = 20;

// Left Root
Vec2 leftRoot = new Vec2(200,100);

//Left Upper Arm
float l0 = 100; 
float a0 = 0.3; //Shoulder joint

//Left Lower Arm
float l1 = 90;
float a1 = 0.3; //Elbow joint

//Left Hand
float l2 = 40;
float a2 = 0.3; //Wrist joint

//Left Finger
float l3 = 10;
float a3 = 0.3; //Finger joint

// Add Right Arm

Vec2 rightRoot = new Vec2(440,100);

float r0 = 100;
float b0 = 0.3; //Shoulder joint

float r1 = 90;
float b1 = 0.3; //Elbow joint

float r2 = 40;
float b2 = 0.3; //Wrist joint

float r3 = 10;
float b3 = 0.3; //Finger joint

Vec2 goal = new Vec2(310,240);

// Add obstacles

Vec2 obstacle1 = new Vec2(350+500,180);
float obstacle1Radius = 30;

Vec2 obstacle2 = new Vec2(270+500,300);
float obstacle2Radius = 60;


Vec2 start_l1,start_l2,start_l3,leftEndPoint;
Vec2 start_r1,start_r2,start_r3,rightEndPoint;

void solveLeft(){
  Vec2 startToGoal, startToEndEffector;
  float dotProd, angleDiff;

  //Update shoulder joint
  startToGoal = goal.minus(leftRoot);
  if (startToGoal.length() < .0001) return;
  startToEndEffector = leftEndPoint.minus(leftRoot);
  dotProd = dot(startToGoal.normalized(),startToEndEffector.normalized());
  dotProd = clamp(dotProd,-1,1);
  angleDiff = acos(dotProd);
  angleDiff = clamp(angleDiff,0,0.04);
  if (cross(startToGoal,startToEndEffector) < 0)
    a0 += angleDiff/3;
  else
    a0 -= angleDiff/3;

  fkLeft(); //Update link positions with fkLeft (e.g. end effector changed)

  // check for collisions
  if (collision(leftRoot, start_l1)) {
    if (cross(startToGoal,startToEndEffector) < 0)
      a0 -= angleDiff;
    else
      a0 += angleDiff;

    fkLeft();
  }


  // clamp the a0 angle to be between 0 and 90 degrees
  a0 = clamp(a0,0,PI/2);


  //Update elbow joint
  startToGoal = goal.minus(start_l1);
  startToEndEffector = leftEndPoint.minus(start_l1);
  dotProd = dot(startToGoal.normalized(),startToEndEffector.normalized());
  dotProd = clamp(dotProd,-1,1);
  angleDiff = acos(dotProd);
  angleDiff = clamp(angleDiff,0,0.04);
  if (cross(startToGoal,startToEndEffector) < 0)
    a1 += angleDiff/2;
  else
    a1 -= angleDiff/2;
  fkLeft(); //Update link positions with fkLeft (e.g. end effector changed)
  
  // check for collisions
  if (collision(start_l1, start_l2)) {
    if (cross(startToGoal,startToEndEffector) < 0)
      a1 -= angleDiff;
    else
      a1 += angleDiff;

    fkLeft();
  }

  //Update wrist joint
  startToGoal = goal.minus(start_l2);
  startToEndEffector = leftEndPoint.minus(start_l2);
  dotProd = dot(startToGoal.normalized(),startToEndEffector.normalized());
  dotProd = clamp(dotProd,-1,1);
  angleDiff = acos(dotProd);
  angleDiff = clamp(angleDiff,0,0.04);
  if (cross(startToGoal,startToEndEffector) < 0)
    a2 += angleDiff;
  else
    a2 -= angleDiff;
  
  fkLeft(); //Update link positions with fkLeft (e.g. end effector changed)

  if (collision(start_l2, start_l3)) {
    if (cross(startToGoal,startToEndEffector) < 0)
      a2 -= angleDiff;
    else
      a2 += angleDiff;

    fkLeft();
  }

  // clamp wrist joint to +/- 90 degrees
  a2 = clamp(a2,-PI/2,PI/2);
  
  //Update finger joint
  startToGoal = goal.minus(start_l3);
  startToEndEffector = leftEndPoint.minus(start_l3);
  dotProd = dot(startToGoal.normalized(),startToEndEffector.normalized());
  dotProd = clamp(dotProd,-1,1);
  angleDiff = acos(dotProd);
  angleDiff = clamp(angleDiff,0,0.04);
  if (cross(startToGoal,startToEndEffector) < 0)
    a3 += angleDiff;
  else
    a3 -= angleDiff;

  fkLeft();
  if (collision(start_l3, leftEndPoint)) {
    if (cross(startToGoal,startToEndEffector) < 0)
      a3 -= angleDiff;
    else
      a3 += angleDiff;

    fkLeft();
  }
  
  //println("Angle 0:",a0,"Angle 1:",a1,"Angle 2:",a2);
}

void fkLeft(){
  start_l1 = new Vec2(cos(a0)*l0,sin(a0)*l0).plus(leftRoot);
  start_l2 = new Vec2(cos(a0+a1)*l1,sin(a0+a1)*l1).plus(start_l1);
  start_l3 = new Vec2(cos(a0+a1+a2)*l2,sin(a0+a1+a2)*l2).plus(start_l2);
  leftEndPoint = new Vec2(cos(a0+a1+a2+a3)*l3,sin(a0+a1+a2+a3)*l3).plus(start_l3);
}

void solveRight(){
  Vec2 startToGoal, startToEndEffector;
  float dotProd, angleDiff;

  //Update finger joint
  startToGoal = goal.minus(start_r3);
  startToEndEffector = rightEndPoint.minus(start_r3);
  dotProd = dot(startToGoal.normalized(),startToEndEffector.normalized());
  dotProd = clamp(dotProd,-1,1);
  angleDiff = acos(dotProd);
  angleDiff = clamp(angleDiff,0,0.04);
  if (cross(startToGoal,startToEndEffector) < 0)
    b3 += angleDiff;
  else
    b3 -= angleDiff;
  fkRight();

  // check for collisions
  if (collision(start_r3, rightEndPoint)) {
    if (cross(startToGoal,startToEndEffector) < 0)
      b3 -= angleDiff;
    else
      b3 += angleDiff;

    fkRight();
  }

  //Update wrist joint
  startToGoal = goal.minus(start_r2);
  startToEndEffector = rightEndPoint.minus(start_r2);
  dotProd = dot(startToGoal.normalized(),startToEndEffector.normalized());
  dotProd = clamp(dotProd,-1,1);
  angleDiff = acos(dotProd);
  angleDiff = clamp(angleDiff,0,0.04);
  if (cross(startToGoal,startToEndEffector) < 0)
    b2 += angleDiff;
  else
    b2 -= angleDiff;

  // clamp wrist joint to +/- 90 degrees
  b2 = clamp(b2,-PI/2,PI/2);
  fkRight(); //Update link positions with fkRight (e.g. end effector changed)

  // check for collisions
  if (collision(start_r2, start_r3)) {
    if (cross(startToGoal,startToEndEffector) < 0)
      b2 -= angleDiff;
    else
      b2 += angleDiff;

    fkRight();
  }

  //Update elbow joint
  startToGoal = goal.minus(start_r1);
  startToEndEffector = rightEndPoint.minus(start_r1);
  dotProd = dot(startToGoal.normalized(),startToEndEffector.normalized());
  dotProd = clamp(dotProd,-1,1);
  angleDiff = acos(dotProd);
  angleDiff = clamp(angleDiff,0,0.04);
  if (cross(startToGoal,startToEndEffector) < 0)
    b1 += angleDiff/2;
  else
    b1 -= angleDiff/2;
  fkRight(); //Update link positions with fkRight (e.g. end effector changed)

  // check for collisions
  if (collision(start_r1, start_r2)) {
    if (cross(startToGoal,startToEndEffector) < 0)
      b1 -= angleDiff;
    else
      b1 += angleDiff;

    fkRight();
  }

  //Update shoulder joint
  startToGoal = goal.minus(rightRoot);
  if (startToGoal.length() < .0001) return;
  startToEndEffector = rightEndPoint.minus(rightRoot);
  dotProd = dot(startToGoal.normalized(),startToEndEffector.normalized());
  dotProd = clamp(dotProd,-1,1);
  angleDiff = acos(dotProd);
  angleDiff = clamp(angleDiff,0,0.04);
  if (cross(startToGoal,startToEndEffector) < 0)
    b0 += angleDiff/3;
  else
    b0 -= angleDiff/3;

  fkRight(); //Update link positions with fkRight (e.g. end effector changed)

  // check for collisions
  if (collision(rightRoot, start_r1)) {
    //println("Collision!");
    if (cross(startToGoal,startToEndEffector) < 0)
      b0 -= angleDiff;
    else
      b0 += angleDiff;

    fkRight();
  } else {
    //println("No Collision!");
  }

  // clamp the b0 angle to be between 0 and 90 degrees
  b0 = clamp(b0,PI/2,PI);

  //println("Angle 0:",b0,"Angle 1:",b1,"Angle 2:",b2);
}

void fkRight(){
  start_r1 = new Vec2(cos(b0)*l0,sin(b0)*l0).plus(rightRoot);
  start_r2 = new Vec2(cos(b0+b1)*l1,sin(b0+b1)*l1).plus(start_r1);
  start_r3 = new Vec2(cos(b0+b1+b2)*l2,sin(b0+b1+b2)*l2).plus(start_r2);
  rightEndPoint = new Vec2(cos(b0+b1+b2+b3)*l3,sin(b0+b1+b2+b3)*l3).plus(start_r3);
}

boolean collision(Vec2 start, Vec2 end){
  return lineCircleIntersection(start,end,obstacle1,obstacle1Radius) ||
  lineCircleIntersection(start,end,obstacle2,obstacle2Radius);
}

boolean lineCircleIntersection(Vec2 start, Vec2 end, Vec2 circleCenter, float radius){
  // draw line
  line(start.x,start.y,end.x,end.y);

  // check if either end of the line is inside the circle
  if (start.minus(circleCenter).length() < radius || end.minus(circleCenter).length() < radius)
    return true;

  // check if any point on the line is inside the circle
  Vec2 lineDir = end.minus(start);
  float lineLength = lineDir.length();
  lineDir = lineDir.normalized();
  float lineStep = lineLength/100;
  for (float i = 0; i < lineLength; i += lineStep) {
    Vec2 point = start.plus(lineDir.times(i));
    if (point.minus(circleCenter).length() < radius)
      return true;
  }
  return false;
}



void draw(){

  // set goal to mouse position on click
  if (mousePressed) {
    goal = new Vec2(mouseX,mouseY);
  }

  
  
  background(250,250,250);
  
  // set color to blue
  fill(0,0,255);
  pushMatrix();
  translate(leftRoot.x,leftRoot.y);
  rotate(a0);
  rect(0, -armW/2, l0, armW);
  popMatrix();
  
  pushMatrix();
  translate(start_l1.x,start_l1.y);
  rotate(a0+a1);
  rect(0, -armW/2, l1, armW);
  popMatrix();
  
  pushMatrix();
  translate(start_l2.x,start_l2.y);
  rotate(a0+a1+a2);
  rect(0, -armW/2, l2, armW);
  popMatrix();
  
  pushMatrix();
  translate(start_l3.x,start_l3.y);
  rotate(a0+a1+a2+a3);
  rect(0, -armW/2, l3, armW);
  popMatrix();

  // set color to red
  fill(255,0,0);
  pushMatrix();
  translate(rightRoot.x,rightRoot.y);
  rotate(b0);
  rect(0, -armW/2, l0, armW);
  popMatrix();

  pushMatrix();
  translate(start_r1.x,start_r1.y);
  rotate(b0+b1);
  rect(0, -armW/2, l1, armW);
  popMatrix();

  pushMatrix();
  translate(start_r2.x,start_r2.y);
  rotate(b0+b1+b2);
  rect(0, -armW/2, l2, armW);
  popMatrix();

  pushMatrix();
  translate(start_r3.x,start_r3.y);
  rotate(b0+b1+b2+b3);
  rect(0, -armW/2, l3, armW);
  popMatrix();

  // draw shoulder connect 2 arms
  fill(255,224,189);
  pushMatrix();
  translate(leftRoot.x,leftRoot.y);
  rect(0,-armW/2,rightRoot.x-leftRoot.x,armW);
  popMatrix();

  // draw obstacles
  // set color to red
  fill(255,0,0);
  circle(obstacle1.x,obstacle1.y,obstacle1Radius);
  circle(obstacle2.x,obstacle2.y,obstacle2Radius);

  // draw goal
  fill(0,0,255);
  circle(goal.x,goal.y,10);

  fkLeft();
  solveLeft();
  fkRight();
  solveRight();

}



//-----------------
// Vector Library
//-----------------

//Vector Library
//CSCI 5611 Vector 2 Library [Example]
// Stephen J. Guy <sjguy@umn.edu>

public class Vec2 {
  public float x, y;
  
  public Vec2(float x, float y){
    this.x = x;
    this.y = y;
  }
  
  public String toString(){
    return "(" + x+ "," + y +")";
  }
  
  public float length(){
    return sqrt(x*x+y*y);
  }
  
  public Vec2 plus(Vec2 rhs){
    return new Vec2(x+rhs.x, y+rhs.y);
  }
  
  public void add(Vec2 rhs){
    x += rhs.x;
    y += rhs.y;
  }
  
  public Vec2 minus(Vec2 rhs){
    return new Vec2(x-rhs.x, y-rhs.y);
  }
  
  public void subtract(Vec2 rhs){
    x -= rhs.x;
    y -= rhs.y;
  }
  
  public Vec2 times(float rhs){
    return new Vec2(x*rhs, y*rhs);
  }
  
  public void mul(float rhs){
    x *= rhs;
    y *= rhs;
  }
  
  public void clampToLength(float maxL){
    float magnitude = sqrt(x*x + y*y);
    if (magnitude > maxL){
      x *= maxL/magnitude;
      y *= maxL/magnitude;
    }
  }
  
  public void setToLength(float newL){
    float magnitude = sqrt(x*x + y*y);
    x *= newL/magnitude;
    y *= newL/magnitude;
  }
  
  public void normalize(){
    float magnitude = sqrt(x*x + y*y);
    x /= magnitude;
    y /= magnitude;
  }
  
  public Vec2 normalized(){
    float magnitude = sqrt(x*x + y*y);
    return new Vec2(x/magnitude, y/magnitude);
  }
  
  public float distanceTo(Vec2 rhs){
    float dx = rhs.x - x;
    float dy = rhs.y - y;
    return sqrt(dx*dx + dy*dy);
  }
}

Vec2 interpolate(Vec2 a, Vec2 b, float t){
  return a.plus((b.minus(a)).times(t));
}

float interpolate(float a, float b, float t){
  return a + ((b-a)*t);
}

float dot(Vec2 a, Vec2 b){
  return a.x*b.x + a.y*b.y;
}

float cross(Vec2 a, Vec2 b){
  return a.x*b.y - a.y*b.x;
}


Vec2 projAB(Vec2 a, Vec2 b){
  return b.times(a.x*b.x + a.y*b.y);
}

float clamp(float f, float min, float max){
  if (f < min) return min;
  if (f > max) return max;
  return f;
}
