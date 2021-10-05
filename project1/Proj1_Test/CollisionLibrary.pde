//Replace this file with your HW1 solution

/////////
// Point Intersection Tests
/////////

//Returns true if the point is inside a box
boolean pointInBox(Vec2 boxTopLeft, float boxW, float boxH, Vec2 pointPos){
  //TODO: Return true if the point is actually inside the box
  if (boxTopLeft.x<pointPos.x && pointPos.x<(boxTopLeft.x+boxW) &&
      boxTopLeft.y<pointPos.y && pointPos.y<(boxTopLeft.y+boxH)) {
    return true;
  }
  return false;
}

//Returns true if the point is inside a circle
boolean pointInCircle(Vec2 center, float r, Vec2 pointPos, float eps){
  float dist = pointPos.distanceTo(center);
  if (dist < (r+eps)){ //small safety factor
    return true;
  }
  return false;
}

//Returns true if the point is inside a list of circle
boolean pointInCircleList(Vec2[] centers, float[] radii, int numObstacles, Vec2 pointPos, float eps){
  for (int i = 0; i < numObstacles; i++){
    Vec2 center =  centers[i];
    float r = radii[i];
    if (pointInCircle(center,r,pointPos, eps)){
      return true;
    }
  }
  return false;
}




/////////
// Ray Intersection Tests
/////////

class hitInfo{
  public boolean hit = false;
  public float t = 9999999;
}

hitInfo rayBoxIntersect(Vec2 boxTopLeft, float boxW, float boxH, Vec2 ray_start, Vec2 ray_dir, float max_t){
  hitInfo hit = new hitInfo();
  hit.hit = true;
  
  float t_left_x, t_right_x, t_top_y, t_bot_y;
  t_left_x = (boxTopLeft.x - ray_start.x)/ray_dir.x;
  t_right_x = (boxTopLeft.x + boxW - ray_start.x)/ray_dir.x;
  t_top_y = (boxTopLeft.y - ray_start.y)/ray_dir.y;
  t_bot_y = (boxTopLeft.y + boxH - ray_start.y)/ray_dir.y;
  
  float t_max_x = max(t_left_x,t_right_x);
  float t_max_y = max(t_top_y,t_bot_y);
  float t_max = min(t_max_x,t_max_y); //When the ray exists the box
  
  float t_min_x = min(t_left_x,t_right_x);
  float t_min_y = min(t_top_y,t_bot_y);
  float t_min = max(t_min_x,t_min_y); //When the ray enters the box
  
  
  //The the box is behind the ray (negative t)
  if (t_max < 0){
    hit.hit = false;
    hit.t = t_max;
    return hit;
  }
  
  //The ray never hits the box
  if (t_min > t_max){
    hit.hit = false;
  }
  
  //The ray hits, but further out than max_t
  if (t_min > max_t){
    hit.hit = false;
  }
  
  hit.t = t_min;
  return hit;
}

hitInfo rayCircleIntesect(Vec2 center, float r, Vec2 l_start, Vec2 l_dir, float max_t, float eps){
  hitInfo hit = new hitInfo();
  
  //Step 2: Compute W - a displacement vector pointing from the start of the line segment to the center of the circle
    Vec2 toCircle = center.minus(l_start);
    
    //Step 3: Solve quadratic equation for intersection point (in terms of l_dir and toCircle)
    float a = 1;  //Length of l_dir (we normalized it)
    float b = -2*dot(l_dir,toCircle); //-2*dot(l_dir,toCircle)
    float c = toCircle.lengthSqr() - (r+eps+strokeWidth)*(r+eps+strokeWidth); //different of squared distances
    
    float d = b*b - 4*a*c; //discriminant 
    
    if (d >=0 ){ 
      //If d is positive we know the line is colliding, but we need to check if the collision line within the line segment
      //  ... this means t will be between 0 and the length of the line segment
      float t1 = (-b - sqrt(d))/(2*a); //Optimization: we only need the first collision
      float t2 = (-b + sqrt(d))/(2*a); //Optimization: we only need the first collision
      //println(hit.t,t1,t2);
      if (t1 > 0 && t1 < max_t){
        hit.hit = true;
        hit.t = t1;
      }
      else if (t1 < 0 && t2 > 0){
        hit.hit = true;
        hit.t = -1;
      }
      
    }
    
  return hit;
}

hitInfo rayCircleListIntersect(Vec2[] centers, float[] radii, int numObstacles, Vec2 l_start, Vec2 l_dir, float max_t, float eps){
  hitInfo hit = new hitInfo();
  hit.t = max_t;
  for (int i = 0; i < numObstacles; i++){
    Vec2 center = centers[i];
    float r = radii[i];
    
    hitInfo circleHit = rayCircleIntesect(center, r, l_start, l_dir, hit.t, eps);
    if (circleHit.t > 0 && circleHit.t < hit.t){
      hit.hit = true;
      hit.t = circleHit.t;
    }
    else if (circleHit.hit && circleHit.t < 0){
      hit.hit = true;
      hit.t = -1;
    }
  }
  return hit;
}
