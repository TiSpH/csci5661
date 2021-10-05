public static class Node implements Comparable<Node>{
  int i;
  float h;
  float g;
  
  public Node(int index, Vec2[] nodePos, int goalID, float gs){
    i = index;
    h = nodePos[i].distanceTo(nodePos[goalID]);
    g = gs;
  }
  
  public int compareTo(Node other){
    if ((h+g)>(other.h+other.g))return 1;
    if ((h+g)<(other.h+other.g))return -1;
    return 0;
  }
  
  public boolean equals(Node other) {
    return (i == other.i);
  }
}
