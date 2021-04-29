template <typename T> struct BlackBoard {
  std::string input;
  std::string eval;
};

struct Behavior {
  std::string description;
  std::vector<BlackBoard> blackboard;
};
struct Position {
  double x, y, z;
};
struct Quaternion {
  double x, y, z, w;
};
struct Pose {
  Position position;
  Quaternion orientation;
};

struct Object {
  int uuid;
  std::vector<std::string> attributes;
  Pose pose;
};


struct Format{
  Behavior behavior;
  std::vector<Object> objects;
};


