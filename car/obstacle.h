#ifndef __OBSTACLE_H__
#define __OBSTACLE_H__

#include "raylib.h"
#include "icollidable.h"

class Obstacle : public ICollidable
{
public:
  Rectangle rect;
  Color color;

  Obstacle(float x, float y, float width, float height, Color c)
      : rect{x, y, width, height}, color(c) {
        this->bound = rect;
      }

  bool CheckCollision(const Rectangle &bounds) const override {
    return CheckCollisionRecs(rect, bounds);
  }

  bool CheckCollision(const ICollidable* collidable) const override {
    return false;
  }
  
  void Draw() const
  {
    DrawRectangleRec(rect, color);
  }
};


#endif