#ifndef __ICOLLIDABLE_H__
#define __ICOLLIDABLE_H__

#include "raylib.h"
#include "raymath.h"

class ICollidable
{
protected:
  Rectangle bound;
public:
  ICollidable () {};
public:
  virtual bool CheckCollision(const Rectangle & bounds) const = 0;
  virtual bool CheckCollision(const ICollidable* collidable) const = 0;
  virtual Rectangle GetBoundBox() const { return bound; };
protected:
  bool CheckCollisionPointPoly(Vector2 point, Vector2 *vertices, int vertexCount) const;
  bool LineIntersectsLine(Vector2 p1, Vector2 p2, Vector2 p3, Vector2 p4, Vector2 *intersection) const;
  bool CheckCollisionLineRec(Vector2 startPos, Vector2 endPos, Rectangle rec, Vector2 *collisionPoint) const;
};

#endif