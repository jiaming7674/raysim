#include "icollidable.h"


// 新增：檢查點是否在多邊形內
bool ICollidable::CheckCollisionPointPoly(Vector2 point, Vector2 *vertices, int vertexCount) const
{
  bool inside = false;
  for (int i = 0, j = vertexCount - 1; i < vertexCount; j = i++)
  {
    if (((vertices[i].y > point.y) != (vertices[j].y > point.y)) &&
        (point.x < (vertices[j].x - vertices[i].x) * (point.y - vertices[i].y) / (vertices[j].y - vertices[i].y) + vertices[i].x))
    {
      inside = !inside;
    }
  }
  return inside;
}


bool ICollidable::LineIntersectsLine(Vector2 p1, Vector2 p2, Vector2 p3, Vector2 p4, Vector2 *intersection) const
{
  float denominator = ((p4.y - p3.y) * (p2.x - p1.x) - (p4.x - p3.x) * (p2.y - p1.y));

  if (denominator == 0)
  {
    return false; // 線段平行
  }

  float ua = ((p4.x - p3.x) * (p1.y - p3.y) - (p4.y - p3.y) * (p1.x - p3.x)) / denominator;
  float ub = ((p2.x - p1.x) * (p1.y - p3.y) - (p2.y - p1.y) * (p1.x - p3.x)) / denominator;

  if (ua < 0 || ua > 1 || ub < 0 || ub > 1)
  {
    return false; // 線段不相交
  }

  if (intersection)
  {
    intersection->x = p1.x + ua * (p2.x - p1.x);
    intersection->y = p1.y + ua * (p2.y - p1.y);
  }

  return true;
}


bool ICollidable::CheckCollisionLineRec(Vector2 startPos, Vector2 endPos, Rectangle rec, Vector2 *collisionPoint) const
{
  Vector2 recTopLeft = {rec.x, rec.y};
  Vector2 recTopRight = {rec.x + rec.width, rec.y};
  Vector2 recBottomLeft = {rec.x, rec.y + rec.height};
  Vector2 recBottomRight = {rec.x + rec.width, rec.y + rec.height};

  if (LineIntersectsLine(startPos, endPos, recTopLeft, recTopRight, collisionPoint) ||
      LineIntersectsLine(startPos, endPos, recTopRight, recBottomRight, collisionPoint) ||
      LineIntersectsLine(startPos, endPos, recBottomRight, recBottomLeft, collisionPoint) ||
      LineIntersectsLine(startPos, endPos, recBottomLeft, recTopLeft, collisionPoint))
  {
    return true;
  }

  return false;
}
