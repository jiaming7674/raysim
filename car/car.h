#ifndef __CAR_H__
#define __CAR_H__

#include "raylib.h"
#include "raymath.h"
#include "icollidable.h"
#include <vector>
#include <cmath>


const int MAX_TRAIL_POINTS = 1000;



class Trail
{
public:
  std::vector<Vector2> points;
public:
  Trail() {};
  void Update(Vector2 newPoint);
  void Draw() const;
};


class Car : protected ICollidable
{
private:
  Vector2 position;
  Vector2 rearAxlePosition;
  float rotation;
  float speed;
  float steering;
  float wheelBase;
  float wheelTrack;
  float wheelWidth;
  float wheelLength;
  float maxSteeringAngle;
  float steeringSpeed;
  float frontWheelOffset;
  float rearWheelOffset;
  float maxSpeed;
  float acceleration;

  Trail flTrail, frTrail, rlTrail, rrTrail;

public:
  Car(Vector2 position);
  void Accelerate(float amount);
  void Steer(float amount);
  void UpdateRearAxlePosition(void);
  Rectangle GetBoundingBox() const;
  bool CheckCollision(const Rectangle &bounds) const override;
  bool CheckCollision(const ICollidable* collidable) const override;
  float DetectObstacleAtAngle(const std::vector<ICollidable*> &collidables, float angle) const;
  std::vector<float> GetSensorData(const std::vector<ICollidable*> &collidables) const;
  void Update(std::vector<ICollidable*> &collidables);
  //std::vector<float> GetState(const std::vector<Obstacle> &obstacles) const;
  void Draw();
  void DrawBoundingBox() const;
  void DrawSensors(const std::vector<ICollidable*> &collidables) const;
};


#endif