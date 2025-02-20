#include "car.h"


void Trail::Update(Vector2 newPoint)
{
  if (points.size() >= MAX_TRAIL_POINTS)
  {
    points.erase(points.begin());
  }
  points.push_back(newPoint);
}


void Trail::Draw() const
{
  for (size_t i = 1; i < points.size(); i++)
  {
    DrawLineV(points[i - 1], points[i], GRAY);
  }
}


Car::Car(Vector2 position)
      : position{position},
        rearAxlePosition{position},
        rotation(0),
        speed(0),
        steering(0),
        wheelBase(100/2),
        wheelTrack(60/2),
        wheelWidth(10/2),
        wheelLength(20/2),
        maxSteeringAngle(0.6f),
        steeringSpeed(0.05f),
        frontWheelOffset(-15),
        rearWheelOffset(15),
        maxSpeed(5.0f),
        acceleration(0.1f)
{
  this->bound = GetBoundingBox();
  UpdateRearAxlePosition();
}


void Car::Accelerate(float amount)
{
  speed += amount * acceleration;
  speed = Clamp(speed, -maxSpeed, maxSpeed);
}


void Car::Steer(float amount)
{
  steering += amount * steeringSpeed;
  steering = Clamp(steering, -maxSteeringAngle, maxSteeringAngle);
}


void Car::UpdateRearAxlePosition()
{
  rearAxlePosition.x = position.x - wheelBase / 2 * std::cos(rotation);
  rearAxlePosition.y = position.y - wheelBase / 2 * std::sin(rotation);
}


Rectangle Car::GetBoundingBox() const
{
  float width = std::max(wheelBase, wheelTrack);
  float height = std::min(wheelBase, wheelTrack);
  return Rectangle{position.x - width / 2, position.y - height / 2, width, height};
}


bool Car::CheckCollision(const Rectangle &bounds) const
{
  return false;
}


bool Car::CheckCollision(const ICollidable* collidable) const
{
  Rectangle carBBox = GetBoundingBox();
  Vector2 corners[4] = {
      {carBBox.x, carBBox.y},
      {carBBox.x + carBBox.width, carBBox.y},
      {carBBox.x + carBBox.width, carBBox.y + carBBox.height},
      {carBBox.x, carBBox.y + carBBox.height}};

  // 旋轉角點
  for (int i = 0; i < 4; i++)
  {
    Vector2 rotated = Vector2Rotate(Vector2Subtract(corners[i], position), rotation);
    corners[i] = Vector2Add(rotated, position);
  }

  auto bound = collidable->GetBoundBox();

  // 檢查每個角點是否在障礙物內
  for (int i = 0; i < 4; i++)
  {
    if (CheckCollisionPointRec(corners[i], bound))
    {
      return true;
    }
  }

  // 檢查障礙物的角點是否在車輛內
  Vector2 obstacleCorners[4] = {
      {bound.x, bound.y},
      {bound.x + bound.width, bound.y},
      {bound.x + bound.width, bound.y + bound.height},
      {bound.x, bound.y + bound.height}};

  for (int i = 0; i < 4; i++)
  {
    if (CheckCollisionPointPoly(obstacleCorners[i], corners, 4))
    {
      return true;
    }
  }

  return false;
}


float Car::DetectObstacleAtAngle(const std::vector<ICollidable*> &collidables, float angle) const
{
  const float MAX_DETECTION_DISTANCE = 1000.0f;
  Vector2 carPos = {position.x, position.y};
  float globalAngle = rotation + angle;

  Vector2 rayEnd = {
      carPos.x + MAX_DETECTION_DISTANCE * cos(globalAngle),
      carPos.y + MAX_DETECTION_DISTANCE * sin(globalAngle)};

  float closestDistance = MAX_DETECTION_DISTANCE;

  for (const auto &collidable : collidables)
  {
    Rectangle obstacleRect = collidable->GetBoundBox();
    Vector2 collisionPoint = {0, 0};

    if (CheckCollisionLineRec(carPos, rayEnd, obstacleRect, &collisionPoint))
    {
      float distance = Vector2Distance(carPos, collisionPoint);
      if (distance < closestDistance)
      {
        closestDistance = distance;
      }
    }
  }

  return closestDistance;
}


std::vector<float> Car::GetSensorData(const std::vector<ICollidable*> &collidables) const
{
  std::vector<float> sensorData;
  std::vector<float> angles = {-PI / 6, -PI / 3, 0, PI / 3, PI / 6}; // -60°, -30°, 0°, 30°, 60°

  for (float angle : angles)
  {
    sensorData.push_back(DetectObstacleAtAngle(collidables, angle));
  }

  return sensorData;
}


void Car::Update(std::vector<ICollidable*> &collidables)
{
  // Apply friction
  speed *= 0.98f;

  // Update steering
  if (std::abs(steering) < 0.01f)
    steering = 0;
  else
    steering *= 0.9f; // Return to center

  // Update rear axle position (this is the rotation center)
  rearAxlePosition.x += speed * std::cos(rotation);
  rearAxlePosition.y += speed * std::sin(rotation);

  // Update vehicle rotation
  if (std::abs(steering) > 0.001f)
  {
    float turningRadius = wheelBase / std::tan(std::abs(steering));
    float angularVelocity = speed / turningRadius;
    rotation += (steering > 0 ? angularVelocity : -angularVelocity);
  }

  // Update vehicle position (body center)
  position.x = rearAxlePosition.x + wheelBase / 2 * std::cos(rotation);
  position.y = rearAxlePosition.y + wheelBase / 2 * std::sin(rotation);

  for (const auto &collidable : collidables)
  {
    //if (collidable->CheckCollision(this->GetBoundingBox()))
    if (this->CheckCollision(collidable))
    {
      speed = -speed * 0.5;
    }
  }
}


// std::vector<float> Car::GetState(const std::vector<Obstacle> &obstacles) const
// {
//   std::vector<float> state;

//   // 添加車輛位置
//   state.push_back(position.x);
//   state.push_back(position.y);

//   // 添加車輛速度
//   state.push_back(speed);

//   // 添加車輛方向
//   state.push_back(rotation);

//   // 添加傳感器數據
//   std::vector<float> sensorData = GetSensorData(obstacles);
//   state.insert(state.end(), sensorData.begin(), sensorData.end());

//   return state;
// }


void Car::Draw()
{
  // Draw car body
  DrawRectanglePro(
      Rectangle{position.x, position.y, wheelBase, wheelTrack},
      Vector2{wheelBase / 2, wheelTrack / 2},
      rotation * RAD2DEG,
      RED);

  // Calculate wheel positions
  float halfWheelTrack = wheelTrack / 2;

  Vector2 fl = {
      rearAxlePosition.x + (wheelBase + frontWheelOffset) * std::cos(rotation) - halfWheelTrack * std::sin(rotation),
      rearAxlePosition.y + (wheelBase + frontWheelOffset) * std::sin(rotation) + halfWheelTrack * std::cos(rotation)};
  Vector2 fr = {
      rearAxlePosition.x + (wheelBase + frontWheelOffset) * std::cos(rotation) + halfWheelTrack * std::sin(rotation),
      rearAxlePosition.y + (wheelBase + frontWheelOffset) * std::sin(rotation) - halfWheelTrack * std::cos(rotation)};
  Vector2 rl = {
      rearAxlePosition.x + rearWheelOffset * std::cos(rotation) - halfWheelTrack * std::sin(rotation),
      rearAxlePosition.y + rearWheelOffset * std::sin(rotation) + halfWheelTrack * std::cos(rotation)};
  Vector2 rr = {
      rearAxlePosition.x + rearWheelOffset * std::cos(rotation) + halfWheelTrack * std::sin(rotation),
      rearAxlePosition.y + rearWheelOffset * std::sin(rotation) - halfWheelTrack * std::cos(rotation)};

  // Draw wheels
  float wheelRotation = rotation + steering;
  DrawRectanglePro(Rectangle{fl.x, fl.y, wheelLength, wheelWidth}, Vector2{wheelLength / 2, wheelWidth / 2}, wheelRotation * RAD2DEG, BLACK);
  DrawRectanglePro(Rectangle{fr.x, fr.y, wheelLength, wheelWidth}, Vector2{wheelLength / 2, wheelWidth / 2}, wheelRotation * RAD2DEG, BLACK);
  DrawRectanglePro(Rectangle{rl.x, rl.y, wheelLength, wheelWidth}, Vector2{wheelLength / 2, wheelWidth / 2}, rotation * RAD2DEG, BLACK);
  DrawRectanglePro(Rectangle{rr.x, rr.y, wheelLength, wheelWidth}, Vector2{wheelLength / 2, wheelWidth / 2}, rotation * RAD2DEG, BLACK);

  // Update and draw trails
  flTrail.Update(fl);
  frTrail.Update(fr);
  rlTrail.Update(rl);
  rrTrail.Update(rr);

  flTrail.Draw();
  frTrail.Draw();
  rlTrail.Draw();
  rrTrail.Draw();
}


void Car::DrawBoundingBox() const
{
  Rectangle bbox = GetBoundingBox();
  Vector2 corners[4] = {
      {bbox.x, bbox.y},
      {bbox.x + bbox.width, bbox.y},
      {bbox.x + bbox.width, bbox.y + bbox.height},
      {bbox.x, bbox.y + bbox.height}};

  for (int i = 0; i < 4; i++)
  {
    corners[i] = Vector2Rotate(Vector2Subtract(corners[i], position), rotation);
    corners[i] = Vector2Add(corners[i], position);
  }

  for (int i = 0; i < 4; i++)
  {
    DrawLineV(corners[i], corners[(i + 1) % 4], GREEN);
  }
}


void Car::DrawSensors(const std::vector<ICollidable*> &collidables) const
{
  Vector2 carPos = {position.x, position.y};
  std::vector<float> angles = {-PI / 6, -PI / 3, 0, PI / 3, PI / 6};

  for (float angle : angles)
  {
    float globalAngle = rotation + angle;
    float distance = DetectObstacleAtAngle(collidables, angle);
    Vector2 rayEnd = {
        carPos.x + distance * cos(globalAngle),
        carPos.y + distance * sin(globalAngle)};

    DrawLineV(carPos, rayEnd, RED);
  }
}
