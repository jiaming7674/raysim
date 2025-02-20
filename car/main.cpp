#include "raylib.h"
#include "raymath.h"
#include <vector>
#include <cmath>
#include "car.h"
//#include "track.h"
#include "obstacle.h"

const int SCREEN_WIDTH = 1440;
const int SCREEN_HEIGHT = 900;


int main()
{
  InitWindow(SCREEN_WIDTH, SCREEN_HEIGHT, "C++ Four-Wheel Car Simulation");

  std::vector<Obstacle*> obstacles;
  obstacles.push_back(new Obstacle(300, 300, 100, 100, BLUE));
  obstacles.push_back(new Obstacle(1200, 500, 150, 150, GREEN));

  std::vector<ICollidable*> collidables;
  for (auto collidable : obstacles) {
    collidables.push_back(collidable);
  }

  //Track track;
  //track.LoadTrack("track.txt");

  Vector2 position = {SCREEN_WIDTH / 2.0f, SCREEN_HEIGHT / 2.0f};
  Car car(position);

  RenderTexture2D renderTexture = LoadRenderTexture(SCREEN_WIDTH, SCREEN_HEIGHT);

  SetTargetFPS(60);

  while (!WindowShouldClose())
  {
    // Handle input
    if (IsKeyDown(KEY_UP))    car.Accelerate(1.0f);
    if (IsKeyDown(KEY_DOWN))  car.Accelerate(-1.0f);
    if (IsKeyDown(KEY_LEFT))  car.Steer(1.0f);
    if (IsKeyDown(KEY_RIGHT)) car.Steer(-1.0f);

    // Update and draw
    car.Update(collidables);

    // if (track.CheckCollision(car)) {
    //     // Handle collision (e.g., reset car position, reduce speed, etc.)
    // }

    std::vector<float> sensorData = car.GetSensorData(collidables);
    //std::vector<float> current_state = car.GetState(obstacles);


    BeginTextureMode(renderTexture);
    ClearBackground(RAYWHITE);
    //track.Draw();
    car.Draw();
    car.DrawBoundingBox();
    car.DrawSensors(collidables);

    for (const auto &obstacle : obstacles)
    {
      obstacle->Draw();
    }
    EndTextureMode();

    BeginDrawing();
    //DrawTexture(renderTexture.texture, 0, 0, WHITE);
    DrawTextureEx(renderTexture.texture, (Vector2){0,0}, 0, 1, WHITE);
    EndDrawing();
  }

  CloseWindow();

  return 0;
}