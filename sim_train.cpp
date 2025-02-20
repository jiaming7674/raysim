/*******************************************************************************************
 *
 *   Train Simulation using raylib and C++
 *
 *   Description:
 *     - A train composed of 3 carriages will automatically travel along a randomly generated
 *       closed-loop track.
 *     - The track is built from a series of straight segments and circular arc segments (rounded
 *       corners) derived from a randomly generated convex polygon.
 *     - The player can control the train's speed using the UP (accelerate) and DOWN (decelerate) keys.
 *
 *   Compilation (example using g++ on Linux):
 *     g++ -std=c++11 -O2 -o train_simulation train_simulation.cpp -lraylib -lopengl32 -lgdi32 -lwinmm
 *
 ********************************************************************************************/

#include "raylib.h"
#include "raymath.h"
#include <vector>
#include <cmath>
#include <cstdlib>
#include <ctime>
#include <stdio.h>

// Constants for window and track
const int screenWidth = 1200;
const int screenHeight = 800;

//-----------------------------------------------------------------------------
// Data structures for track segments

// 定義段落類型：直線段或弧形段
enum SegmentType
{
  STRAIGHT,
  ARC
};

// 每個段落會儲存必要參數與長度
struct Segment
{
  SegmentType type;
  float length;
  // For straight segment:
  Vector2 start;
  Vector2 end;
  // For arc segment:
  Vector2 center;
  float radius;
  float startAngle; // 起始角度 (弧度)
  float arcAngle;   // 弧形的角度（正值表示逆時針）
};

//-----------------------------------------------------------------------------
// 結構用於存放每個頂點處的圓角資料
struct VertexRound
{
  Vector2 S;         // 進入圓角的起點（沿上一邊線段）
  Vector2 E;         // 離開圓角的終點（沿下一邊線段）
  Vector2 arcCenter; // 圓弧中心
  float R;           // 圓弧半徑
  float startAngle;  // 圓弧起始角度
  float arcAngle;    // 圓弧掃過角度 (通常 = PI - interiorAngle)
};

//-----------------------------------------------------------------------------
// 全域變數：儲存軌道段落與總長度
std::vector<Segment> trackSegments;
float trackLength = 0.0f;

//-----------------------------------------------------------------------------
// 輔助函式：夾住數值於 -1 ~ 1 範圍
float ClampFloat(float value, float minVal, float maxVal)
{
  if (value < minVal)
    return minVal;
  if (value > maxVal)
    return maxVal;
  return value;
}

// 輔助函式：線性插值
float LerpFloat(float a, float b, float t)
{
  return a + t * (b - a);
}

//-----------------------------------------------------------------------------
// Function: GenerateTrack()
// 說明: 隨機生成一個閉環軌道（由直線與圓弧段組成）
//-----------------------------------------------------------------------------
void GenerateTrack()
{
  trackSegments.clear();
  trackLength = 0.0f;

  // 產生一個隨機凸多邊形作為軌道基底
  const int numVertices = 8; // 可依需求調整頂點數目
  std::vector<Vector2> vertices(numVertices);

  // 設定多邊形的中心與半徑參數
  Vector2 center = {screenWidth / 2.0f, screenHeight / 2.0f};
  float baseRadius = 300.0f;     // 基本半徑
  float radiusVariation = 150.0f; // 半徑隨機變化

  // 為了產生凸多邊形，依序以角度排序（在此用固定間隔再加上微調）
  for (int i = 0; i < numVertices; i++)
  {
    float angle = (2 * PI / numVertices) * i + ((std::rand() % 21 - 10) * PI / 180.0f); // ±10° 隨機偏移
    float radius = baseRadius + (std::rand() % (int)(radiusVariation * 2)) - radiusVariation;
    vertices[i] = {center.x + radius * cosf(angle), center.y + radius * sinf(angle)};
  }

  // 計算每個頂點處的圓角資料
  std::vector<VertexRound> vertexRounds(numVertices);

  for (int i = 0; i < numVertices; i++)
  {
    // 取得前一個、當前與下一個頂點（環狀）
    Vector2 prev = vertices[(i - 1 + numVertices) % numVertices];
    Vector2 curr = vertices[i];
    Vector2 next = vertices[(i + 1) % numVertices];

    // 計算從當前頂點出發指向前一點和下一點的單位向量
    Vector2 d1 = Vector2Normalize({prev.x - curr.x, prev.y - curr.y});
    Vector2 d2 = Vector2Normalize({next.x - curr.x, next.y - curr.y});

    // 計算兩向量間的夾角（內角）
    float dotVal = ClampFloat(Vector2DotProduct(d1, d2), -1.0f, 1.0f);
    float theta = acosf(dotVal); // 夾角（弧度）

    // 判斷當前頂點是凸角還是凹角
    // 依據 cross product 與整體多邊形方向來判斷    
    float cross = d1.x * d2.y - d1.y * d2.x;
    bool convex = (cross >= 0) ? true : false;

    // fillet（圓角）的設計參數
    // 我們希望選擇一個 fillet 半徑 R，並依據公式 d = R / tan(theta/2)
    float desiredR = 80.0f; // 希望的 fillet 半徑
    float dCandidate = desiredR / tanf(theta / 2.0f);

    // 限制 d 不超過邊長的一定比例，避免圓角過大
    float l1 = Vector2Distance(curr, prev);
    float l2 = Vector2Distance(curr, next);
    float maxD1 = l1 * 0.3f;
    float maxD2 = l2 * 0.3f;
    float d = dCandidate;
    if (d > maxD1 || d > maxD2)
    {
      d = (maxD1 < maxD2) ? maxD1 : maxD2;
      desiredR = d * tanf(theta / 2.0f);
    }
    // 記錄 fillet 半徑 R
    float R = desiredR;

    // 計算圓角起點與終點：位於從頂點沿 d1 與 d2 的方向上距離 d 處
    Vector2 S = {curr.x + d * d1.x, curr.y + d * d1.y}; // 進入圓角的點（位於上一邊線段上）
    Vector2 E = {curr.x + d * d2.x, curr.y + d * d2.y}; // 離開圓角的點（位於下一邊線段上）

    // 計算角平分線（bisector）
    Vector2 bisector = Vector2Normalize({d1.x + d2.x, d1.y + d2.y});
    // 根據幾何關係，從頂點到圓弧中心的距離 L = R / sin(theta/2)
    float L = R / sinf(theta / 2.0f);
    Vector2 arcCenter = {curr.x + L * bisector.x, curr.y + L * bisector.y};

    // 計算圓弧起始角度
    float startAngle = atan2f(S.y - arcCenter.y, S.x - arcCenter.x);
    // 計算圓弧掃過角度。標準 fillet 的圓弧角度 = PI - interiorAngle
    float arcAngle = convex ? theta - PI : PI - theta;

    // 儲存此頂點的圓角資料
    vertexRounds[i] = {S, E, arcCenter, R, startAngle, arcAngle};
  }

  // 接下來建立軌道段落：每個邊由「直線段」與「圓弧段」組成
  // 順序為：上一個頂點的圓角終點 -> 當前頂點的圓角起點（直線段），接著當前頂點的圓弧段
  for (int i = 0; i < numVertices; i++)
  {
    int nextIndex = (i + 1) % numVertices;
    // 直線段：從當前頂點的圓角終點到下一頂點的圓角起點
    Segment segStraight;
    segStraight.type = STRAIGHT;
    segStraight.start = vertexRounds[i].E;
    segStraight.end = vertexRounds[nextIndex].S;
    segStraight.length = Vector2Distance(segStraight.start, segStraight.end);
    trackSegments.push_back(segStraight);
    trackLength += segStraight.length;

    // 圓弧段：下一頂點的圓角，由其 S 為起始角，角度為 arcAngle
    Segment segArc;
    segArc.type = ARC;
    segArc.center = vertexRounds[nextIndex].arcCenter;
    segArc.radius = vertexRounds[nextIndex].R;
    segArc.startAngle = vertexRounds[nextIndex].startAngle;
    segArc.arcAngle = vertexRounds[nextIndex].arcAngle; // 假設 arcAngle 為正（逆時針方向）
    segArc.length = fabs(segArc.arcAngle) * segArc.radius;
    trackSegments.push_back(segArc);
    trackLength += segArc.length;
  }
}

//-----------------------------------------------------------------------------
// Function: GetPositionAndTangent()
// 說明: 給定軌道上的距離 dist (0 ~ trackLength)，計算此處的座標與切線方向
//-----------------------------------------------------------------------------
void GetPositionAndTangent(float dist, Vector2 &pos, Vector2 &tangent)
{
  float d = dist;
  // 依序檢查每一段
  for (size_t i = 0; i < trackSegments.size(); i++)
  {
    if (d > trackSegments[i].length)
    {
      d -= trackSegments[i].length;
    }
    else
    {
      if (trackSegments[i].type == STRAIGHT)
      {
        // 線性插值
        float t = d / trackSegments[i].length;
        pos.x = LerpFloat(trackSegments[i].start.x, trackSegments[i].end.x, t);
        pos.y = LerpFloat(trackSegments[i].start.y, trackSegments[i].end.y, t);
        // 切線方向即為直線方向
        tangent = Vector2Normalize({trackSegments[i].end.x - trackSegments[i].start.x, trackSegments[i].end.y - trackSegments[i].start.y});
      }
      else // ARC
      {
        // 根據弧長比例計算當前角度
        float fraction = d / trackSegments[i].length;
        float angle = trackSegments[i].startAngle + fraction * trackSegments[i].arcAngle;
        pos.x = trackSegments[i].center.x + cosf(angle) * trackSegments[i].radius;
        pos.y = trackSegments[i].center.y + sinf(angle) * trackSegments[i].radius;
        // 切線為半徑的垂直向量 (此處假設 arc 為逆時針方向)
        tangent = Vector2Normalize({-sinf(angle), cosf(angle)});
      }
      return;
    }
  }
  // 如果超過軌道總長度則回到起點（不太可能發生，因為會做模環處理）
  pos = trackSegments.back().end;
  tangent = {0, 0};
}

//-----------------------------------------------------------------------------
// 主程式
//-----------------------------------------------------------------------------
int main(void)
{
  // 初始化亂數種子
  std::srand((unsigned int)time(0));

  // 初始化視窗
  InitWindow(screenWidth, screenHeight, "Train Simulation with raylib and C++");

  // 產生軌道
  GenerateTrack();

  // 火車參數
  float trainPos = 0.0f;   // 火車前端在軌道上的距離位置
  float trainSpeed = 0.0f; // 當前速度（單位：像素/秒）
  const float maxSpeed = 180.0f;
  const float acceleration = 80.0f;  // 加速值
  const float deceleration = 100.0f; // 減速值
  const float friction = 1.0f;      // 自動減速

  // 火車車廂參數
  const int numCars = 8;
  const float carSpacing = 35.0f;   // 車廂間距（前後車之間的距離）
  const Vector2 carSize = {30, 15}; // 車廂大小

  bool resetTrack = false;

  SetTargetFPS(60);

  while (!WindowShouldClose()) // 主遊戲迴圈
  {
    if (IsKeyDown(KEY_R)) {
      // 重新產生軌道
      if (resetTrack == false) GenerateTrack();
      resetTrack = true;
    }
    else resetTrack = false;

    float dt = GetFrameTime();

    // --- 更新邏輯 ---
    // 處理玩家輸入：上下鍵控制加減速
    if (IsKeyDown(KEY_UP))
    {
      trainSpeed += acceleration * dt;
    }
    else if (IsKeyDown(KEY_DOWN))
    {
      trainSpeed -= deceleration * dt;
    }
    else
    {
      // 無輸入時以摩擦力自動降低速度
      if (trainSpeed > 0)
        trainSpeed -= friction * dt;
      else if (trainSpeed < 0)
        trainSpeed += friction * dt;
    }
    // 限制速度範圍
    if (trainSpeed > maxSpeed)
      trainSpeed = maxSpeed;
    if (trainSpeed < 0)
      trainSpeed = 0;

    // 更新火車前端在軌道上的位置
    trainPos += trainSpeed * dt;
    // 若超過總長度則循環
    if (trainPos >= trackLength)
      trainPos -= trackLength;
    if (trainPos < 0)
      trainPos += trackLength;

    // --- 繪圖 ---
    BeginDrawing();
    ClearBackground(RAYWHITE);

    // 畫出軌道：將每個段落畫出來
    for (size_t i = 0; i < trackSegments.size(); i++)
    {
      if (trackSegments[i].type == STRAIGHT)
      {
        DrawLineV(trackSegments[i].start, trackSegments[i].end, BLACK);
      }
      else // ARC 段落，使用多點逼近繪出
      {
        const int numArcPoints = 20;
        for (int j = 0; j < numArcPoints; j++)
        {
          float t1 = (float)j / numArcPoints;
          float t2 = (float)(j + 1) / numArcPoints;
          float angle1 = trackSegments[i].startAngle + t1 * trackSegments[i].arcAngle;
          float angle2 = trackSegments[i].startAngle + t2 * trackSegments[i].arcAngle;
          Vector2 p1 = {trackSegments[i].center.x + cosf(angle1) * trackSegments[i].radius,
                        trackSegments[i].center.y + sinf(angle1) * trackSegments[i].radius};
          Vector2 p2 = {trackSegments[i].center.x + cosf(angle2) * trackSegments[i].radius,
                        trackSegments[i].center.y + sinf(angle2) * trackSegments[i].radius};
          DrawLineV(p1, p2, BLACK);
        }
      }
    }

    // 畫出火車（每個車廂）
    for (int i = 0; i < numCars; i++)
    {
      // 每個車廂沿軌道的位置為：trainPos - i * carSpacing (做循環處理)
      float posOnTrack = trainPos - i * carSpacing;
      while (posOnTrack < 0)
        posOnTrack += trackLength;
      while (posOnTrack >= trackLength)
        posOnTrack -= trackLength;

      Vector2 carPos;
      Vector2 carTangent;
      GetPositionAndTangent(posOnTrack, carPos, carTangent);
      // 計算車廂旋轉角度（由切線方向決定）
      float rotation = atan2f(carTangent.y, carTangent.x) * 180.0f / PI;
      // 使用 DrawRectanglePro 畫出旋轉過的車廂
      Rectangle carRect = {carPos.x, carPos.y, carSize.x, carSize.y};
      DrawRectanglePro(carRect, {carSize.x / 2, carSize.y / 2}, rotation, RED);
    }

    // 顯示速度資訊
    DrawText(TextFormat("Speed: %.1f", trainSpeed), 10, 10, 20, DARKGRAY);

#ifdef __DEBUG__
    // -----------------
    // 偵錯視覺化：
    // 對每個弧段，繪製圓心、圓弧完整圓周、以及弧的起點與終點
    // -----------------
    for (size_t i = 0; i < trackSegments.size(); i++)
    {
      if (trackSegments[i].type == ARC)
      {
        // 繪製圓心（藍色小圓點）
        DrawCircleV(trackSegments[i].center, 4, BLUE);

        // 繪製圓弧所在完整圓的外框（淺灰色）
        DrawCircleLines(trackSegments[i].center.x, trackSegments[i].center.y, trackSegments[i].radius, LIGHTGRAY);

        // 計算弧線的起點與終點
        Vector2 arcStart = {trackSegments[i].center.x + cosf(trackSegments[i].startAngle) * trackSegments[i].radius,
                            trackSegments[i].center.y + sinf(trackSegments[i].startAngle) * trackSegments[i].radius};
        Vector2 arcEnd = {trackSegments[i].center.x + cosf(trackSegments[i].startAngle + trackSegments[i].arcAngle) * trackSegments[i].radius,
                          trackSegments[i].center.y + sinf(trackSegments[i].startAngle + trackSegments[i].arcAngle) * trackSegments[i].radius};

        // 繪製起點（綠色）與終點（紅色）
        DrawCircleV(arcStart, 4, GREEN);
        DrawCircleV(arcEnd, 4, RED);

        // 繪製從圓心到起點與終點的連線（藍色）
        DrawLineV(trackSegments[i].center, arcStart, BLUE);
        DrawLineV(trackSegments[i].center, arcEnd, BLUE);
      }
    }  
#endif  

    EndDrawing();
  }

  CloseWindow();
  return 0;
}
