#include "MetaBallWatchy.h"
#include <VectorXf.h>

const float VOLTAGE_MIN = 3.5;
const float VOLTAGE_MAX = 4.2;
const float VOLTAGE_WARNING = 3.6;
const float VOLTAGE_RANGE = VOLTAGE_MAX - VOLTAGE_MIN;

MetaBallWatchy::MetaBallWatchy(const watchySettings& s) : Watchy(s)
{
  //Serial.begin(115200);
//while (!Serial); // wait for serial port to connect. Needed for native USB port on Arduino only

}

static bool getColor(const int16_t& x, const int16_t& y, const uint16_t& color) 
{
  return color > BlueNoise200[y * 200 + x];
}

static bool getColor2(int16_t x, int16_t y, int16_t xUv, int16_t yUv, const uint8_t *bitmap, int16_t w, int16_t h) 
{
  return getColor(x, y, bitmap[yUv * w + xUv]);
}

static bool getColor3(const int16_t& x, const int16_t& y, const int16_t& xUv, const int16_t& yUv, const uint8_t *bitmap, const int16_t& w, const int16_t& h) 
{
  return getColor(x,y,bitmap[yUv * w + xUv]);
}

static float clamp(float val, const float& min, const float& max)
{
  if (val > max)
    val = max;
  
  if (val < min)
    val = min;

  return val;
}

float MetaBallWatchy::getBatteryFill()
{
  float VBAT = getBatteryVoltage();

  float batState = ((VBAT - VOLTAGE_MIN) / VOLTAGE_RANGE);

  return clamp(batState, 0.0f, 1.0f);
}

static float unlerp(const float& value, const float& min, const float& max) {
  return (value - min) / (max - min);
}

static float smoothstep(const float& x) {
  // Evaluate polynomial
  return x * x * (3 - 2 * x);
}

static Vec2f ClosestPointOnSegment(const Vec2f& s1, const Vec2f& s2, const Vec2f& p, float& unlerp)
{
  Vec2f difference = s2 - s1;
  float sqrMagnitude = difference.lengthSquared();

  if (sqrMagnitude > 0.0f)
    unlerp = (p - s1).dot(difference) / sqrMagnitude;
  else
    unlerp = 0.0f;

  unlerp = clamp(unlerp, 0.0f, 1.0f);
  return s1 + difference * unlerp;
}

static Vec2f ClosestPointOnSegment(const Vec2f& s1, const Vec2f& s2, const Vec2f& p)
{
  float unlerp = 0.0f;
  return ClosestPointOnSegment(s1, s2, p, unlerp);
}

static Vec2f ClosestPointOnArc(Vec2f center, float radius, float angleStart, float angle, Vec2f p)
{
  Vec2f diff = p - center;
  Vec2f start (radius, 0.0f);
  start.rotate(angleStart);

  float angleToPoint = start.angle(diff);
  float angleToPointPositive = angleToPoint;

  if (angleToPointPositive < 0.0f)
    angleToPointPositive += 360.0f;

  float angleSigned = angle;

  if (angleSigned > 180.0f)
    angleSigned -= 360.0f;

  if (angleToPointPositive > 0.0f && angleToPointPositive <= angle)
    return center + diff.getNormalized() * radius;

  if ((angleSigned > 0.0f || angleSigned < 0.0f && angleToPoint < angleSigned * 0.5f) && angleToPointPositive > angle)
    return center + start.getRotated(angle);
  
  return center + start;
}

static Vec2f ClosestPointOnArc(const Vec2f& center, const Vec2f& start, const Vec2f& end, const float& radius, const float& angleStart, const float& angle, const Vec2f& p)
{
  Vec2f diff = p - center;

  float angleToPoint = start.angle(diff);
  float angleToPointPositive = angleToPoint;

  if (angleToPointPositive < 0.0f)
    angleToPointPositive += 360.0f;

  float angleSigned = angle;

  if (angleSigned > 180.0f)
    angleSigned -= 360.0f;

  if (angleToPointPositive > 0.0f && angleToPointPositive <= angle)
    return center + diff.getNormalized() * radius;

  if ((angleSigned >= 0.0f || angleSigned < 0.0f && angleToPoint < angleSigned * 0.5f) && angleToPointPositive > angle)
    return center + end * radius;
  
  return center + start * radius;
}

// ToDo return minimal distance for calculating normals when blending

static void MetaBall(const Vec2f& currentPos, const Vec2f& circleCenter, const float& radius, const float& extraRadius, int& count, float& totalDistance, Vec2f& totalNormal, float& prevRadius, Vec2f& prevCenter)
{
  Vec2f offset = currentPos - circleCenter;
  float totalRadus = radius + extraRadius;
  float lengthSqr = offset.lengthSquared();

  if (lengthSqr >= totalRadus * totalRadus)
    return;

  float distance = sqrtf(lengthSqr);
  float newDistance = totalRadus - distance;

  if (newDistance >= extraRadius)
  {
    newDistance = smoothstep(unlerp(newDistance, extraRadius, totalRadus) * 0.5f + 0.5f) * 2.0f;
  }
  else
  {
    newDistance = smoothstep(unlerp(newDistance, 0, extraRadius) * 0.5f) * 2.0f;
  }

  // if (distance > 0.0f)
  //   newDistance = (radius) / distance;

  count++;
  totalDistance += newDistance;
  Vec2f normal = offset / radius;

  if (count == 1)
  {
    prevRadius = radius;
    prevCenter = circleCenter;
    totalNormal = normal;
    return;
  }

  // ToDo: fix
  //float weight = newDistance / (totalDistance);
  //float otherWeight = 1.0f - weight;

  //Vec2f center = circleCenter * weight + prevCenter * otherWeight;
  float weight = 0.0f;
  Vec2f center = ClosestPointOnSegment(circleCenter, prevCenter, currentPos, weight);
  float otherWeight = 1.0f - weight;
  Vec2f newOffset = currentPos - center;
  //radius = newOffset.length();
  //float newRadius = radius * weight + prevRadius * otherWeight;

  float newRadius = ((offset.getNormalized() * radius + (currentPos - prevCenter).getNormalized() * prevRadius) * 0.5f).length();
  
  Vec2f newNormal = newOffset / newRadius;
  totalNormal = newNormal;
  prevRadius = newRadius;

  /* float totalNormalLength = totalNormal.length();
  float normalLenght = normal.length();
  totalNormal = (totalNormal * otherWeight + weight * normal).getNormalized();
  float newLength = totalNormalLength * otherWeight + weight * normalLenght;

  if (newLength > 1.0f)
    newLength = 1.0f;

  totalNormal *= newLength; */

  prevCenter = center;
}

static void Arc(Vec2f currentPos, Vec2f center, float radius, float extraRadius, float arcRadius, float arcStartAngle, float arcAngle, int& count, float& totalDistance, Vec2f& totalNormal, float& prevRadius, Vec2f& prevCenter)
{
  Vec2f offset = currentPos - center;
  float outterRadius = radius + extraRadius + arcRadius;

  float lengthSquared = offset.lengthSquared();

  if (lengthSquared > outterRadius * outterRadius)
    return;
    
  float innerRadius = arcRadius - radius - extraRadius;

  if (innerRadius < 0.0f)
    innerRadius = 0.0f;

  if (lengthSquared < innerRadius * innerRadius)
    return;

  Vec2f closestToArc = ClosestPointOnArc(center, arcRadius, arcStartAngle, arcAngle, currentPos);
  MetaBall(currentPos, closestToArc, radius, extraRadius, count, totalDistance, totalNormal, prevRadius, prevCenter);
}

static void Arc(const Vec2f& currentPos, const Vec2f& center, const Vec2f& start, const Vec2f& end, const float& radius, const float& extraRadius,
 const float& arcRadius, const float& arcStartAngle, const float& arcAngle,
  int& count, float& totalDistance, Vec2f& totalNormal, float& prevRadius, Vec2f& prevCenter)
{
  Vec2f offset = currentPos - center;
  float outterRadius = radius + extraRadius + arcRadius;

  float lengthSquared = offset.lengthSquared();

  if (lengthSquared >= outterRadius * outterRadius)
    return;
    
  float innerRadius = arcRadius - radius - extraRadius;

  if (innerRadius < 0.0f)
    innerRadius = 0.0f;

  if (lengthSquared <= innerRadius * innerRadius)
    return;

  Vec2f closestToArc = ClosestPointOnArc(center, start, end, arcRadius, arcStartAngle, arcAngle, currentPos);
  MetaBall(currentPos, closestToArc, radius, extraRadius, count, totalDistance, totalNormal, prevRadius, prevCenter);
}

static void Draw0(const Vec2f& currentPos, const Vec2f& center, const float& size, const float& extraRadius,
 int& count, float& totalDistance, Vec2f& totalNormal, float& prevRadius, Vec2f& prevCenter)
{
  float radius = size * 0.5f;
  Vec2f s1 = center + Vec2f(0.0f, radius);
  Vec2f s2 = center + Vec2f(0.0f, -radius);

  Vec2f closestPoint = ClosestPointOnSegment(s1, s2, currentPos);
  MetaBall(currentPos, closestPoint, radius, extraRadius, count, totalDistance, totalNormal, prevRadius, prevCenter);
}

static void Draw1(const Vec2f& currentPos, const Vec2f& center, const float& size, const float& extraRadius,
int& count, float& totalDistance, Vec2f& totalNormal, float& prevRadius, Vec2f& prevCenter)
{
  float radiusPercentage = 0.2f;
  float halfOffset = (0.5f - radiusPercentage);
  float offset = (1.0f - radiusPercentage);
  float radius = size * radiusPercentage;
  Vec2f s1 = center + Vec2f(size * halfOffset, -size * offset);
  Vec2f s2 = center + Vec2f(size * halfOffset, size * offset);
  Vec2f s3 = center + Vec2f(-size * halfOffset, -size * 0.2f);

  Vec2f closestPoint = ClosestPointOnSegment(s1, s2, currentPos);
  MetaBall(currentPos, closestPoint, radius, extraRadius, count, totalDistance, totalNormal, prevRadius, prevCenter);
  
  closestPoint = ClosestPointOnSegment(s1, s3, currentPos);
  MetaBall(currentPos, closestPoint, radius, extraRadius, count, totalDistance, totalNormal, prevRadius, prevCenter);
}

const float DRAW_2_ARC_START_ANGLE = 180.0f;
const Vec2f DRAW_2_ARC_START = Vec2f(cos(DRAW_2_ARC_START_ANGLE * DEG_TO_RAD), sin(DRAW_2_ARC_START_ANGLE * DEG_TO_RAD));

const float DRAW_2_ARC_END_ANGLE = 200.0f;
const float DRAW_2_ARC_END_ANGLE_SUM = DRAW_2_ARC_START_ANGLE + DRAW_2_ARC_END_ANGLE;
const Vec2f DRAW_2_ARC_END = Vec2f(cos(DRAW_2_ARC_END_ANGLE_SUM * DEG_TO_RAD), sin(DRAW_2_ARC_END_ANGLE_SUM * DEG_TO_RAD));

static void Draw2(const Vec2f& currentPos, const Vec2f& center, const float& size, const float& extraRadius,
int& count, float& totalDistance, Vec2f& totalNormal, float& prevRadius, Vec2f& prevCenter)
{
  float radiusPercentage = 0.2f;
  float halfOffset = (0.5f - radiusPercentage);
  float offset = (1.0f - radiusPercentage);
  float radius = size * radiusPercentage;
  Vec2f s2 = center + Vec2f(size * 0.2f, -size * 0.2f);
  Vec2f s3 = center + Vec2f(-size * halfOffset, size * offset);
  Vec2f s4 = center + Vec2f(size * halfOffset, size * offset);

  Arc(currentPos, center + Vec2f(0.0f, -size * 0.5f), DRAW_2_ARC_START, DRAW_2_ARC_END, radius, extraRadius, size * halfOffset, DRAW_2_ARC_START_ANGLE, DRAW_2_ARC_END_ANGLE, count, totalDistance, totalNormal, prevRadius, prevCenter);

  Vec2f closestPoint = ClosestPointOnSegment(s2, s3, currentPos);
  MetaBall(currentPos, closestPoint, radius, extraRadius, count, totalDistance, totalNormal, prevRadius, prevCenter);
  
  closestPoint = ClosestPointOnSegment(s3, s4, currentPos);
  MetaBall(currentPos, closestPoint, radius, extraRadius, count, totalDistance, totalNormal, prevRadius, prevCenter);
}


const float DRAW_3_ARC_START_ANGLE = 180.0f;
const Vec2f DRAW_3_ARC_START = Vec2f(cos(DRAW_3_ARC_START_ANGLE * DEG_TO_RAD), sin(DRAW_3_ARC_START_ANGLE * DEG_TO_RAD));

const float DRAW_3_ARC_END_ANGLE = 270.0f;
const float DRAW_3_ARC_END_ANGLE_SUM = DRAW_3_ARC_START_ANGLE + DRAW_3_ARC_END_ANGLE;
const Vec2f DRAW_3_ARC_END = Vec2f(cos(DRAW_3_ARC_END_ANGLE_SUM * DEG_TO_RAD), sin(DRAW_3_ARC_END_ANGLE_SUM * DEG_TO_RAD));

const float DRAW_3_2_ARC_START_ANGLE = 280.0f;
const Vec2f DRAW_3_2_ARC_START = Vec2f(cos(DRAW_3_2_ARC_START_ANGLE * DEG_TO_RAD), sin(DRAW_3_2_ARC_START_ANGLE * DEG_TO_RAD));

const float DRAW_3_2_ARC_END_ANGLE = 195.0f;
const float DRAW_3_2_ARC_END_ANGLE_SUM = DRAW_3_2_ARC_START_ANGLE + DRAW_3_2_ARC_END_ANGLE;
const Vec2f DRAW_3_2_ARC_END = Vec2f(cos(DRAW_3_2_ARC_END_ANGLE_SUM * DEG_TO_RAD), sin(DRAW_3_2_ARC_END_ANGLE_SUM * DEG_TO_RAD));

static void Draw3(const Vec2f& currentPos, const Vec2f& center, const float& size, const float& extraRadius,
int& count, float& totalDistance, Vec2f& totalNormal, float& prevRadius, Vec2f& prevCenter)
{
  float radiusPercentage = 0.2f;
  float halfOffset = (0.5f - radiusPercentage);
  float offset = (1.0f - radiusPercentage);
  float radius = size * radiusPercentage;
  
  Arc(currentPos, center + Vec2f(0.0f, -size * 0.5f), DRAW_3_ARC_START, DRAW_3_ARC_END, radius, extraRadius, size * halfOffset, DRAW_3_ARC_START_ANGLE, DRAW_3_ARC_END_ANGLE, count, totalDistance, totalNormal, prevRadius, prevCenter);
  
  Arc(currentPos, center + Vec2f(-size * 0.1f, size * 0.4f), DRAW_3_2_ARC_START, DRAW_3_2_ARC_END, radius, extraRadius, size * (halfOffset + 0.1f), DRAW_3_2_ARC_START_ANGLE, DRAW_3_2_ARC_END_ANGLE, count, totalDistance, totalNormal, prevRadius, prevCenter);
}

static void Draw4(const Vec2f& currentPos, const Vec2f& center, const float& size, const float& extraRadius,
int& count, float& totalDistance, Vec2f& totalNormal, float& prevRadius, Vec2f& prevCenter)
{
  float radiusPercentage = 0.2f;
  float halfOffset = (0.5f - radiusPercentage);
  float offset = (1.0f - radiusPercentage);
  float radius = size * radiusPercentage;
  Vec2f s1 = center + Vec2f(0.0f, -size * offset);
  Vec2f s2 = center + Vec2f(-size * halfOffset, size * 0.2f);
  Vec2f s3 = center + Vec2f(size * halfOffset, size * 0.2f);
  Vec2f s4 = center + Vec2f(size * 0.3f, -size * 0.1f);
  Vec2f s5 = center + Vec2f(size * 0.1f, size * offset);

  Vec2f closestPoint = ClosestPointOnSegment(s1, s2, currentPos);
  MetaBall(currentPos, closestPoint, radius, extraRadius, count, totalDistance, totalNormal, prevRadius, prevCenter);

  closestPoint = ClosestPointOnSegment(s2, s3, currentPos);
  MetaBall(currentPos, closestPoint, radius, extraRadius, count, totalDistance, totalNormal, prevRadius, prevCenter);

  closestPoint = ClosestPointOnSegment(s4, s5, currentPos);
  MetaBall(currentPos, closestPoint, radius, extraRadius, count, totalDistance, totalNormal, prevRadius, prevCenter);
}

const float DRAW_5_ARC_START_ANGLE = 235.0f;
const Vec2f DRAW_5_ARC_START = Vec2f(cos(DRAW_5_ARC_START_ANGLE * DEG_TO_RAD), sin(DRAW_5_ARC_START_ANGLE * DEG_TO_RAD));

const float DRAW_5_ARC_END_ANGLE = 250.0f;
const float DRAW_5_ARC_END_ANGLE_SUM = DRAW_5_ARC_START_ANGLE + DRAW_5_ARC_END_ANGLE;
const Vec2f DRAW_5_ARC_END = Vec2f(cos(DRAW_5_ARC_END_ANGLE_SUM * DEG_TO_RAD), sin(DRAW_5_ARC_END_ANGLE_SUM * DEG_TO_RAD));

static void Draw5(const Vec2f& currentPos, const Vec2f& center, const float& size, const float& extraRadius,
int& count, float& totalDistance, Vec2f& totalNormal, float& prevRadius, Vec2f& prevCenter)
{
  float radiusPercentage = 0.2f;
  float halfOffset = (0.5f - radiusPercentage);
  float offset = (1.0f - radiusPercentage);
  float radius = size * radiusPercentage;
  Vec2f s1 = center + Vec2f(size * halfOffset, -size * offset);
  Vec2f s2 = center + Vec2f(-size * halfOffset, -size * offset);
  Vec2f s3 = center + Vec2f(-size * halfOffset, 0.0f);

  Vec2f closestPoint = ClosestPointOnSegment(s1, s2, currentPos);
  MetaBall(currentPos, closestPoint, radius, extraRadius, count, totalDistance, totalNormal, prevRadius, prevCenter);
  
  closestPoint = ClosestPointOnSegment(s2, s3, currentPos);
  MetaBall(currentPos, closestPoint, radius, extraRadius, count, totalDistance, totalNormal, prevRadius, prevCenter);

  Arc(currentPos, center + Vec2f(-size * 0.1f, size * 0.4f), DRAW_5_ARC_START, DRAW_5_ARC_END, radius, extraRadius, size * (halfOffset + 0.1f), DRAW_5_ARC_START_ANGLE, DRAW_5_ARC_END_ANGLE, count, totalDistance, totalNormal, prevRadius, prevCenter);
}

static void Draw6(const Vec2f& currentPos, const Vec2f& center, const float& size, const float& extraRadius,
int& count, float& totalDistance, Vec2f& totalNormal, float& prevRadius, Vec2f& prevCenter)
{
  float radiusPercentage = 0.2f;
  float halfOffset = (0.5f - radiusPercentage);
  float offset = (1.0f - radiusPercentage);
  float radius = size * radiusPercentage;
  Vec2f s1 = center + Vec2f(size * halfOffset, -size * offset);
  Vec2f s2 = center + Vec2f(-size * halfOffset, size * 0.35f);
  Vec2f s3 = center + Vec2f(0.0f, size * 0.5f);

  Vec2f closestPoint = ClosestPointOnSegment(s1, s2, currentPos);
  MetaBall(currentPos, closestPoint, radius, extraRadius, count, totalDistance, totalNormal, prevRadius, prevCenter);
  
  MetaBall(currentPos, s3, size * 0.5f, extraRadius, count, totalDistance, totalNormal, prevRadius, prevCenter);
}

static void Draw7(const Vec2f& currentPos, const Vec2f& center, const float& size, const float& extraRadius,
int& count, float& totalDistance, Vec2f& totalNormal, float& prevRadius, Vec2f& prevCenter)
{
  float radiusPercentage = 0.2f;
  float halfOffset = (0.5f - radiusPercentage);
  float offset = (1.0f - radiusPercentage);
  float radius = size * radiusPercentage;
  Vec2f s1 = center + Vec2f(size * halfOffset, -size * offset);
  Vec2f s2 = center + Vec2f(-size * halfOffset, size * offset);
  Vec2f s3 = center + Vec2f(-size * halfOffset, -size * offset);

  Vec2f closestPoint = ClosestPointOnSegment(s1, s2, currentPos);
  MetaBall(currentPos, closestPoint, radius, extraRadius, count, totalDistance, totalNormal, prevRadius, prevCenter);
  
  closestPoint = ClosestPointOnSegment(s1, s3, currentPos);
  MetaBall(currentPos, closestPoint, radius, extraRadius, count, totalDistance, totalNormal, prevRadius, prevCenter);
}

static void Draw8(const Vec2f& currentPos, const Vec2f& center, const float& size, const float& extraRadius,
int& count, float& totalDistance, Vec2f& totalNormal, float& prevRadius, Vec2f& prevCenter)
{
  Vec2f s1 = center + Vec2f(0.0f, -size * 0.6f);
  Vec2f s2 = center + Vec2f(0.0f, size * 0.5f);
  Vec2f s3 = center + Vec2f(0.0f, size * 0.2f);

  MetaBall(currentPos, s1, size * 0.4f, extraRadius, count, totalDistance, totalNormal, prevRadius, prevCenter);
  
  Vec2f closestPoint = ClosestPointOnSegment(s2, s3, currentPos);
  MetaBall(currentPos, closestPoint, size * 0.5f, extraRadius, count, totalDistance, totalNormal, prevRadius, prevCenter);
}

static void Draw9(const Vec2f& currentPos, const Vec2f& center, const float& size, const float& extraRadius,
int& count, float& totalDistance, Vec2f& totalNormal, float& prevRadius, Vec2f& prevCenter)
{
  float radiusPercentage = 0.2f;
  float halfOffset = (0.5f - radiusPercentage);
  float offset = (1.0f - radiusPercentage);
  float radius = size * radiusPercentage;
  Vec2f s1 = center + Vec2f(-size * halfOffset, size * offset);
  Vec2f s2 = center + Vec2f(size * halfOffset, -size * 0.35f);
  Vec2f s3 = center + Vec2f(0.0f, -size * 0.5f);

  Vec2f closestPoint = ClosestPointOnSegment(s1, s2, currentPos);
  MetaBall(currentPos, closestPoint, radius, extraRadius, count, totalDistance, totalNormal, prevRadius, prevCenter);
  
  MetaBall(currentPos, s3, size * 0.5f, extraRadius, count, totalDistance, totalNormal, prevRadius, prevCenter);
}

static void DrawDigit(const Vec2f& currentPos, const Vec2f& center, const int& digit, const float& size, const float& extraRadius,
int& count, float& totalDistance, Vec2f& totalNormal, float& prevRadius, Vec2f& prevCenter)
{
  switch (digit)
  {
  case 0:
    Draw0(currentPos, center, size, extraRadius, count, totalDistance, totalNormal, prevRadius, prevCenter);
    break;
    
  case 1:
    Draw1(currentPos, center, size, extraRadius, count, totalDistance, totalNormal, prevRadius, prevCenter);
    break;
    
  case 2:
    Draw2(currentPos, center, size, extraRadius, count, totalDistance, totalNormal, prevRadius, prevCenter);
    break;
    
  case 3:
    Draw3(currentPos, center, size, extraRadius, count, totalDistance, totalNormal, prevRadius, prevCenter);
    break;
    
  case 4:
    Draw4(currentPos, center, size, extraRadius, count, totalDistance, totalNormal, prevRadius, prevCenter);
    break;
    
  case 5:
    Draw5(currentPos, center, size, extraRadius, count, totalDistance, totalNormal, prevRadius, prevCenter);
    break;
    
  case 6:
    Draw6(currentPos, center, size, extraRadius, count, totalDistance, totalNormal, prevRadius, prevCenter);
    break;
    
  case 7:
    Draw7(currentPos, center, size, extraRadius, count, totalDistance, totalNormal, prevRadius, prevCenter);
    break;
    
  case 8:
    Draw8(currentPos, center, size, extraRadius, count, totalDistance, totalNormal, prevRadius, prevCenter);
    break;
    
  case 9:
    Draw9(currentPos, center, size, extraRadius, count, totalDistance, totalNormal, prevRadius, prevCenter);
    break;
  
  default:
    break;
  }
}

static void DrawNumber(const Vec2f& currentPos, const Vec2f& center, const int& number, const float& size, const float& extraRadius,
int& count, float& totalDistance, Vec2f& totalNormal, float& prevRadius, Vec2f& prevCenter)
{
  int firstDigit = number / 10;
  int secondDigit = number % 10;

  if (currentPos.x <= center.x + extraRadius)
    DrawDigit(currentPos, center + Vec2f(-size * 0.5f, 0.0f), firstDigit, size, extraRadius, count, totalDistance, totalNormal, prevRadius, prevCenter);

  if (currentPos.x >= center.x - extraRadius)
    DrawDigit(currentPos, center + Vec2f(size * 0.5f, 0.0f), secondDigit, size, extraRadius, count, totalDistance, totalNormal, prevRadius, prevCenter);
}

const float EXTRA_RADIUS = 8.0f;
const float NUMBER_SIZE = 44.0f;
const float TOP_LINE = 45.0f;
const float BOTTOM_LINE = 155.0f;

void MetaBallWatchy::drawWatchFace()
{
  display.fillScreen(GxEPD_WHITE);
  display.setTextColor(GxEPD_BLACK);

  float battery = getBatteryFill();

  int hour = currentTime.Hour;
  
  int minute = currentTime.Minute;

  int month = currentTime.Month;

  int day = currentTime.Day;

  for (int y = 0; y < 200; ++y)
  {
    for (int x = 0; x < 200; ++x)
    {
      Vec2f currentPos ((float)x, (float)y);

      float totalDistance = 0.0f;
      Vec2f totalNormal (0.0f,0.0f);
      int count = 0;

      float prevRadius = 0.0f;
      Vec2f center = currentPos;

      if (y <= TOP_LINE + NUMBER_SIZE + EXTRA_RADIUS)
      {
        if (x < NUMBER_SIZE * 2 + EXTRA_RADIUS)
        {
          DrawNumber(currentPos, Vec2f(NUMBER_SIZE, TOP_LINE), hour, NUMBER_SIZE, EXTRA_RADIUS, count, totalDistance, totalNormal, prevRadius, center);
        }

        if (x > 200 - NUMBER_SIZE * 2 - EXTRA_RADIUS)
        {
          DrawNumber(currentPos, Vec2f(200.0f - NUMBER_SIZE, TOP_LINE), minute, NUMBER_SIZE, EXTRA_RADIUS, count, totalDistance, totalNormal, prevRadius, center);
        }

        const float radius = 6.0f;
        
        if (x >= 100 - radius - EXTRA_RADIUS && x <= 100 + radius + EXTRA_RADIUS)
        {
          MetaBall(currentPos, Vec2f(100.0f, TOP_LINE - 9.0f), radius, EXTRA_RADIUS, count, totalDistance, totalNormal, prevRadius, center);
          MetaBall(currentPos, Vec2f(100.0f, TOP_LINE + 9.0f), radius, EXTRA_RADIUS, count, totalDistance, totalNormal, prevRadius, center);
        }
      }

      if (y >= BOTTOM_LINE - NUMBER_SIZE - EXTRA_RADIUS)
      {
        if (x < NUMBER_SIZE * 2 + EXTRA_RADIUS)
        {
          DrawNumber(currentPos, Vec2f(NUMBER_SIZE, BOTTOM_LINE), month, NUMBER_SIZE, EXTRA_RADIUS, count, totalDistance, totalNormal, prevRadius, center);
        }

        if (x > 200 - NUMBER_SIZE * 2 - EXTRA_RADIUS)
        {
          DrawNumber(currentPos, Vec2f(200.0f - NUMBER_SIZE, BOTTOM_LINE), day, NUMBER_SIZE, EXTRA_RADIUS, count, totalDistance, totalNormal, prevRadius, center);
        }

        const float radius = 6.0f;

        if (x > 95.0f - radius - EXTRA_RADIUS && x < 105.0f + radius + EXTRA_RADIUS)
        {
          Vec2f s1 (105.0f, BOTTOM_LINE - 40.0f);
          Vec2f s2 (95.0f, BOTTOM_LINE + 40.0f);

          Vec2f closestPoint = ClosestPointOnSegment(s1, s2, currentPos);
          MetaBall(currentPos, closestPoint, radius, EXTRA_RADIUS, count, totalDistance, totalNormal, prevRadius, center);
        }
      }

      const float radius = 5.0f;

      if (y >= 100 - radius - EXTRA_RADIUS && y <= 100 + radius + EXTRA_RADIUS)
      {
        Vec2f s3 (100.0f - 90.0f * battery, 100.0f);
        Vec2f s4 (100.0f + 90.0f * battery, 100.0f);

        Vec2f closestPoint = ClosestPointOnSegment(s3, s4, currentPos);
        MetaBall(currentPos, closestPoint, radius, EXTRA_RADIUS, count, totalDistance, totalNormal, prevRadius, center);
      }

      if (count > 0)
      { 
        if (totalDistance > 1.0f)
        { 
          totalNormal *= 100.0f;
          totalNormal += Vec2f(100.0f, 100.0f);
          display.drawPixel(x, y, getColor3(x, y, totalNormal.x, totalNormal.y, MatCapSource, 200,200));
        }
        else if (totalDistance >= 1.0f - 1.0f / EXTRA_RADIUS)
          display.drawPixel(x,y,GxEPD_BLACK);
      }
    }
  }
}
