#include "MetaBallWatchy.h"
#include <VectorXf.h>

const float VOLTAGE_MIN = 3.5;
const float VOLTAGE_MAX = 4.2;
const float VOLTAGE_WARNING = 3.6;
const float VOLTAGE_RANGE = VOLTAGE_MAX - VOLTAGE_MIN;

MetaBallWatchy::MetaBallWatchy(const watchySettings& s) : Watchy(s)
{
  Serial.begin(115200);
while (!Serial); // wait for serial port to connect. Needed for native USB port on Arduino only

}

static float DistanceToCircle(Vec2f position, Vec2f center)
{
  return (position - center).length();
}

double MetaBallWatchy::getBatteryFill()
{
  float VBAT = getBatteryVoltage();

  // 12 battery states
  double batState = ((VBAT - VOLTAGE_MIN) / VOLTAGE_RANGE);

  if (batState > 1.0)
    batState = 1.0;

  if (batState < 0)
    batState = 0;

  return batState;
}

static bool getColor(int16_t x, int16_t y, uint16_t color) 
{
  return color > BlueNoise200[y * 200 + x];
}

static bool getColor2(int16_t x, int16_t y, int16_t xUv, int16_t yUv, const uint8_t *bitmap, int16_t w, int16_t h) 
{
  return getColor(x, y, bitmap[yUv * w + xUv]);
}

static bool getColor3(int16_t x, int16_t y, int16_t xUv, int16_t yUv, const uint8_t *bitmap, int16_t w, int16_t h) 
{
  return getColor(x,y,bitmap[yUv * w + xUv]);
}

static float clamp(float val, float min, float max)
{
  if (val > max)
    val = max;
  
  if (val < min)
    val = min;

  return val;
}

static float unlerp(float value, float min, float max) {
  // Evaluate polynomial
  return (value - min) / (max - min);
}

static float smoothstep(float x) {
  // Evaluate polynomial
  return x * x * (3 - 2 * x);
}

static Vec2f ClosestPointOnSegment(Vec2f s1, Vec2f s2, Vec2f p, float& unlerp)
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

// ToDo return minimal distance for calculating normals when blending
static void Circle(Vec2f currentPos, Vec2f circleCenter, float radius, float extraRadius, int& count, float& totalDistance, Vec2f& totalNormal, float& prevRadius, Vec2f& prevCenter)
{
  Vec2f offset = currentPos - circleCenter;
  float totalRadus = radius + extraRadius;

  if (offset.lengthSquared() >= totalRadus * totalRadus)
    return;
      
  float distance = offset.length();
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
  float innerRadius = arcRadius - radius - extraRadius;

  if (innerRadius < 0.0f)
    innerRadius = 0.0f;

  float lengthSquared = offset.lengthSquared();

  if (lengthSquared > outterRadius * outterRadius || lengthSquared < innerRadius * innerRadius)
    return;

  Vec2f closestToArc = ClosestPointOnArc(center, arcRadius, arcStartAngle, arcAngle, currentPos);
  Circle(currentPos, closestToArc, radius, extraRadius, count, totalDistance, totalNormal, prevRadius, prevCenter);
}

static void Draw0(Vec2f currentPos, Vec2f center, float size, float extraRadius, int& count, float& totalDistance, Vec2f& totalNormal, float& prevRadius, Vec2f& prevCenter)
{
  float weight = 0.0f;

  float radius = size * 0.5f;
  Vec2f s1 = center + Vec2f(0.0f, radius);
  Vec2f s2 = center + Vec2f(0.0f, -radius);

  Vec2f closestPoint = ClosestPointOnSegment(s1, s2, currentPos, weight);
  Circle(currentPos, closestPoint, radius, extraRadius, count, totalDistance, totalNormal, prevRadius, prevCenter);
}

static void Draw1(Vec2f currentPos, Vec2f center, float size, float extraRadius, int& count, float& totalDistance, Vec2f& totalNormal, float& prevRadius, Vec2f& prevCenter)
{
  float weight = 0.0f;

  float radiusPercentage = 0.2f;
  float halfOffset = (0.5f - radiusPercentage);
  float offset = (1.0f - radiusPercentage);
  float radius = size * radiusPercentage;
  Vec2f s1 = center + Vec2f(size * halfOffset, -size * offset);
  Vec2f s2 = center + Vec2f(size * halfOffset, size * offset);
  Vec2f s3 = center + Vec2f(-size * halfOffset, -size * 0.2f);

  Vec2f closestPoint = ClosestPointOnSegment(s1, s2, currentPos, weight);
  Circle(currentPos, closestPoint, radius, extraRadius, count, totalDistance, totalNormal, prevRadius, prevCenter);
  
  closestPoint = ClosestPointOnSegment(s1, s3, currentPos, weight);
  Circle(currentPos, closestPoint, radius, extraRadius, count, totalDistance, totalNormal, prevRadius, prevCenter);
}

static void Draw2(Vec2f currentPos, Vec2f center, float size, float extraRadius, int& count, float& totalDistance, Vec2f& totalNormal, float& prevRadius, Vec2f& prevCenter)
{
  float weight = 0.0f;

  float radiusPercentage = 0.2f;
  float halfOffset = (0.5f - radiusPercentage);
  float offset = (1.0f - radiusPercentage);
  float radius = size * radiusPercentage;
  Vec2f s1 = center + Vec2f(-size * halfOffset, -size * offset);
  Vec2f s2 = center + Vec2f(size * 0.2f, -size * 0.2f);
  Vec2f s3 = center + Vec2f(-size * halfOffset, size * offset);
  Vec2f s4 = center + Vec2f(size * halfOffset, size * offset);

  //Vec2f closestToArc = ClosestPointOnArc(center + Vec2f(0.0f, -size * 0.5f), size * halfOffset, 180.0f, 270.0f, currentPos);
  //Circle(currentPos, closestToArc, radius, extraRadius, count, totalDistance, totalNormal, prevRadius, center);

  Arc(currentPos, center + Vec2f(0.0f, -size * 0.5f), radius, extraRadius, size * halfOffset, 180.0f, 200.0f, count, totalDistance, totalNormal, prevRadius, prevCenter);

  Vec2f closestPoint = ClosestPointOnSegment(s2, s3, currentPos, weight);
  Circle(currentPos, closestPoint, radius, extraRadius, count, totalDistance, totalNormal, prevRadius, prevCenter);
  
  closestPoint = ClosestPointOnSegment(s3, s4, currentPos, weight);
  Circle(currentPos, closestPoint, radius, extraRadius, count, totalDistance, totalNormal, prevRadius, prevCenter);
}

static void Draw3(Vec2f currentPos, Vec2f center, float size, float extraRadius, int& count, float& totalDistance, Vec2f& totalNormal, float& prevRadius, Vec2f& prevCenter)
{
  float weight = 0.0f;

  float radiusPercentage = 0.2f;
  float halfOffset = (0.5f - radiusPercentage);
  float offset = (1.0f - radiusPercentage);
  float radius = size * radiusPercentage;
  Vec2f s1 = center + Vec2f(-size * halfOffset, -size * offset);
  Vec2f s2 = center + Vec2f(size * halfOffset, -size * offset);
  Vec2f s3 = center + Vec2f(size * halfOffset, size * offset);
  Vec2f s4 = center + Vec2f(-size * halfOffset, size * offset);

  
  Arc(currentPos, center + Vec2f(0.0f, -size * 0.5f), radius, extraRadius, size * halfOffset, 180.0f, 270.0f, count, totalDistance, totalNormal, prevRadius, prevCenter);
  
  Arc(currentPos, center + Vec2f(-size * 0.1f, size * 0.4f), radius, extraRadius, size * (halfOffset + 0.1f), 280.0f, 195.0f, count, totalDistance, totalNormal, prevRadius, prevCenter);

  //Vec2f closestToArc = ClosestPointOnArc(center + Vec2f(0.0f, -size * 0.5f), size * halfOffset, 180.0f, 270.0f, currentPos);
  //Circle(currentPos, closestToArc, radius, extraRadius, count, totalDistance, totalNormal, prevRadius, center);

  //closestToArc = ClosestPointOnArc(center + Vec2f(-size * 0.1f, size * 0.1f), size * (halfOffset + 0.1f), 300.0f, 180.0f, currentPos);
  //Circle(currentPos, closestToArc, radius, extraRadius, count, totalDistance, totalNormal, prevRadius, center);

return;
  Vec2f closestPoint = ClosestPointOnSegment(s1, s2, currentPos, weight);
  Circle(currentPos, closestPoint, radius, extraRadius, count, totalDistance, totalNormal, prevRadius, prevCenter);
  
  closestPoint = ClosestPointOnSegment(s2, center, currentPos, weight);
  Circle(currentPos, closestPoint, radius, extraRadius, count, totalDistance, totalNormal, prevRadius, prevCenter);

  closestPoint = ClosestPointOnSegment(center, s3, currentPos, weight);
  Circle(currentPos, closestPoint, radius, extraRadius, count, totalDistance, totalNormal, prevRadius, prevCenter);
  
  closestPoint = ClosestPointOnSegment(s3, s4, currentPos, weight);
  Circle(currentPos, closestPoint, radius, extraRadius, count, totalDistance, totalNormal, prevRadius, prevCenter);
}

static void Draw4(Vec2f currentPos, Vec2f center, float size, float extraRadius, int& count, float& totalDistance, Vec2f& totalNormal, float& prevRadius, Vec2f& prevCenter)
{
  float weight = 0.0f;

  float radiusPercentage = 0.2f;
  float halfOffset = (0.5f - radiusPercentage);
  float offset = (1.0f - radiusPercentage);
  float radius = size * radiusPercentage;
  Vec2f s1 = center + Vec2f(0.0f, -size * offset);
  Vec2f s2 = center + Vec2f(-size * halfOffset, size * 0.2f);
  Vec2f s3 = center + Vec2f(size * halfOffset, size * 0.2f);
  Vec2f s4 = center + Vec2f(size * 0.3f, -size * 0.1f);
  Vec2f s5 = center + Vec2f(size * 0.1f, size * offset);

  Vec2f closestPoint = ClosestPointOnSegment(s1, s2, currentPos, weight);
  Circle(currentPos, closestPoint, radius, extraRadius, count, totalDistance, totalNormal, prevRadius, prevCenter);

  closestPoint = ClosestPointOnSegment(s2, s3, currentPos, weight);
  Circle(currentPos, closestPoint, radius, extraRadius, count, totalDistance, totalNormal, prevRadius, prevCenter);

  closestPoint = ClosestPointOnSegment(s4, s5, currentPos, weight);
  Circle(currentPos, closestPoint, radius, extraRadius, count, totalDistance, totalNormal, prevRadius, prevCenter);
}

static void Draw5(Vec2f currentPos, Vec2f center, float size, float extraRadius, int& count, float& totalDistance, Vec2f& totalNormal, float& prevRadius, Vec2f& prevCenter)
{
  float weight = 0.0f;

  float radiusPercentage = 0.2f;
  float halfOffset = (0.5f - radiusPercentage);
  float offset = (1.0f - radiusPercentage);
  float radius = size * radiusPercentage;
  Vec2f s1 = center + Vec2f(size * halfOffset, -size * offset);
  Vec2f s2 = center + Vec2f(-size * halfOffset, -size * offset);
  Vec2f s3 = center + Vec2f(-size * halfOffset, 0.0f);
  Vec2f s4 = center + Vec2f(size * halfOffset, 0.0f);
  Vec2f s5 = center + Vec2f(size * halfOffset, size * offset);
  Vec2f s6 = center + Vec2f(-size * halfOffset, size * offset);

  Vec2f closestPoint = ClosestPointOnSegment(s1, s2, currentPos, weight);
  Circle(currentPos, closestPoint, radius, extraRadius, count, totalDistance, totalNormal, prevRadius, prevCenter);
  
  closestPoint = ClosestPointOnSegment(s2, s3, currentPos, weight);
  Circle(currentPos, closestPoint, radius, extraRadius, count, totalDistance, totalNormal, prevRadius, prevCenter);

  Arc(currentPos, center + Vec2f(-size * 0.1f, size * 0.4f), radius, extraRadius, size * (halfOffset + 0.1f), 235.0f, 250.0f, count, totalDistance, totalNormal, prevRadius, prevCenter);
  
  //closestPoint = ClosestPointOnSegment(s3, s4, currentPos, weight);
  //Circle(currentPos, closestPoint, radius, extraRadius, count, totalDistance, totalNormal, prevRadius, center);

  //closestPoint = ClosestPointOnSegment(s4, s5, currentPos, weight);
  //Circle(currentPos, closestPoint, radius, extraRadius, count, totalDistance, totalNormal, prevRadius, center);

  //closestPoint = ClosestPointOnSegment(s5, s6, currentPos, weight);
  //Circle(currentPos, closestPoint, radius, extraRadius, count, totalDistance, totalNormal, prevRadius, center);
}

static void Draw6(Vec2f currentPos, Vec2f center, float size, float extraRadius, int& count, float& totalDistance, Vec2f& totalNormal, float& prevRadius, Vec2f& prevCenter)
{
  float weight = 0.0f;

  float radiusPercentage = 0.2f;
  float halfOffset = (0.5f - radiusPercentage);
  float offset = (1.0f - radiusPercentage);
  float radius = size * radiusPercentage;
  Vec2f s1 = center + Vec2f(size * halfOffset, -size * offset);
  Vec2f s2 = center + Vec2f(-size * halfOffset, size * 0.35f);
  Vec2f s3 = center + Vec2f(0.0f, size * 0.5f);

  Vec2f closestPoint = ClosestPointOnSegment(s1, s2, currentPos, weight);
  Circle(currentPos, closestPoint, radius, extraRadius, count, totalDistance, totalNormal, prevRadius, prevCenter);
  
  Circle(currentPos, s3, size * 0.5f, extraRadius, count, totalDistance, totalNormal, prevRadius, prevCenter);
}

static void Draw7(Vec2f currentPos, Vec2f center, float size, float extraRadius, int& count, float& totalDistance, Vec2f& totalNormal, float& prevRadius, Vec2f& prevCenter)
{
  float weight = 0.0f;

  float radiusPercentage = 0.2f;
  float halfOffset = (0.5f - radiusPercentage);
  float offset = (1.0f - radiusPercentage);
  float radius = size * radiusPercentage;
  Vec2f s1 = center + Vec2f(size * halfOffset, -size * offset);
  Vec2f s2 = center + Vec2f(-size * halfOffset, size * offset);
  Vec2f s3 = center + Vec2f(-size * halfOffset, -size * offset);

  Vec2f closestPoint = ClosestPointOnSegment(s1, s2, currentPos, weight);
  Circle(currentPos, closestPoint, radius, extraRadius, count, totalDistance, totalNormal, prevRadius, prevCenter);
  
  closestPoint = ClosestPointOnSegment(s1, s3, currentPos, weight);
  Circle(currentPos, closestPoint, radius, extraRadius, count, totalDistance, totalNormal, prevRadius, prevCenter);
}

static void Draw8(Vec2f currentPos, Vec2f center, float size, float extraRadius, int& count, float& totalDistance, Vec2f& totalNormal, float& prevRadius, Vec2f& prevCenter)
{
  Vec2f s1 = center + Vec2f(0.0f, -size * 0.6f);
  Vec2f s2 = center + Vec2f(0.0f, size * 0.5f);
  Vec2f s3 = center + Vec2f(0.0f, size * 0.2f);

  Circle(currentPos, s1, size * 0.4f, extraRadius, count, totalDistance, totalNormal, prevRadius, prevCenter);
  
  float weight = 0.0f;

  Vec2f closestPoint = ClosestPointOnSegment(s2, s3, currentPos, weight);
  Circle(currentPos, closestPoint, size * 0.5f, extraRadius, count, totalDistance, totalNormal, prevRadius, prevCenter);
}

static void Draw9(Vec2f currentPos, Vec2f center, float size, float extraRadius, int& count, float& totalDistance, Vec2f& totalNormal, float& prevRadius, Vec2f& prevCenter)
{
  float weight = 0.0f;

  float radiusPercentage = 0.2f;
  float halfOffset = (0.5f - radiusPercentage);
  float offset = (1.0f - radiusPercentage);
  float radius = size * radiusPercentage;
  Vec2f s1 = center + Vec2f(-size * halfOffset, size * offset);
  Vec2f s2 = center + Vec2f(size * halfOffset, -size * 0.35f);
  Vec2f s3 = center + Vec2f(0.0f, -size * 0.5f);

  Vec2f closestPoint = ClosestPointOnSegment(s1, s2, currentPos, weight);
  Circle(currentPos, closestPoint, radius, extraRadius, count, totalDistance, totalNormal, prevRadius, prevCenter);
  
  Circle(currentPos, s3, size * 0.5f, extraRadius, count, totalDistance, totalNormal, prevRadius, prevCenter);
}

static void DrawDigit(Vec2f currentPos, Vec2f center, int digit, float size, float extraRadius, int& count, float& totalDistance, Vec2f& totalNormal, 
float& prevRadius, Vec2f& prevCenter)
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

void MetaBallWatchy::drawWatchFace()
{
  display.fillScreen(GxEPD_WHITE);
  display.setTextColor(GxEPD_BLACK);

  /*const float multiplier = 0.2f;
  const float scale = multiplier * 0.01f;
  const float offset = 99.5f;*/

  float battery = getBatteryFill();

  int hour = currentTime.Hour;
  int hourFirstDigit = hour / 10;
  int hourSecondDigit = hour % 10;
  
  int minute = currentTime.Minute;
  int minuteFirstDigit = minute / 10;
  int minuteSecondDigit = minute % 10;

  int month = currentTime.Month;
  int monthFirstDigit = month / 10;
  int monthSecondDigit = month % 10;

  int day = currentTime.Day;
  int dayFirstDigit = day / 10;
  int daySecondDigit = day % 10;

  for (int y = 0; y < 200; ++y)
  {
    for (int x = 0; x < 200; ++x)
    {
      Vec2f currentPos ((float)x, (float)y);

      float totalDistance = 0.0f;
      Vec2f totalNormal (0.0f,0.0f);
      int count = 0;
      float extraRadius = 8.0f;

      float prevRadius = 0.0f;
      Vec2f center = currentPos;

      float size = 45.0f;
      float topLine = 46.0f;

      Vec2f digitCenter (22.5f, topLine);
      DrawDigit(currentPos, digitCenter, hourFirstDigit, size, extraRadius, count, totalDistance, totalNormal, prevRadius, center);

      digitCenter.set(Vec2f(70.0f, topLine));
      DrawDigit(currentPos, digitCenter, hourSecondDigit, size, extraRadius, count, totalDistance, totalNormal, prevRadius, center);

      digitCenter.set(Vec2f(130.0f, topLine));
      DrawDigit(currentPos, digitCenter, minuteFirstDigit, size, extraRadius, count, totalDistance, totalNormal, prevRadius, center);

      digitCenter.set(Vec2f(177.5f, topLine));
      DrawDigit(currentPos, digitCenter, minuteSecondDigit, size, extraRadius, count, totalDistance, totalNormal, prevRadius, center);

      float radius = 10.0f;

      Vec2f circleCenter (100.0f, topLine - 20.0f);

      Circle(currentPos, circleCenter, radius, extraRadius, count, totalDistance, totalNormal, prevRadius, center);

      circleCenter.set(100.0f, topLine + 20.0f);
      Circle(currentPos, circleCenter, radius, extraRadius, count, totalDistance, totalNormal, prevRadius, center);

      
      float bottomLine = 154.0f;

      digitCenter.set(Vec2f(22.5f, bottomLine));
      DrawDigit(currentPos, digitCenter, monthFirstDigit, size, extraRadius, count, totalDistance, totalNormal, prevRadius, center);

      digitCenter.set(Vec2f(70.0f, bottomLine));
      DrawDigit(currentPos, digitCenter, monthSecondDigit, size, extraRadius, count, totalDistance, totalNormal, prevRadius, center);

      digitCenter.set(Vec2f(130.0f, bottomLine));
      DrawDigit(currentPos, digitCenter, dayFirstDigit, size, extraRadius, count, totalDistance, totalNormal, prevRadius, center);

      digitCenter.set(Vec2f(177.5f, bottomLine));
      DrawDigit(currentPos, digitCenter, daySecondDigit, size, extraRadius, count, totalDistance, totalNormal, prevRadius, center);

      Vec2f s1 (105.0f, bottomLine - 40.0f);
      Vec2f s2 (95.0f, bottomLine + 40.0f);

      radius = 8.0f;

      float weight = 0.0f;
      Vec2f closestPoint = ClosestPointOnSegment(s1, s2, currentPos, weight);
      Circle(currentPos, closestPoint, radius, extraRadius, count, totalDistance, totalNormal, prevRadius, center);


      radius = 5.0f;

      Vec2f s3 (100.0f - 90.0f * battery, 100.0f);
      Vec2f s4 (100.0f + 90.0f * battery, 100.0f);

      closestPoint = ClosestPointOnSegment(s3, s4, currentPos, weight);
      Circle(currentPos, closestPoint, radius, extraRadius, count, totalDistance, totalNormal, prevRadius, center);


      //Circle(currentPos, circleCenter, radius, extraRadius, count, totalDistance, totalNormal, prevRadius, center);

      /* if (offset.lengthSquared() < (radius + extraRadius) * (radius + extraRadius))
      {
        float distance = offset.length();
        float newDistance = radius + extraRadius - distance;

        count++;

        // ToDo: fix
        totalDistance += newDistance;
        float weight = newDistance / (totalDistance / count);

        Vec2f normal = offset / radius;
        totalNormal = totalNormal * (1.0f - weight) + weight * normal;

      } */

      //circleCenter.set(130.0f, 125.0f);
      //radius = 35.0f;
      
      //Circle(currentPos, circleCenter, radius, extraRadius, count, totalDistance, totalNormal, prevRadius, center);

      /* if (offset.lengthSquared() < (radius + extraRadius) * (radius + extraRadius))
      {
        float distance = offset.length();
        float newDistance = radius + extraRadius - distance;

        count++;

        totalDistance += newDistance;
        float weight = newDistance / (totalDistance / count);

        Vec2f normal = offset / radius;
        totalNormal = totalNormal * (1.0f - weight) + weight * normal;
      } */

      //circleCenter.set(160.0f, 60.0f);
      //radius = 35.0f;
      
      //Circle(currentPos, circleCenter, radius, extraRadius, count, totalDistance, totalNormal, prevRadius, center);


      if (count > 0)
      { 
        if (totalDistance > 1.0f)
        { 
          totalNormal *= 100.0f;
          totalNormal += (100.0f, 100.0f);
          display.drawPixel(x, y, getColor3(x, y, totalNormal.x, totalNormal.y, MatCapSource, 200,200));
          //display.drawPixel(x, y, GxEPD_BLACK);
        }
        else if (totalDistance >= 0.98f)
          display.drawPixel(x,y,GxEPD_BLACK);
      }

      /*Vec2f circleCenter2 (125.0f, 130.0f);
      distance = DistanceToCircle(currentPos, circleCenter2);
      newDistance = 35.0f - distance;

      if (newDistance < 0.0f)
        newDistance = 0.0f;

      totalDistance += newDistance;*/

    }
  }
}
