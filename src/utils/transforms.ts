/**
 * Утилиты для преобразования координат
 */

import type { Quaternion, Transform, RobotPosition } from '../types';

/**
 * Преобразует кватернион в угол Эйлера (yaw) для 2D
 */
export function quaternionToYaw(q: Quaternion): number {
  return Math.atan2(
    2 * (q.w * q.z + q.x * q.y),
    1 - 2 * (q.y * q.y + q.z * q.z)
  );
}

/**
 * Преобразует угол Эйлера (yaw) в кватернион
 */
export function yawToQuaternion(yaw: number): Quaternion {
  const halfYaw = yaw / 2;
  return {
    x: 0,
    y: 0,
    z: Math.sin(halfYaw),
    w: Math.cos(halfYaw),
  };
}

/**
 * Преобразует позицию из системы odom в систему map
 */
export function transformOdomToMap(
  odomX: number,
  odomY: number,
  odomTheta: number,
  mapToOdom: Transform
): RobotPosition {
  const { x: tx, y: ty } = mapToOdom.translation;
  const { x: qx, y: qy, z: qz, w: qw } = mapToOdom.rotation;

  // Вычисляем угол поворота трансформации odom -> map
  const odomToMapTheta = Math.atan2(
    2 * (qw * qz + qx * qy),
    1 - 2 * (qy * qy + qz * qz)
  );

  // Преобразуем точку из системы odom в систему map
  const cosTheta = Math.cos(odomToMapTheta);
  const sinTheta = Math.sin(odomToMapTheta);

  const mapX = tx + cosTheta * odomX - sinTheta * odomY;
  const mapY = ty + sinTheta * odomX + cosTheta * odomY;

  // Угол робота в системе map (складываем углы)
  const mapTheta = odomTheta + odomToMapTheta;

  return { x: mapX, y: mapY, theta: mapTheta };
}

/**
 * Преобразует координаты карты в координаты пикселей canvas
 */
export function mapToCanvas(
  mapX: number,
  mapY: number,
  mapWidth: number,
  mapHeight: number,
  resolution: number,
  originX: number,
  originY: number,
  canvasWidth: number,
  canvasHeight: number
): { x: number; y: number } {
  // Преобразуем координаты карты в пиксели
  const gridX = (mapX - originX) / resolution;
  const gridY = (mapY - originY) / resolution;

  // Масштабируем под размер canvas
  const scaleX = canvasWidth / mapWidth;
  const scaleY = canvasHeight / mapHeight;

  const canvasX = gridX * scaleX;
  const canvasY = canvasHeight - gridY * scaleY; // Инвертируем Y

  return { x: canvasX, y: canvasY };
}

/**
 * Преобразует координаты canvas в координаты карты
 */
export function canvasToMap(
  canvasX: number,
  canvasY: number,
  mapWidth: number,
  mapHeight: number,
  resolution: number,
  originX: number,
  originY: number,
  canvasWidth: number,
  canvasHeight: number
): { x: number; y: number } {
  // Масштабируем обратно
  const scaleX = canvasWidth / mapWidth;
  const scaleY = canvasHeight / mapHeight;

  const gridX = canvasX / scaleX;
  const gridY = (canvasHeight - canvasY) / scaleY; // Инвертируем Y обратно

  // Преобразуем в координаты карты
  const mapX = gridX * resolution + originX;
  const mapY = gridY * resolution + originY;

  return { x: mapX, y: mapY };
}

/**
 * Вычисляет расстояние между двумя точками
 */
export function distance(
  x1: number,
  y1: number,
  x2: number,
  y2: number
): number {
  const dx = x2 - x1;
  const dy = y2 - y1;
  return Math.sqrt(dx * dx + dy * dy);
}

/**
 * Нормализует угол в диапазон [-π, π]
 */
export function normalizeAngle(angle: number): number {
  while (angle > Math.PI) angle -= 2 * Math.PI;
  while (angle < -Math.PI) angle += 2 * Math.PI;
  return angle;
}
