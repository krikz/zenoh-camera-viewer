/**
 * Рендерер для отображения карты (OccupancyGrid)
 */

import type { OccupancyGrid, Waypoint, RobotPosition } from '@types';
import { CANVAS_CONFIG, RENDER_CONFIG } from '@config';
import { mapToCanvas } from '@utils';

export class MapRenderer {
  private ctx: CanvasRenderingContext2D;
  private currentMap: OccupancyGrid | null = null;

  constructor(private canvas: HTMLCanvasElement) {
    const ctx = canvas.getContext('2d');
    if (!ctx) throw new Error('Failed to get 2D context');
    this.ctx = ctx;
    
    // Установка размера canvas
    canvas.width = CANVAS_CONFIG.MAP.WIDTH;
    canvas.height = CANVAS_CONFIG.MAP.HEIGHT;
  }

  /**
   * Отрисовывает карту
   */
  renderMap(map: OccupancyGrid): void {
    this.currentMap = map;
    const { width, height } = map.info;
    const { data } = map;

    // Создаем ImageData
    const imageData = this.ctx.createImageData(this.canvas.width, this.canvas.height);

    const scaleX = this.canvas.width / width;
    const scaleY = this.canvas.height / height;

    // Заполняем пиксели
    for (let y = 0; y < height; y++) {
      for (let x = 0; x < width; x++) {
        const value = data[y * width + x];
        
        let color: [number, number, number];
        if (value === -1) {
          color = [128, 128, 128]; // unknown - gray
        } else if (value === 0) {
          color = [255, 255, 255]; // free - white
        } else {
          color = [0, 0, 0]; // occupied - black
        }

        // Масштабируем и рисуем пиксель
        const canvasX = Math.floor(x * scaleX);
        const canvasY = Math.floor((height - 1 - y) * scaleY); // Инвертируем Y

        const idx = (canvasY * this.canvas.width + canvasX) * 4;
        imageData.data[idx] = color[0];
        imageData.data[idx + 1] = color[1];
        imageData.data[idx + 2] = color[2];
        imageData.data[idx + 3] = 255; // Alpha
      }
    }

    this.ctx.putImageData(imageData, 0, 0);
  }

  /**
   * Отрисовывает позицию робота
   */
  renderRobotPosition(position: RobotPosition): void {
    if (!this.currentMap) return;

    const { x, y, theta } = position;
    const { info } = this.currentMap;

    const canvasPos = mapToCanvas(
      x,
      y,
      info.width,
      info.height,
      info.resolution,
      info.origin.position.x,
      info.origin.position.y,
      this.canvas.width,
      this.canvas.height
    );

    // Рисуем робота
    this.ctx.fillStyle = RENDER_CONFIG.LIDAR_COLORS.ROBOT;
    this.ctx.beginPath();
    this.ctx.arc(canvasPos.x, canvasPos.y, RENDER_CONFIG.SIZES.ROBOT_RADIUS, 0, 2 * Math.PI);
    this.ctx.fill();

    // Рисуем направление
    const arrowLength = 15;
    const endX = canvasPos.x + Math.cos(theta) * arrowLength;
    const endY = canvasPos.y - Math.sin(theta) * arrowLength;

    this.ctx.strokeStyle = RENDER_CONFIG.LIDAR_COLORS.ROBOT;
    this.ctx.lineWidth = 2;
    this.ctx.beginPath();
    this.ctx.moveTo(canvasPos.x, canvasPos.y);
    this.ctx.lineTo(endX, endY);
    this.ctx.stroke();
  }

  /**
   * Отрисовывает вейпоинты
   */
  renderWaypoints(waypoints: Waypoint[]): void {
    if (!this.currentMap) return;

    const { info } = this.currentMap;

    waypoints.forEach((wp, index) => {
      const canvasPos = mapToCanvas(
        wp.x,
        wp.y,
        info.width,
        info.height,
        info.resolution,
        info.origin.position.x,
        info.origin.position.y,
        this.canvas.width,
        this.canvas.height
      );

      // Рисуем вейпоинт
      this.ctx.fillStyle = RENDER_CONFIG.LIDAR_COLORS.WAYPOINT;
      this.ctx.beginPath();
      this.ctx.arc(canvasPos.x, canvasPos.y, RENDER_CONFIG.SIZES.WAYPOINT_RADIUS, 0, 2 * Math.PI);
      this.ctx.fill();

      // Рисуем номер
      this.ctx.fillStyle = 'white';
      this.ctx.font = '10px Arial';
      this.ctx.fillText(`${index + 1}`, canvasPos.x - 3, canvasPos.y + 3);
    });
  }

  /**
   * Отрисовывает траекторию
   */
  renderTrajectory(path: Array<{ x: number; y: number }>): void {
    if (!this.currentMap || path.length === 0) return;

    const { info } = this.currentMap;

    this.ctx.strokeStyle = RENDER_CONFIG.LIDAR_COLORS.TRAJECTORY;
    this.ctx.lineWidth = RENDER_CONFIG.SIZES.TRAJECTORY_WIDTH;
    this.ctx.beginPath();

    path.forEach((point, index) => {
      const canvasPos = mapToCanvas(
        point.x,
        point.y,
        info.width,
        info.height,
        info.resolution,
        info.origin.position.x,
        info.origin.position.y,
        this.canvas.width,
        this.canvas.height
      );

      if (index === 0) {
        this.ctx.moveTo(canvasPos.x, canvasPos.y);
      } else {
        this.ctx.lineTo(canvasPos.x, canvasPos.y);
      }
    });

    this.ctx.stroke();
  }

  /**
   * Очищает canvas
   */
  clear(): void {
    this.ctx.clearRect(0, 0, this.canvas.width, this.canvas.height);
  }

  /**
   * Получает текущую карту
   */
  getCurrentMap(): OccupancyGrid | null {
    return this.currentMap;
  }
}
