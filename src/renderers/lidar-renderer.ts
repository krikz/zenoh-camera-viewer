/**
 * Рендерер для отображения лидара (LaserScan)
 */

import type { LaserScan, RobotPosition } from '@types';
import { CANVAS_CONFIG, RENDER_CONFIG } from '@config';

export class LidarRenderer {
  private ctx: CanvasRenderingContext2D;

  constructor(private canvas: HTMLCanvasElement) {
    const ctx = canvas.getContext('2d', { willReadFrequently: true });
    if (!ctx) throw new Error('Failed to get 2D context');
    this.ctx = ctx;

    // Установка размера canvas
    canvas.width = CANVAS_CONFIG.LIDAR.WIDTH;
    canvas.height = CANVAS_CONFIG.LIDAR.HEIGHT;
  }

  /**
   * Отрисовывает данные лидара
   */
  render(scan: LaserScan, robotPosition: RobotPosition): void {
    this.clear();

    const centerX = this.canvas.width / 2;
    const centerY = this.canvas.height / 2;
    const scale = 50; // пикселей на метр

    const { angle_min, angle_increment, ranges } = scan;
    const { theta } = robotPosition;

    // Рисуем точки лидара
    this.ctx.fillStyle = RENDER_CONFIG.LIDAR_COLORS.POINT;
    
    ranges.forEach((range, i) => {
      if (range < scan.range_min || range > scan.range_max) return;
      if (!isFinite(range)) return;

      const angle = angle_min + i * angle_increment + theta;
      const x = centerX + range * Math.cos(angle) * scale;
      const y = centerY - range * Math.sin(angle) * scale;

      this.ctx.beginPath();
      this.ctx.arc(x, y, RENDER_CONFIG.SIZES.LIDAR_POINT_SIZE, 0, 2 * Math.PI);
      this.ctx.fill();
    });

    // Рисуем робота в центре
    this.ctx.fillStyle = RENDER_CONFIG.LIDAR_COLORS.ROBOT;
    this.ctx.beginPath();
    this.ctx.arc(centerX, centerY, RENDER_CONFIG.SIZES.ROBOT_RADIUS, 0, 2 * Math.PI);
    this.ctx.fill();

    // Рисуем направление робота
    const arrowLength = 20;
    const endX = centerX + Math.cos(theta) * arrowLength;
    const endY = centerY - Math.sin(theta) * arrowLength;

    this.ctx.strokeStyle = RENDER_CONFIG.LIDAR_COLORS.ROBOT;
    this.ctx.lineWidth = 2;
    this.ctx.beginPath();
    this.ctx.moveTo(centerX, centerY);
    this.ctx.lineTo(endX, endY);
    this.ctx.stroke();
  }

  /**
   * Очищает canvas
   */
  clear(): void {
    this.ctx.clearRect(0, 0, this.canvas.width, this.canvas.height);
  }
}
