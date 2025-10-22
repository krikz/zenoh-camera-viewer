/**
 * Рендерер для отображения камеры (Image)
 */

import type { Image } from '@types';
import { CANVAS_CONFIG } from '@config';

export class CameraRenderer {
  private ctx: CanvasRenderingContext2D;

  constructor(private canvas: HTMLCanvasElement) {
    const ctx = canvas.getContext('2d', { willReadFrequently: true });
    if (!ctx) throw new Error('Failed to get 2D context');
    this.ctx = ctx;
  }

  /**
   * Отрисовывает изображение с камеры
   */
  render(image: Image): void {
    const { width, height, encoding, data, step } = image;

    if (!width || !height || !data || data.length === 0) return;

    // Устанавливаем размер canvas
    this.canvas.width = width;
    this.canvas.height = height;

    const imageData = this.ctx.createImageData(width, height);

    // Конвертируем данные в RGBA
    if (encoding === 'bgr8') {
      this.convertBGR8(data, imageData, width, height, step);
    } else if (encoding === 'rgb8') {
      this.convertRGB8(data, imageData, width, height, step);
    } else if (encoding === 'mono8') {
      this.convertMono8(data, imageData, width, height, step);
    } else {
      console.warn('[CameraRenderer] Неподдерживаемая кодировка:', encoding);
      return;
    }

    this.ctx.putImageData(imageData, 0, 0);
  }

  /**
   * Конвертирует BGR8 в RGBA
   */
  private convertBGR8(
    data: number[],
    imageData: ImageData,
    width: number,
    height: number,
    step: number
  ): void {
    for (let y = 0; y < height; y++) {
      for (let x = 0; x < width; x++) {
        const srcIdx = y * step + x * 3;
        const dstIdx = (y * width + x) * 4;

        imageData.data[dstIdx + 0] = data[srcIdx + 2]; // R
        imageData.data[dstIdx + 1] = data[srcIdx + 1]; // G
        imageData.data[dstIdx + 2] = data[srcIdx + 0]; // B
        imageData.data[dstIdx + 3] = 255; // A
      }
    }
  }

  /**
   * Конвертирует RGB8 в RGBA
   */
  private convertRGB8(
    data: number[],
    imageData: ImageData,
    width: number,
    height: number,
    step: number
  ): void {
    for (let y = 0; y < height; y++) {
      for (let x = 0; x < width; x++) {
        const srcIdx = y * step + x * 3;
        const dstIdx = (y * width + x) * 4;

        imageData.data[dstIdx + 0] = data[srcIdx + 0]; // R
        imageData.data[dstIdx + 1] = data[srcIdx + 1]; // G
        imageData.data[dstIdx + 2] = data[srcIdx + 2]; // B
        imageData.data[dstIdx + 3] = 255; // A
      }
    }
  }

  /**
   * Конвертирует Mono8 в RGBA
   */
  private convertMono8(
    data: number[],
    imageData: ImageData,
    width: number,
    height: number,
    step: number
  ): void {
    for (let y = 0; y < height; y++) {
      for (let x = 0; x < width; x++) {
        const srcIdx = y * step + x;
        const dstIdx = (y * width + x) * 4;
        const v = data[srcIdx];

        imageData.data[dstIdx + 0] = v; // R
        imageData.data[dstIdx + 1] = v; // G
        imageData.data[dstIdx + 2] = v; // B
        imageData.data[dstIdx + 3] = 255; // A
      }
    }
  }

  /**
   * Очищает canvas
   */
  clear(): void {
    this.ctx.clearRect(0, 0, this.canvas.width, this.canvas.height);
  }
}
