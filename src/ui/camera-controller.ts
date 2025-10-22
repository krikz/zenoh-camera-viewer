/**
 * Управление окном камеры: перетаскивание и изменение размера
 */

import { STORAGE_KEYS } from '../config';

interface CameraPosition {
  x: number;
  y: number;
  width: number;
  height: number;
}

const DEFAULT_POSITION: CameraPosition = {
  x: 20,
  y: 20,
  width: 400,
  height: 300,
};

export class CameraController {
  private isDragging = false;
  private isResizing = false;
  private startX = 0;
  private startY = 0;
  private startLeft = 0;
  private startTop = 0;
  private startWidth = 0;
  private startHeight = 0;

  constructor(
    private overlay: HTMLElement,
    private canvas: HTMLCanvasElement,
    private resizeHandle: HTMLElement
  ) {
    this.loadPosition();
    this.setupEventListeners();
  }

  /**
   * Загружает сохраненную позицию из localStorage
   */
  private loadPosition(): void {
    try {
      const saved = localStorage.getItem(STORAGE_KEYS.CAMERA_POSITION);
      if (saved) {
        const position: CameraPosition = JSON.parse(saved);
        this.applyPosition(position);
      } else {
        this.applyPosition(DEFAULT_POSITION);
      }
    } catch (err) {
      console.error('[CameraController] Ошибка загрузки позиции:', err);
      this.applyPosition(DEFAULT_POSITION);
    }
  }

  /**
   * Сохраняет текущую позицию в localStorage
   */
  private savePosition(): void {
    try {
      const position: CameraPosition = {
        x: parseInt(this.overlay.style.left) || DEFAULT_POSITION.x,
        y: parseInt(this.overlay.style.top) || DEFAULT_POSITION.y,
        width: this.overlay.offsetWidth,
        height: this.overlay.offsetHeight,
      };
      localStorage.setItem(STORAGE_KEYS.CAMERA_POSITION, JSON.stringify(position));
    } catch (err) {
      console.error('[CameraController] Ошибка сохранения позиции:', err);
    }
  }

  /**
   * Применяет позицию к элементу
   */
  private applyPosition(position: CameraPosition): void {
    this.overlay.style.left = `${position.x}px`;
    this.overlay.style.top = `${position.y}px`;
    this.overlay.style.width = `${position.width}px`;
    this.overlay.style.height = `${position.height}px`;
  }

  /**
   * Настраивает обработчики событий
   */
  private setupEventListeners(): void {
    // Перетаскивание окна камеры
    this.canvas.addEventListener('mousedown', this.onDragStart.bind(this));
    document.addEventListener('mousemove', this.onDragMove.bind(this));
    document.addEventListener('mouseup', this.onDragEnd.bind(this));

    // Изменение размера
    this.resizeHandle.addEventListener('mousedown', this.onResizeStart.bind(this));
    document.addEventListener('mousemove', this.onResizeMove.bind(this));
    document.addEventListener('mouseup', this.onResizeEnd.bind(this));

    // Touch events для мобильных устройств
    this.canvas.addEventListener('touchstart', this.onTouchDragStart.bind(this));
    document.addEventListener('touchmove', this.onTouchDragMove.bind(this));
    document.addEventListener('touchend', this.onDragEnd.bind(this));

    this.resizeHandle.addEventListener('touchstart', this.onTouchResizeStart.bind(this));
    document.addEventListener('touchmove', this.onTouchResizeMove.bind(this));
    document.addEventListener('touchend', this.onResizeEnd.bind(this));
  }

  // ==================== Перетаскивание ====================

  private onDragStart(e: MouseEvent): void {
    if (this.isResizing) return;
    
    this.isDragging = true;
    this.startX = e.clientX;
    this.startY = e.clientY;
    this.startLeft = parseInt(this.overlay.style.left) || 0;
    this.startTop = parseInt(this.overlay.style.top) || 0;
    
    this.overlay.style.cursor = 'grabbing';
    e.preventDefault();
  }

  private onDragMove(e: MouseEvent): void {
    if (!this.isDragging) return;

    const deltaX = e.clientX - this.startX;
    const deltaY = e.clientY - this.startY;

    const newLeft = this.startLeft + deltaX;
    const newTop = this.startTop + deltaY;

    // Ограничиваем положение, чтобы окно не уходило за границы
    const maxLeft = window.innerWidth - this.overlay.offsetWidth;
    const maxTop = window.innerHeight - this.overlay.offsetHeight;

    this.overlay.style.left = `${Math.max(0, Math.min(newLeft, maxLeft))}px`;
    this.overlay.style.top = `${Math.max(0, Math.min(newTop, maxTop))}px`;
  }

  private onDragEnd(): void {
    if (this.isDragging) {
      this.isDragging = false;
      this.overlay.style.cursor = 'grab';
      this.savePosition();
    }
  }

  // ==================== Изменение размера ====================

  private onResizeStart(e: MouseEvent): void {
    this.isResizing = true;
    this.startX = e.clientX;
    this.startY = e.clientY;
    this.startWidth = this.overlay.offsetWidth;
    this.startHeight = this.overlay.offsetHeight;
    
    e.stopPropagation();
    e.preventDefault();
  }

  private onResizeMove(e: MouseEvent): void {
    if (!this.isResizing) return;

    const deltaX = e.clientX - this.startX;
    const deltaY = e.clientY - this.startY;

    const newWidth = Math.max(200, this.startWidth + deltaX);
    const newHeight = Math.max(150, this.startHeight + deltaY);

    this.overlay.style.width = `${newWidth}px`;
    this.overlay.style.height = `${newHeight}px`;
  }

  private onResizeEnd(): void {
    if (this.isResizing) {
      this.isResizing = false;
      this.savePosition();
    }
  }

  // ==================== Touch events ====================

  private onTouchDragStart(e: TouchEvent): void {
    if (this.isResizing || e.touches.length !== 1) return;
    
    const touch = e.touches[0];
    this.isDragging = true;
    this.startX = touch.clientX;
    this.startY = touch.clientY;
    this.startLeft = parseInt(this.overlay.style.left) || 0;
    this.startTop = parseInt(this.overlay.style.top) || 0;
    
    e.preventDefault();
  }

  private onTouchDragMove(e: TouchEvent): void {
    if (!this.isDragging || e.touches.length !== 1) return;

    const touch = e.touches[0];
    const deltaX = touch.clientX - this.startX;
    const deltaY = touch.clientY - this.startY;

    const newLeft = this.startLeft + deltaX;
    const newTop = this.startTop + deltaY;

    const maxLeft = window.innerWidth - this.overlay.offsetWidth;
    const maxTop = window.innerHeight - this.overlay.offsetHeight;

    this.overlay.style.left = `${Math.max(0, Math.min(newLeft, maxLeft))}px`;
    this.overlay.style.top = `${Math.max(0, Math.min(newTop, maxTop))}px`;
  }

  private onTouchResizeStart(e: TouchEvent): void {
    if (e.touches.length !== 1) return;
    
    const touch = e.touches[0];
    this.isResizing = true;
    this.startX = touch.clientX;
    this.startY = touch.clientY;
    this.startWidth = this.overlay.offsetWidth;
    this.startHeight = this.overlay.offsetHeight;
    
    e.stopPropagation();
    e.preventDefault();
  }

  private onTouchResizeMove(e: TouchEvent): void {
    if (!this.isResizing || e.touches.length !== 1) return;

    const touch = e.touches[0];
    const deltaX = touch.clientX - this.startX;
    const deltaY = touch.clientY - this.startY;

    const newWidth = Math.max(200, this.startWidth + deltaX);
    const newHeight = Math.max(150, this.startHeight + deltaY);

    this.overlay.style.width = `${newWidth}px`;
    this.overlay.style.height = `${newHeight}px`;
  }

  /**
   * Сбрасывает позицию к значениям по умолчанию
   */
  public resetPosition(): void {
    this.applyPosition(DEFAULT_POSITION);
    this.savePosition();
  }
}
