/**
 * Zenoh Client для работы с REST API и SSE
 */

import { ZENOH_CONFIG, CDR_LIMITS, ROS_TOPICS, LOG_CONFIG } from '../config';
import { logger } from '../utils';
import type { FeedConfig } from '../types';

export type MessageHandler<T> = (message: T) => void;
export type ErrorHandler = (error: Error) => void;

/**
 * Менеджер для одного EventSource подключения
 */
class FeedManager<T> {
  private eventSource: EventSource | null = null;
  private reconnectAttempts = 0;
  private reconnectTimer: number | null = null;
  private isActive = false;

  constructor(
    private config: FeedConfig,
    private messageHandler: MessageHandler<T>,
    private errorHandler?: ErrorHandler
  ) {}

  /**
   * Запускает подключение
   */
  start(): void {
    this.isActive = true;
    this.connect();
  }

  /**
   * Останавливает подключение
   */
  stop(): void {
    this.isActive = false;
    this.cleanup();
  }

  /**
   * Устанавливает подключение к SSE
   */
  private connect(): void {
    const url = `${ZENOH_CONFIG.REST_BASE}/robots/${this.config.robotName}/${this.config.topic}`;

    logger.debug(LOG_CONFIG.PREFIXES.ZENOH, `Подключение к ${url}`);

    this.eventSource = new EventSource(url);

    this.eventSource.addEventListener('open', () => {
      logger.info(LOG_CONFIG.PREFIXES.ZENOH, `Подключено к ${this.config.topic}`);
      this.reconnectAttempts = 0;
    });

    this.eventSource.addEventListener('error', (err) => {
      logger.error(LOG_CONFIG.PREFIXES.ZENOH, `Ошибка SSE ${this.config.topic}:`, err);
      
      if (this.errorHandler) {
        this.errorHandler(new Error(`SSE error for ${this.config.topic}`));
      }

      this.handleError();
    });

    this.eventSource.addEventListener('PUT', (event: MessageEvent) => {
      try {
        this.messageHandler(event.data as T);
      } catch (err) {
        logger.error(
          LOG_CONFIG.PREFIXES.ZENOH,
          `Ошибка обработки сообщения ${this.config.topic}:`,
          err
        );
      }
    });
  }

  /**
   * Обрабатывает ошибку подключения
   */
  private handleError(): void {
    this.cleanup();

    if (!this.isActive) return;

    if (this.reconnectAttempts < ZENOH_CONFIG.MAX_RECONNECT_ATTEMPTS) {
      this.reconnectAttempts++;
      
      logger.warn(
        LOG_CONFIG.PREFIXES.ZENOH,
        `Попытка переподключения ${this.reconnectAttempts}/${ZENOH_CONFIG.MAX_RECONNECT_ATTEMPTS}`
      );

      this.reconnectTimer = window.setTimeout(() => {
        if (this.isActive) {
          this.connect();
        }
      }, ZENOH_CONFIG.RECONNECT_DELAY);
    } else {
      logger.error(
        LOG_CONFIG.PREFIXES.ZENOH,
        `Превышено максимальное количество попыток переподключения для ${this.config.topic}`
      );
    }
  }

  /**
   * Очищает ресурсы
   */
  private cleanup(): void {
    if (this.eventSource) {
      this.eventSource.close();
      this.eventSource = null;
    }

    if (this.reconnectTimer !== null) {
      clearTimeout(this.reconnectTimer);
      this.reconnectTimer = null;
    }
  }
}

/**
 * Главный клиент для работы с Zenoh
 */
export class ZenohClient {
  private feeds = new Map<string, FeedManager<any>>();

  /**
   * Получает список роботов
   */
  async fetchRobots(): Promise<string[]> {
    try {
      const url = `${ZENOH_CONFIG.REST_BASE}/robots/**`;
      const resp = await fetch(url);
      
      if (!resp.ok) {
        throw new Error(`HTTP ${resp.status}`);
      }

      const data: Array<{ key: string }> = await resp.json();

      // Извлекаем уникальные имена роботов
      const robotsSet = new Set<string>();
      for (const item of data) {
        const parts = item.key.split('/');
        if (parts.length > 1 && parts[0] === 'robots') {
          const robotName = parts[1];
          if (robotName) robotsSet.add(robotName);
        }
      }

      return Array.from(robotsSet).sort();
    } catch (err) {
      logger.error(LOG_CONFIG.PREFIXES.ZENOH, 'Ошибка получения списка роботов:', err);
      throw err;
    }
  }

  /**
   * Подписывается на топик
   */
  subscribe<T>(
    robotName: string,
    topic: string,
    messageHandler: MessageHandler<T>,
    errorHandler?: ErrorHandler,
    maxSequenceSize?: number
  ): string {
    const feedId = `${robotName}/${topic}`;
    
    // Если уже подписаны, отписываемся
    if (this.feeds.has(feedId)) {
      this.unsubscribe(feedId);
    }

    const config: FeedConfig = {
      robotName,
      topic,
      maxSequenceSize,
    };

    const feed = new FeedManager<T>(config, messageHandler, errorHandler);
    this.feeds.set(feedId, feed);
    feed.start();

    return feedId;
  }

  /**
   * Отписывается от топика
   */
  unsubscribe(feedId: string): void {
    const feed = this.feeds.get(feedId);
    if (feed) {
      feed.stop();
      this.feeds.delete(feedId);
    }
  }

  /**
   * Отписывается от всех топиков робота
   */
  unsubscribeRobot(robotName: string): void {
    const feedIds = Array.from(this.feeds.keys()).filter((id) =>
      id.startsWith(`${robotName}/`)
    );
    
    for (const feedId of feedIds) {
      this.unsubscribe(feedId);
    }
  }

  /**
   * Отписывается от всех топиков
   */
  unsubscribeAll(): void {
    for (const feedId of this.feeds.keys()) {
      this.unsubscribe(feedId);
    }
  }

  /**
   * Публикует сообщение в топик
   */
  async publish(
    robotName: string,
    topic: string,
    data: Uint8Array
  ): Promise<void> {
    try {
      const url = `${ZENOH_CONFIG.REST_BASE}/robots/${robotName}/${topic}`;
      
      const resp = await fetch(url, {
        method: 'PUT',
        headers: {
          'Content-Type': 'application/octet-stream',
        },
        body: data,
      });

      if (!resp.ok) {
        throw new Error(`HTTP ${resp.status}`);
      }

      logger.debug(LOG_CONFIG.PREFIXES.ZENOH, `Опубликовано в ${topic}`);
    } catch (err) {
      logger.error(LOG_CONFIG.PREFIXES.ZENOH, `Ошибка публикации в ${topic}:`, err);
      throw err;
    }
  }
}
