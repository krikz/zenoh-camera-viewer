/**
 * Zenoh Client для работы с REST API и SSE
 */

import { ZENOH_CONFIG, LOG_CONFIG } from '../config';
import { logger, extractRobotName, buildZenohPath } from '../utils';
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
    // Используем новый формат пути с доменом и wildcard
    // robots/{robot}/{domain}/{topic}/**
    const url = `${ZENOH_CONFIG.REST_BASE}/${buildZenohPath(
      this.config.robotName,
      this.config.topic,
      ZENOH_CONFIG.ROS_DOMAIN
    )}`;

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
  private robotFeed: EventSource | null = null;
  private currentRobotName: string = '';
  private unifiedMessageHandler: ((event: MessageEvent) => void) | null = null;

  /**
   * Подписывается на робота целиком (все топики)
   * Это единственная SSE подписка на robots/{robotName}/**
   */
  subscribeToRobot(
    robotName: string,
    messageHandler: (key: string, data: string) => void
  ): void {
    // Закрываем предыдущую подписку если есть
    this.unsubscribeRobot();

    this.currentRobotName = robotName;
    
    // Единственный SSE на всего робота
    const keyExpr = `robots/${robotName}/**`;
    const url = `${ZENOH_CONFIG.REST_BASE}/${keyExpr}`;

    logger.debug(LOG_CONFIG.PREFIXES.ZENOH, `Подключение к ${url}`);

    this.robotFeed = new EventSource(url);

    this.robotFeed.addEventListener('open', () => {
      logger.info(LOG_CONFIG.PREFIXES.ZENOH, `Подключено к роботу ${robotName} (unified feed)`);
    });

    this.robotFeed.addEventListener('error', (err) => {
      logger.error(LOG_CONFIG.PREFIXES.ZENOH, `Ошибка SSE робота ${robotName}:`, err);
      
      if (this.robotFeed?.readyState === EventSource.CLOSED) {
        this.robotFeed = null;
      }
    });

    // Создаём обработчик один раз
    this.unifiedMessageHandler = (event: MessageEvent) => {
      try {
        const sample = JSON.parse(event.data) as { key: string; value: string };
        if (sample.key && sample.value) {
          // Передаём ключ и данные в роутер
          messageHandler(sample.key, sample.value);
        }
      } catch (err) {
        logger.error(LOG_CONFIG.PREFIXES.ZENOH, 'Ошибка парсинга unified сообщения:', err);
      }
    };

    this.robotFeed.addEventListener('PUT', this.unifiedMessageHandler);
  }

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

      // Извлекаем уникальные имена роботов из путей
      const robotsSet = new Set<string>();
      for (const item of data) {
        // Используем утилиту для парсинга пути
        const robotName = extractRobotName(item.key);
        if (robotName) {
          robotsSet.add(robotName);
        }
      }

      return Array.from(robotsSet).sort();
    } catch (err) {
      logger.error(LOG_CONFIG.PREFIXES.ZENOH, 'Ошибка получения списка роботов:', err);
      throw err;
    }
  }

  /**
   * Получает список доступных топиков камер для робота
   * Ищет топики содержащие: camera, image, rgb, compressed
   * Исключает camera_info топики
   */
  async fetchCameraTopics(robotName: string): Promise<string[]> {
    try {
      const url = `${ZENOH_CONFIG.REST_BASE}/robots/${robotName}/${ZENOH_CONFIG.ROS_DOMAIN}/**`;
      const resp = await fetch(url);
      
      if (!resp.ok) {
        throw new Error(`HTTP ${resp.status}`);
      }

      const data: Array<{ key: string }> = await resp.json();
      
      // Паттерны для поиска топиков с изображениями (НЕ camera_info)
      const imagePatterns = [
        /\/image/i,
        /\/compressed/i,
        /\/depth/i,
        /\/color/i,
      ];
      
      // Исключаем топики, которые НЕ содержат изображения
      const excludePatterns = [
        /camera_info/i,
        /\/imu\//i,
        /detection/i,
      ];

      // Извлекаем уникальные топики камер
      const cameraTopics = new Set<string>();
      
      for (const item of data) {
        const key = item.key;
        
        // Проверяем, что это топик с изображением
        const hasImage = imagePatterns.some(pattern => pattern.test(key));
        const isExcluded = excludePatterns.some(pattern => pattern.test(key));
        
        if (hasImage && !isExcluded) {
          // Извлекаем путь топика без префикса robots/{name}/{domain}/ и суффикса /{messageType}/{hashStatus}
          // Пример: robots/RBXU100001/0/camera/rgb/image_raw/compressed/sensor_msgs::msg::dds_::CompressedImage_/TypeHashNotSupported
          // Нужно извлечь: camera/rgb/image_raw/compressed
          
          // Разделяем по слешам и ищем, где начинается messageType (содержит ::)
          const parts = key.split('/');
          
          // Первые 3 части: robots, robotName, domain
          // Затем идет путь топика до messageType (который содержит ::)
          const topicParts: string[] = [];
          for (let i = 3; i < parts.length; i++) {
            if (parts[i].includes('::') || parts[i] === 'TypeHashNotSupported') {
              break; // Дошли до messageType или hashStatus
            }
            topicParts.push(parts[i]);
          }
          
          if (topicParts.length > 0) {
            const topicPath = topicParts.join('/');
            cameraTopics.add(topicPath);
          }
        }
      }

      const topics = Array.from(cameraTopics).sort();
      logger.info(LOG_CONFIG.PREFIXES.ZENOH, `Найдено топиков камер: ${topics.length}`, topics);
      return topics;
      return topics;
    } catch (err) {
      logger.error(LOG_CONFIG.PREFIXES.ZENOH, 'Ошибка получения топиков камер:', err);
      return [];
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
  unsubscribeRobot(): void {
    if (this.robotFeed) {
      if (this.unifiedMessageHandler) {
        this.robotFeed.removeEventListener('PUT', this.unifiedMessageHandler);
        this.unifiedMessageHandler = null;
      }
      this.robotFeed.close();
      this.robotFeed = null;
    }
    
    // Очищаем старые подписки если были
    const feedIds = Array.from(this.feeds.keys()).filter((id) =>
      id.startsWith(`${this.currentRobotName}/`)
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
