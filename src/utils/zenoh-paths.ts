/**
 * Утилиты для работы с путями Zenoh
 */

/**
 * Структура пути Zenoh
 * Формат: robots/{robot_name}/{domain}/{topic_path}/{message_type}/{hash_status}
 * Пример: robots/RBXU100001/0/scan/sensor_msgs::msg::dds_::LaserScan_/TypeHashNotSupported
 */
export interface ZenohPath {
  /** Имя робота */
  robotName: string;
  /** Домен ROS (обычно 0) */
  domain: string;
  /** Путь топика */
  topicPath: string;
  /** Тип сообщения (опционально) */
  messageType?: string;
  /** Статус хэша (опционально) */
  hashStatus?: string;
  /** Полный оригинальный ключ */
  fullKey: string;
}

/**
 * Парсит путь Zenoh в структурированный объект
 * 
 * Примеры:
 * - robots/RBXU100001/0/scan/sensor_msgs::msg::dds_::LaserScan_/TypeHashNotSupported
 * - robots/RBXU100001/0/map/nav_msgs::msg::dds_::OccupancyGrid_/TypeHashNotSupported
 * - robots/RBXU100001/scan (старый формат, для обратной совместимости)
 */
export function parseZenohPath(key: string): ZenohPath | null {
  if (!key.startsWith('robots/')) {
    return null;
  }

  const parts = key.split('/');
  
  // Минимум должно быть: robots/{name}/{...}
  if (parts.length < 3) {
    return null;
  }

  const robotName = parts[1];
  
  // Проверяем, есть ли домен (число после имени робота)
  const hasDomain = !isNaN(parseInt(parts[2]));
  
  if (hasDomain) {
    // Новый формат с доменом
    // robots/{robot}/{domain}/{topic_path}/{message_type}/{hash_status}
    const domain = parts[2];
    
    // Ищем части с типом сообщения (содержат ::)
    let messageTypeIndex = -1;
    for (let i = 3; i < parts.length; i++) {
      if (parts[i].includes('::')) {
        messageTypeIndex = i;
        break;
      }
    }
    
    let topicPath: string;
    let messageType: string | undefined;
    let hashStatus: string | undefined;
    
    if (messageTypeIndex > 0) {
      // Топик = всё между доменом и типом сообщения
      topicPath = parts.slice(3, messageTypeIndex).join('/');
      messageType = parts[messageTypeIndex];
      
      // Хэш статус (если есть) идёт после типа
      if (messageTypeIndex + 1 < parts.length) {
        hashStatus = parts[messageTypeIndex + 1];
      }
    } else {
      // Нет типа сообщения, всё остальное - топик
      topicPath = parts.slice(3).join('/');
    }
    
    return {
      robotName,
      domain,
      topicPath,
      messageType,
      hashStatus,
      fullKey: key,
    };
  } else {
    // Старый формат без домена
    // robots/{robot}/{topic_path}
    const topicPath = parts.slice(2).join('/');
    
    return {
      robotName,
      domain: '0', // Предполагаем домен 0
      topicPath,
      fullKey: key,
    };
  }
}

/**
 * Извлекает имя робота из пути Zenoh
 */
export function extractRobotName(key: string): string | null {
  const parsed = parseZenohPath(key);
  return parsed?.robotName || null;
}

/**
 * Извлекает путь топика из пути Zenoh
 */
export function extractTopicPath(key: string): string | null {
  const parsed = parseZenohPath(key);
  return parsed?.topicPath || null;
}

/**
 * Проверяет, соответствует ли путь Zenoh заданному роботу и топику
 * 
 * @param key - Полный путь Zenoh
 * @param robotName - Имя робота для сравнения
 * @param topicPattern - Паттерн топика (может содержать *)
 * @returns true если путь соответствует
 */
export function matchesRobotAndTopic(
  key: string,
  robotName: string,
  topicPattern: string
): boolean {
  const parsed = parseZenohPath(key);
  
  if (!parsed || parsed.robotName !== robotName) {
    return false;
  }
  
  // Простое совпадение
  if (topicPattern === parsed.topicPath) {
    return true;
  }
  
  // Паттерн с wildcard
  if (topicPattern.includes('*')) {
    const regex = new RegExp(
      '^' + topicPattern.replace(/\*/g, '.*') + '$'
    );
    return regex.test(parsed.topicPath);
  }
  
  return false;
}

/**
 * Строит полный путь Zenoh для подписки
 * 
 * @param robotName - Имя робота
 * @param topic - Топик (без домена)
 * @param domain - Домен ROS (по умолчанию из ZENOH_CONFIG)
 * @returns Полный путь для SSE подписки
 */
export function buildZenohPath(
  robotName: string,
  topic: string,
  domain?: string
): string {
  // Импортируем конфиг здесь чтобы избежать циклических зависимостей
  const actualDomain = domain || '0';
  
  // Формат: robots/{robot}/{domain}/{topic}/**
  // ** в конце нужен чтобы получать все варианты с разными типами
  // Нормализуем части: убираем ведущие/концевые слеши чтобы избежать двойных слешей
  const domainClean = String(actualDomain).replace(/^\/+|\/+$/g, '');
  const topicClean = String(topic || '').replace(/^\/+|\/+$/g, '');

  if (topicClean.length === 0) {
    return `robots/${robotName}/${domainClean}/**`;
  }

  return `robots/${robotName}/${domainClean}/${topicClean}/**`;
}

/**
 * Строит путь для получения списка топиков робота
 */
export function buildRobotTopicsPath(robotName: string): string {
  return `robots/${robotName}/**`;
}

/**
 * Фильтрует список ключей по топику
 * 
 * @param keys - Список ключей Zenoh
 * @param robotName - Имя робота
 * @param topicPattern - Паттерн топика
 * @returns Отфильтрованные ключи
 */
export function filterKeysByTopic(
  keys: string[],
  robotName: string,
  topicPattern: string
): string[] {
  return keys.filter(key => 
    matchesRobotAndTopic(key, robotName, topicPattern)
  );
}

/**
 * Группирует ключи по топикам
 * 
 * @param keys - Список ключей Zenoh
 * @returns Map<topicPath, keys[]>
 */
export function groupKeysByTopic(keys: string[]): Map<string, string[]> {
  const groups = new Map<string, string[]>();
  
  for (const key of keys) {
    const parsed = parseZenohPath(key);
    if (parsed) {
      const existing = groups.get(parsed.topicPath) || [];
      existing.push(key);
      groups.set(parsed.topicPath, existing);
    }
  }
  
  return groups;
}

/**
 * Извлекает все уникальные топики для робота
 */
export function extractUniqueTopics(keys: string[], robotName: string): string[] {
  const topics = new Set<string>();
  
  for (const key of keys) {
    const parsed = parseZenohPath(key);
    if (parsed && parsed.robotName === robotName) {
      topics.add(parsed.topicPath);
    }
  }
  
  return Array.from(topics).sort();
}
