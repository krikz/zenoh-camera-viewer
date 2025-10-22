/**
 * Утилиты для парсинга CDR сообщений
 */

import { parseCDRBytes } from '@mono424/cdr-ts';
import { LOG_CONFIG } from '../config';

/**
 * Декодирует base64 строку в Uint8Array
 */
export function base64ToBytes(base64: string): Uint8Array {
  const binaryString = atob(base64);
  const bytes = new Uint8Array(binaryString.length);
  for (let i = 0; i < binaryString.length; i++) {
    bytes[i] = binaryString.charCodeAt(i);
  }
  return bytes;
}

/**
 * Парсит Zenoh сообщение с CDR данными
 */
export function parseZenohMessage<T>(
  eventData: string,
  schema: any,
  maxSequenceSize: number = 10_000
): T | null {
  try {
    const sample = JSON.parse(eventData) as { value: string };
    if (!sample.value) return null;

    const bytes = base64ToBytes(sample.value);
    const parsed = parseCDRBytes(bytes, schema, { maxSequenceSize });
    
    return parsed.payload as T;
  } catch (err) {
    if (LOG_CONFIG.DEBUG) {
      console.error(`${LOG_CONFIG.PREFIXES.PARSER} Ошибка парсинга:`, err);
    }
    return null;
  }
}

/**
 * Безопасное извлечение данных из MessageEvent
 */
export function extractMessageData(event: MessageEvent): string | null {
  try {
    return event.data;
  } catch (err) {
    if (LOG_CONFIG.DEBUG) {
      console.error(`${LOG_CONFIG.PREFIXES.PARSER} Ошибка извлечения данных:`, err);
    }
    return null;
  }
}
