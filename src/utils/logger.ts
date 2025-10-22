/**
 * Простая система логирования
 */

import { LOG_CONFIG } from '../config';

export enum LogLevel {
  DEBUG = 0,
  INFO = 1,
  WARN = 2,
  ERROR = 3,
}

class Logger {
  private level: LogLevel = LOG_CONFIG.DEBUG ? LogLevel.DEBUG : LogLevel.INFO;

  setLevel(level: LogLevel) {
    this.level = level;
  }

  debug(prefix: string, message: string, ...args: any[]) {
    if (this.level <= LogLevel.DEBUG) {
      console.debug(`${prefix} ${message}`, ...args);
    }
  }

  info(prefix: string, message: string, ...args: any[]) {
    if (this.level <= LogLevel.INFO) {
      console.info(`${prefix} ${message}`, ...args);
    }
  }

  warn(prefix: string, message: string, ...args: any[]) {
    if (this.level <= LogLevel.WARN) {
      console.warn(`${prefix} ${message}`, ...args);
    }
  }

  error(prefix: string, message: string, ...args: any[]) {
    if (this.level <= LogLevel.ERROR) {
      console.error(`${prefix} ${message}`, ...args);
    }
  }
}

export const logger = new Logger();
