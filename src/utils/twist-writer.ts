/**
 * Утилита для сериализации Twist сообщений в CDR формат
 */

import { MessageWriter } from '@lichtblick/omgidl-serialization';
import { twistDefinition } from '../../idl';
import type { Twist } from '../types';

// Создаём writer один раз (можно кэшировать)
const twistWriter = new MessageWriter(
  'geometry_msgs::Twist',
  twistDefinition,
  { kind: 0x01 } // XCDR
);

/**
 * Сериализует Twist сообщение в CDR формат
 */
export function serializeTwist(linear: number, angular: number): Uint8Array {
  const twist: Twist = {
    linear: { x: linear, y: 0, z: 0 },
    angular: { x: 0, y: 0, z: angular },
  };

  return twistWriter.writeMessage(twist);
}
