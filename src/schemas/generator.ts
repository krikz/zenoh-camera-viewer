/**
 * Генератор CDR схем для ROS сообщений
 * Упрощает создание новых схем сообщений
 */

export interface SchemaField {
  index: number;
  value: SchemaValue;
}

export type SchemaValue =
  | { type: 'uint' | 'int' | 'float'; len: 8 | 16 | 32 | 64; format?: 'number' | 'bigint' }
  | { type: 'string' }
  | { type: 'sequence'; itemSchema: SchemaValue }
  | { type: 'dictionary'; items: Record<string, SchemaField> };

/**
 * Базовые типы ROS
 */
const baseTypes = {
  uint8: { type: 'uint', len: 8, format: 'number' } as const,
  uint16: { type: 'uint', len: 16, format: 'number' } as const,
  uint32: { type: 'uint', len: 32, format: 'number' } as const,
  uint64: { type: 'uint', len: 64, format: 'bigint' } as const,
  int8: { type: 'int', len: 8, format: 'number' } as const,
  int16: { type: 'int', len: 16, format: 'number' } as const,
  int32: { type: 'int', len: 32, format: 'number' } as const,
  int64: { type: 'int', len: 64, format: 'bigint' } as const,
  float32: { type: 'float', len: 32 } as const,
  float64: { type: 'float', len: 64 } as const,
  string: { type: 'string' } as const,
};

/**
 * Создает поле словаря со схемой
 */
export function field(index: number, value: SchemaValue): SchemaField {
  return { index, value };
}

/**
 * Создает словарь (struct)
 */
export function dictionary(items: Record<string, SchemaField>) {
  return {
    type: 'dictionary' as const,
    items,
  };
}

/**
 * Создает последовательность (array)
 */
export function sequence(itemSchema: SchemaValue) {
  return {
    type: 'sequence' as const,
    itemSchema,
  };
}

/**
 * Стандартная схема Time
 */
export const timeSchema = dictionary({
  sec: field(0, baseTypes.uint32),
  nanosec: field(1, baseTypes.uint32),
});

/**
 * Стандартная схема Header
 */
export const headerSchema = dictionary({
  stamp: field(0, timeSchema),
  frame_id: field(1, baseTypes.string),
});

/**
 * Схема Vector3
 */
export const vector3Schema = dictionary({
  x: field(0, baseTypes.float64),
  y: field(1, baseTypes.float64),
  z: field(2, baseTypes.float64),
});

/**
 * Схема Quaternion
 */
export const quaternionSchema = dictionary({
  x: field(0, baseTypes.float64),
  y: field(1, baseTypes.float64),
  z: field(2, baseTypes.float64),
  w: field(3, baseTypes.float64),
});

/**
 * Схема Point
 */
export const pointSchema = dictionary({
  x: field(0, baseTypes.float64),
  y: field(1, baseTypes.float64),
  z: field(2, baseTypes.float64),
});

/**
 * Схема Pose
 */
export const poseSchema = dictionary({
  position: field(0, pointSchema),
  orientation: field(1, quaternionSchema),
});

/**
 * Схема PoseStamped
 */
export const poseStampedSchema = dictionary({
  header: field(0, headerSchema),
  pose: field(1, poseSchema),
});

/**
 * Схема Transform
 */
export const transformSchema = dictionary({
  translation: field(0, vector3Schema),
  rotation: field(1, quaternionSchema),
});

/**
 * Экспорт базовых типов для использования
 */
export { baseTypes };
