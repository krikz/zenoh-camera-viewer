import { defineConfig } from 'vite';
import wasm from 'vite-plugin-wasm';
import { fileURLToPath, URL } from 'node:url';

export default defineConfig({
  plugins: [
    wasm(), // <-- добавляем поддержку .wasm
  ],
  // Чтобы избежать предупреждений о top-level await
  worker: {
    format: 'es',
  },
  // Алиасы для удобного импорта
  resolve: {
    alias: {
      '@': fileURLToPath(new URL('./src', import.meta.url)),
      '@config': fileURLToPath(new URL('./src/config', import.meta.url)),
      '@types': fileURLToPath(new URL('./src/types', import.meta.url)),
      '@schemas': fileURLToPath(new URL('./src/schemas', import.meta.url)),
      '@services': fileURLToPath(new URL('./src/services', import.meta.url)),
      '@renderers': fileURLToPath(new URL('./src/renderers', import.meta.url)),
      '@utils': fileURLToPath(new URL('./src/utils', import.meta.url)),
      '@ui': fileURLToPath(new URL('./src/ui', import.meta.url)),
    },
  },
});