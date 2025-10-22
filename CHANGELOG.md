# Changelog

Все заметные изменения в этом проекте будут документироваться в этом файле.

Формат основан на [Keep a Changelog](https://keepachangelog.com/ru/1.0.0/),
и этот проект придерживается [Semantic Versioning](https://semver.org/lang/ru/).

## [Unreleased]

### Added
- Файл `.gitignore` для правильного управления версиями
- Подробное `CODE_REVIEW.md` с рекомендациями по улучшению
- `tsconfig.json` с строгими настройками TypeScript
- `CONTRIBUTING.md` с правилами для контрибьюторов
- `CHANGELOG.md` для отслеживания изменений
- Badges в README для версий технологий
- Структура проекта в README
- Информация о лицензии и авторе в README

### Changed
- Обновлен `package.json` с описанием, keywords и repository URL
- Улучшена документация в `README.md`

## [1.0.0] - 2025-10-22

### Added
- Начальный релиз проекта
- Веб-интерфейс для управления роботами через Zenoh
- Отображение карты (OccupancyGrid)
- Отображение лидара (LaserScan)
- Отображение видео с камеры (Image)
- Навигация по карте (NavigateToPose)
- Управление с геймпада
- Режим автоматического исследования (explore)
- Работа с вейпоинтами
- Режим "змейки" для навигации
- Поддержка SSE (Server-Sent Events)
- Визуализация траектории движения
- Сохранение позиции камеры в localStorage
- Преобразование координат map ↔ odom
- CDR сериализация/десериализация

[Unreleased]: https://github.com/krikz/zenoh-camera-viewer/compare/v1.0.0...HEAD
[1.0.0]: https://github.com/krikz/zenoh-camera-viewer/releases/tag/v1.0.0
