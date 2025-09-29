#!/bin/bash

# Запускаем zenohd в фоновом режиме с указанной конфигурацией
zenohd -c /etc/zenoh/config.json5 &
ZENOH_PID=$!

# Выводим информацию о запуске
echo "Zenoh router started with PID $ZENOH_PID"

# Даем время на инициализацию Zenoh (5 секунд)
echo "Ожидание инициализации Zenoh (5 секунд)..."
sleep 5

# Проверяем, запущен ли zenohd
if ! kill -0 $ZENOH_PID > /dev/null 2>&1; then
    echo "Ошибка: Zenoh роутер не запустился"
    exit 1
fi

echo "Zenoh роутер успешно запущен и инициализирован"

# Загружаем настройки ROS2
source /opt/ros/humble/setup.bash
source /ws/install/setup.bash

# Запускаем ROS2 приложение напрямую через ros2 launch
echo "Запуск ROS2 навигации в Gazebo..."
ros2 launch sam_bot_nav2_gz complete_navigation.launch.py

# После завершения основного приложения завершаем zenohd
echo "Завершение работы Zenoh роутера..."
kill $ZENOH_PID

# Ждем завершения процесса zenohd
wait $ZENOH_PID 2>/dev/null

echo "Контейнер завершил работу"