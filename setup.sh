#!/bin/bash
# setup.sh
# =========
# Автоматическая установка всех зависимостей для проекта G1 Avatar.
# Запускать ОДИН РАЗ после первой установки Ubuntu в WSL2.
#
# Использование:
#   chmod +x setup.sh
#   ./setup.sh
#
# Автор: Андреюк М.О., Жук Б.Д. (Группа ИИ-25, БрГТУ, 2026)

set -e  # Прервать при любой ошибке

echo "============================================================"
echo "  G1 Avatar — Установка зависимостей"
echo "============================================================"

# ─── 1. Системные пакеты ─────────────────────────────────────────────────
echo ""
echo "[1/6] Обновление системы и установка пакетов..."
sudo apt update && sudo apt upgrade -y
sudo apt install -y \
    python3 python3-pip python3-venv \
    git cmake build-essential \
    libssl-dev libffi-dev python3-dev \
    net-tools iputils-ping

# ─── 2. Виртуальное окружение Python ─────────────────────────────────────
echo ""
echo "[2/6] Создание виртуального окружения Python..."
PROJECT_DIR="$HOME/g1_avatar_project"
mkdir -p "$PROJECT_DIR"
cd "$PROJECT_DIR"

python3 -m venv venv
source venv/bin/activate

pip install --upgrade pip
pip install \
    websockets \
    numpy \
    scipy \
    asyncio

# ─── 3. Unitree SDK2 ──────────────────────────────────────────────────────
echo ""
echo "[3/6] Установка Unitree SDK2..."
if [ ! -d "$PROJECT_DIR/unitree_sdk2_python" ]; then
    git clone https://github.com/unitreerobotics/unitree_sdk2_python.git
fi
cd unitree_sdk2_python
pip install -e .
cd "$PROJECT_DIR"

# ─── 4. CycloneDDS ───────────────────────────────────────────────────────
echo ""
echo "[4/6] Сборка CycloneDDS 0.10.x..."
if [ ! -d "$HOME/cyclonedds" ]; then
    cd ~
    git clone https://github.com/eclipse-cyclonedds/cyclonedds -b releases/0.10.x
    cd cyclonedds
    mkdir -p build install
    cd build
    cmake .. -DCMAKE_INSTALL_PREFIX=../install
    cmake --build . --target install
    cd "$PROJECT_DIR"
else
    echo "  CycloneDDS уже установлен, пропускаем."
fi

# ─── 5. Переменные окружения ──────────────────────────────────────────────
echo ""
echo "[5/6] Настройка переменных окружения..."
CYCLONE_EXPORT='export CYCLONEDDS_HOME="$HOME/cyclonedds/install"'
if ! grep -qF "CYCLONEDDS_HOME" "$HOME/.bashrc"; then
    echo "$CYCLONE_EXPORT" >> "$HOME/.bashrc"
    echo "  Добавлено CYCLONEDDS_HOME в ~/.bashrc"
fi
export CYCLONEDDS_HOME="$HOME/cyclonedds/install"

# ─── 6. Копирование файлов проекта ───────────────────────────────────────
echo ""
echo "[6/6] Копирование файлов проекта..."
# Скрипт предполагает, что g1_avatar_controller.py лежит рядом с setup.sh
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cp -f "$SCRIPT_DIR/g1_avatar_controller.py" "$PROJECT_DIR/" 2>/dev/null || \
    echo "  ВНИМАНИЕ: g1_avatar_controller.py не найден рядом с setup.sh"
cp -f "$SCRIPT_DIR/run.sh" "$PROJECT_DIR/" 2>/dev/null || true
chmod +x "$PROJECT_DIR/run.sh" 2>/dev/null || true

echo ""
echo "============================================================"
echo "  Установка завершена!"
echo ""
echo "  Следующий шаг: запустите ./run.sh"
echo "  (или вручную: cd ~/g1_avatar_project && ./run.sh)"
echo "============================================================"
