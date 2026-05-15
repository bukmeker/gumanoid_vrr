#!/usr/bin/env python3
"""
g1_avatar_controller.py
========================
Управление роботом Unitree G1 EDU в режиме аватара через Meta Quest 2.

Схема работы:
  Meta Quest 2  --[WebSocket / Wi-Fi 6]--> Этот сервер (ПК)
  Этот сервер   --[DDS / Ethernet]------> Unitree G1 EDU

Запуск:
  python3 g1_avatar_controller.py [сетевой_интерфейс]
  Пример: python3 g1_avatar_controller.py eth0

Автор: Андреюк М.О., Жук Б.Д. (Группа ИИ-25, БрГТУ, 2026)
"""

import asyncio
import json
import sys
import time
import math
import logging
import numpy as np
import websockets

from unitree_sdk2py.core.channel import ChannelFactoryInitialize
from unitree_sdk2py.g1.loco.g1_loco_client import LocoClient

# ─────────────────────────────────────────────────────────────────────────────
# Конфигурация
# ─────────────────────────────────────────────────────────────────────────────
ROBOT_INTERFACE = "eth0"          # Сетевой интерфейс к роботу
WS_HOST         = "0.0.0.0"      # Слушать на всех интерфейсах
WS_PORT         = 8765            # WebSocket порт
CONTROL_HZ      = 50              # Частота управляющего цикла (Гц)
WATCHDOG_SEC    = 0.200           # Таймаут связи с Quest 2 (сек)

# Ограничения безопасности (радианы)
NECK_PITCH_LIMIT  = math.radians(40)
NECK_YAW_LIMIT    = math.radians(40)
WRIST_LIMIT       = math.radians(75)
SHOULDER_LIMIT    = math.radians(120)

# Коэффициенты масштабирования
K_NECK    = 0.90   # Шея
K_TORSO   = 0.70   # Поясница
K_VX_MAX  = 0.40   # Макс. линейная скорость (м/с)
K_VY_MAX  = 0.25   # Макс. боковая скорость (м/с)
K_YAW_MAX = 0.80   # Макс. угловая скорость (рад/с)

# ─────────────────────────────────────────────────────────────────────────────
# Логирование
# ─────────────────────────────────────────────────────────────────────────────
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(message)s",
    datefmt="%H:%M:%S"
)
log = logging.getLogger("G1Avatar")


# ─────────────────────────────────────────────────────────────────────────────
# SafetyGuard — проверка допустимых диапазонов
# ─────────────────────────────────────────────────────────────────────────────
class SafetyGuard:
    """Проверяет, что вычисленные углы суставов находятся в безопасных пределах."""

    def check_neck(self, pitch: float, yaw: float) -> bool:
        return abs(pitch) <= NECK_PITCH_LIMIT and abs(yaw) <= NECK_YAW_LIMIT

    def check_shoulder(self, angle: float) -> bool:
        return abs(angle) <= SHOULDER_LIMIT

    def check_wrist(self, angle: float) -> bool:
        return abs(angle) <= WRIST_LIMIT

    def clamp_velocity(self, vx: float, vy: float, vyaw: float):
        vx   = max(-K_VX_MAX,  min(K_VX_MAX,  vx))
        vy   = max(-K_VY_MAX,  min(K_VY_MAX,  vy))
        vyaw = max(-K_YAW_MAX, min(K_YAW_MAX, vyaw))
        return vx, vy, vyaw


# ─────────────────────────────────────────────────────────────────────────────
# RetargetEngine — маппинг VR-поз на суставы робота
# ─────────────────────────────────────────────────────────────────────────────
class RetargetEngine:
    """
    Преобразует 6DOF-позы Meta Quest 2 в целевые углы суставов G1 EDU.
    Кватернион формат: [x, y, z, w]
    """

    @staticmethod
    def quat_to_euler(q) -> tuple:
        """Кватернион [x,y,z,w] → (pitch, yaw) в радианах (ZYX-конвенция)."""
        x, y, z, w = float(q[0]), float(q[1]), float(q[2]), float(q[3])
        # Pitch (вокруг X)
        sinr_cosp = 2.0 * (w * x + y * z)
        cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
        pitch = math.atan2(sinr_cosp, cosr_cosp)
        # Yaw (вокруг Y)
        siny_cosp = 2.0 * (w * y - z * x)
        yaw = math.asin(max(-1.0, min(1.0, siny_cosp)))
        return pitch, yaw

    def head_to_neck(self, head_quat) -> tuple:
        """
        Ориентация шлема → углы шеи робота.
        Возвращает (neck_pitch, neck_yaw) в радианах.
        """
        pitch, yaw = self.quat_to_euler(head_quat)
        return pitch * K_NECK, yaw * K_NECK

    def head_to_torso(self, head_quat) -> tuple:
        """
        Ориентация корпуса оператора → углы поясницы робота.
        Возвращает (torso_pitch, torso_yaw) в радианах.
        """
        pitch, yaw = self.quat_to_euler(head_quat)
        return pitch * K_TORSO, yaw * (K_TORSO * 0.6)

    def hand_ik(self, ctrl_pos, side: str = "right") -> tuple:
        """
        Обратная кинематика руки.
        Принимает позицию контроллера [x, y, z] (метры, относительно плеча).
        Возвращает (shoulder_pitch, elbow_angle, wrist_yaw) в радианах.

        Параметры сегментов руки G1 EDU:
          upper_arm = 0.35 м, forearm = 0.30 м
        """
        x, y, z = float(ctrl_pos[0]), float(ctrl_pos[1]), float(ctrl_pos[2])
        L1 = 0.35  # плечо
        L2 = 0.30  # предплечье

        dist = math.sqrt(x**2 + y**2 + z**2)
        dist = min(dist, L1 + L2 - 0.01)  # не выходить за длину руки

        # Угол в локте (закон косинусов)
        cos_elbow = (L1**2 + L2**2 - dist**2) / (2 * L1 * L2)
        elbow = math.acos(max(-1.0, min(1.0, cos_elbow)))

        # Угол подъёма плеча
        shoulder_pitch = math.atan2(z, math.sqrt(x**2 + y**2))

        # Поворот запястья
        mirror = 1.0 if side == "right" else -1.0
        wrist_yaw = math.atan2(y, x) * mirror

        return shoulder_pitch, elbow, wrist_yaw

    def stick_to_velocity(self, stick: list) -> tuple:
        """
        Аналоговый стик [x, y] → (vx, vy, vyaw).
        stick[1] — вперёд/назад, stick[0] — повороты.
        """
        sx = float(stick[0]) if stick else 0.0
        sy = float(stick[1]) if len(stick) > 1 else 0.0
        vx   =  sy * K_VX_MAX
        vy   =  0.0
        vyaw = -sx * K_YAW_MAX
        return vx, vy, vyaw


# ─────────────────────────────────────────────────────────────────────────────
# G1AvatarController — главный класс управления
# ─────────────────────────────────────────────────────────────────────────────
class G1AvatarController:
    """
    Основной контроллер системы аватара.

    Жизненный цикл:
      1. __init__  — подключение к роботу
      2. startup   — подъём в стойку
      3. run       — запуск WebSocket-сервера + управляющего цикла
      4. shutdown  — безопасная остановка
    """

    def __init__(self, interface: str = ROBOT_INTERFACE):
        log.info(f"Подключение к роботу через интерфейс: {interface}")
        ChannelFactoryInitialize(0, interface)

        self.client   = LocoClient()
        self.client.SetTimeout(10.0)
        self.client.Init()

        self.retarget    = RetargetEngine()
        self.safety      = SafetyGuard()
        self.latest_data = None
        self.last_vr_ts  = time.time()
        self.running     = True
        self.vr_connected = False

        log.info("Соединение с роботом установлено.")

    # ── Жизненный цикл ────────────────────────────────────────────────────

    async def startup(self):
        """Поднять робота в рабочую стойку."""
        log.info("Инициализация: поднимаю робота...")
        self.client.Squat2StandUp()
        await asyncio.sleep(3.5)
        log.info("Робот стоит. Система аватара готова.")

    async def shutdown(self):
        """Безопасная остановка."""
        log.info("Завершение работы...")
        self.running = False
        self.client.StopMove()
        await asyncio.sleep(0.3)
        self.client.Damp()
        log.info("Робот переведён в режим демпфирования. До свидания!")

    def emergency_stop(self):
        """Аварийная остановка — немедленно."""
        log.warning("!!! АВАРИЙНАЯ ОСТАНОВКА !!!")
        self.client.StopMove()
        self.client.ZeroTorque()

    # ── WebSocket-обработчик ───────────────────────────────────────────────

    async def ws_handler(self, websocket):
        """Принимает JSON-пакеты с данными трекинга от Meta Quest 2."""
        addr = websocket.remote_address
        log.info(f"VR-клиент подключён: {addr}")
        self.vr_connected = True

        try:
            async for message in websocket:
                try:
                    data = json.loads(message)
                except json.JSONDecodeError:
                    log.warning("Получен невалидный JSON-пакет, пропускаем.")
                    continue

                # Аварийная кнопка
                if data.get("emergency", False):
                    self.emergency_stop()
                    continue

                self.latest_data = data
                self.last_vr_ts  = time.time()

        except websockets.exceptions.ConnectionClosedError:
            log.warning(f"VR-клиент отключился: {addr}")
        finally:
            self.vr_connected = False
            self.latest_data  = None

    # ── Применение одного кадра ────────────────────────────────────────────

    def apply_frame(self, data: dict):
        """
        Применяет один кадр VR-данных к роботу.

        Ожидаемая структура data:
        {
          "head":    {"quat": [x,y,z,w], "pos": [x,y,z]},
          "right":   {"pos": [x,y,z], "quat": [x,y,z,w], "grip": 0.0..1.0},
          "left":    {"pos": [x,y,z], "quat": [x,y,z,w], "grip": 0.0..1.0},
          "stick_r": [x, y],
          "stick_l": [x, y],
          "emergency": false
        }
        """
        # ── Голова → шея
        head = data.get("head", {})
        if "quat" in head:
            neck_p, neck_y = self.retarget.head_to_neck(head["quat"])
            if not self.safety.check_neck(neck_p, neck_y):
                log.debug(f"Шея вне диапазона: pitch={math.degrees(neck_p):.1f}°, yaw={math.degrees(neck_y):.1f}°")
            # NOTE: отправка углов шеи через низкоуровневый Joint API
            # self.client.SetJointPositions({"neck_pitch": neck_p, "neck_yaw": neck_y})
            # В текущей версии SDK2 используем только LocoClient для локомоции

        # ── Правая рука → IK
        right = data.get("right", {})
        if "pos" in right:
            sp, elbow, wy = self.retarget.hand_ik(right["pos"], side="right")
            # Здесь можно передавать через ArmSDK или Dex3-1 SDK
            # self.client.SetArmJoints("right", sp, elbow, wy)

        # ── Левая рука → IK
        left = data.get("left", {})
        if "pos" in left:
            sp, elbow, wy = self.retarget.hand_ik(left["pos"], side="left")

        # ── Стик → движение ног
        stick = data.get("stick_r", [0.0, 0.0])
        vx, vy, vyaw = self.retarget.stick_to_velocity(stick)
        vx, vy, vyaw = self.safety.clamp_velocity(vx, vy, vyaw)

        if abs(vx) > 0.01 or abs(vy) > 0.01 or abs(vyaw) > 0.01:
            self.client.Move(vx, vy, vyaw, True)
        else:
            self.client.StopMove()

    # ── Управляющий цикл ──────────────────────────────────────────────────

    async def control_loop(self):
        """
        Цикл 50 Гц: проверяет watchdog и применяет последний кадр VR-данных.
        """
        dt = 1.0 / CONTROL_HZ
        log.info(f"Управляющий цикл запущен ({CONTROL_HZ} Гц, watchdog={WATCHDOG_SEC*1000:.0f} мс)")

        while self.running:
            elapsed = time.time() - self.last_vr_ts

            if self.vr_connected and elapsed > WATCHDOG_SEC:
                # Связь с Quest 2 потеряна — стоп
                log.warning(f"Watchdog: нет данных {elapsed*1000:.0f} мс — останавливаю робота")
                self.client.StopMove()

            elif self.latest_data is not None:
                try:
                    self.apply_frame(self.latest_data)
                except Exception as e:
                    log.error(f"Ошибка применения кадра: {e}")

            await asyncio.sleep(dt)

    # ── Главная точка входа ────────────────────────────────────────────────

    async def run(self):
        await self.startup()

        async with websockets.serve(
            self.ws_handler,
            WS_HOST,
            WS_PORT,
            ping_interval=5,
            ping_timeout=10
        ):
            log.info(f"WebSocket-сервер запущен: ws://{WS_HOST}:{WS_PORT}")
            log.info("Ожидаю подключения Meta Quest 2...")

            try:
                await self.control_loop()
            except asyncio.CancelledError:
                pass
            finally:
                await self.shutdown()


# ─────────────────────────────────────────────────────────────────────────────
# Точка входа
# ─────────────────────────────────────────────────────────────────────────────
def main():
    interface = sys.argv[1] if len(sys.argv) > 1 else ROBOT_INTERFACE

    log.info("=" * 55)
    log.info("  G1 EDU Avatar Controller — БрГТУ, Группа ИИ-25")
    log.info("=" * 55)
    log.info(f"  Интерфейс робота : {interface}")
    log.info(f"  WebSocket порт   : {WS_PORT}")
    log.info(f"  Частота цикла    : {CONTROL_HZ} Гц")
    log.info(f"  Watchdog         : {WATCHDOG_SEC*1000:.0f} мс")
    log.info("=" * 55)

    controller = G1AvatarController(interface=interface)

    try:
        asyncio.run(controller.run())
    except KeyboardInterrupt:
        log.info("Прерывание пользователем (Ctrl+C)")


if __name__ == "__main__":
    main()
