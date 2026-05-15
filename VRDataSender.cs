// VRDataSender.cs
// ================
// Unity-скрипт для Meta Quest 2.
// Считывает позиции шлема и контроллеров (6DOF) и отправляет
// JSON-пакеты на Python WebSocket-сервер каждый кадр.
//
// Зависимости:
//   - Meta XR All-in-One SDK (через Package Manager)
//   - NativeWebSocket (https://github.com/endel/NativeWebSocket)
//     Установить: Window → Package Manager → Add from git URL:
//     https://github.com/endel/NativeWebSocket.git#upm
//
// Настройка в Unity:
//   1. Прикрепить этот скрипт к пустому GameObject "AvatarSender"
//   2. В Inspector задать Server IP = IP-адрес вашего ПК в сети Wi-Fi
//   3. Назначить ссылки OVRCameraRig, правый и левый OVRHand/Controller
//
// Автор: Андреюк М.О., Жук Б.Д. (Группа ИИ-25, БрГТУ, 2026)

using System;
using System.Text;
using System.Collections;
using UnityEngine;
using NativeWebSocket;

public class VRDataSender : MonoBehaviour
{
    // ── Inspector-поля ────────────────────────────────────────────────────
    [Header("Сеть")]
    [Tooltip("IP-адрес ПК с запущенным g1_avatar_controller.py")]
    public string serverIP   = "192.168.1.100";
    public int    serverPort = 8765;

    [Header("VR-объекты")]
    public Transform headTransform;         // OVRCameraRig → CenterEyeAnchor
    public Transform rightHandTransform;    // OVRCameraRig → RightHandAnchor
    public Transform leftHandTransform;     // OVRCameraRig → LeftHandAnchor

    [Header("Параметры отправки")]
    [Tooltip("Частота отправки пакетов (Гц). 0 = каждый кадр")]
    public float sendHz = 72f;

    // ── Приватные поля ────────────────────────────────────────────────────
    private WebSocket _ws;
    private float     _sendInterval;
    private float     _sendTimer;
    private bool      _isConnected;

    // OVR-контроллеры
    private OVRInput.Controller _rightCtrl = OVRInput.Controller.RTouch;
    private OVRInput.Controller _leftCtrl  = OVRInput.Controller.LTouch;

    // ── Lifecycle ─────────────────────────────────────────────────────────

    private async void Start()
    {
        _sendInterval = sendHz > 0 ? 1f / sendHz : 0f;
        await ConnectToServer();
    }

    private void Update()
    {
        if (_ws == null) return;

        // Обработать входящие сообщения (NativeWebSocket требует это)
#if !UNITY_WEBGL || UNITY_EDITOR
        _ws.DispatchMessageQueue();
#endif

        // Таймер отправки
        if (sendHz > 0)
        {
            _sendTimer += Time.deltaTime;
            if (_sendTimer < _sendInterval) return;
            _sendTimer = 0f;
        }

        if (_isConnected)
            SendVRFrame();
    }

    private async void OnApplicationQuit()
    {
        if (_ws != null)
            await _ws.Close();
    }

    // ── Подключение ───────────────────────────────────────────────────────

    private async System.Threading.Tasks.Task ConnectToServer()
    {
        string url = $"ws://{serverIP}:{serverPort}";
        Debug.Log($"[VRDataSender] Подключение к {url}...");

        _ws = new WebSocket(url);

        _ws.OnOpen += () =>
        {
            _isConnected = true;
            Debug.Log("[VRDataSender] Соединение установлено.");
        };

        _ws.OnClose += (e) =>
        {
            _isConnected = false;
            Debug.LogWarning($"[VRDataSender] Соединение закрыто: {e}");
            // Переподключение через 2 секунды
            Invoke(nameof(Reconnect), 2f);
        };

        _ws.OnError += (e) =>
        {
            Debug.LogError($"[VRDataSender] Ошибка: {e}");
        };

        await _ws.Connect();
    }

    private async void Reconnect()
    {
        Debug.Log("[VRDataSender] Переподключение...");
        await ConnectToServer();
    }

    // ── Формирование и отправка пакета ────────────────────────────────────

    private void SendVRFrame()
    {
        // Аварийная остановка — кнопка B правого контроллера
        bool emergency = OVRInput.GetDown(OVRInput.Button.Two, _rightCtrl);

        // Голова
        Vector3    headPos  = headTransform  ? headTransform.position  : Vector3.zero;
        Quaternion headQuat = headTransform  ? headTransform.rotation  : Quaternion.identity;

        // Правая рука
        Vector3    rPos  = rightHandTransform ? rightHandTransform.position : Vector3.zero;
        Quaternion rQuat = rightHandTransform ? rightHandTransform.rotation : Quaternion.identity;
        float      rGrip = OVRInput.Get(OVRInput.Axis1D.PrimaryHandTrigger, _rightCtrl);

        // Левая рука
        Vector3    lPos  = leftHandTransform ? leftHandTransform.position : Vector3.zero;
        Quaternion lQuat = leftHandTransform ? leftHandTransform.rotation : Quaternion.identity;
        float      lGrip = OVRInput.Get(OVRInput.Axis1D.PrimaryHandTrigger, _leftCtrl);

        // Стики
        Vector2 stickR = OVRInput.Get(OVRInput.Axis2D.PrimaryThumbstick, _rightCtrl);
        Vector2 stickL = OVRInput.Get(OVRInput.Axis2D.PrimaryThumbstick, _leftCtrl);

        // Сборка JSON вручную (без лишних зависимостей)
        string json = BuildJson(
            headPos, headQuat,
            rPos, rQuat, rGrip,
            lPos, lQuat, lGrip,
            stickR, stickL,
            emergency
        );

        _ws.SendText(json);
    }

    // ── JSON-сборщик ──────────────────────────────────────────────────────

    private static string V3(Vector3 v) =>
        $"[{v.x:F4},{v.y:F4},{v.z:F4}]";

    private static string Q(Quaternion q) =>
        $"[{q.x:F4},{q.y:F4},{q.z:F4},{q.w:F4}]";

    private static string V2(Vector2 v) =>
        $"[{v.x:F4},{v.y:F4}]";

    private string BuildJson(
        Vector3 hPos, Quaternion hQuat,
        Vector3 rPos, Quaternion rQuat, float rGrip,
        Vector3 lPos, Quaternion lQuat, float lGrip,
        Vector2 stickR, Vector2 stickL,
        bool emergency)
    {
        var sb = new StringBuilder();
        sb.Append("{");

        // Голова
        sb.Append($"\"head\":{{\"pos\":{V3(hPos)},\"quat\":{Q(hQuat)}}},");

        // Правая рука
        sb.Append($"\"right\":{{\"pos\":{V3(rPos)},\"quat\":{Q(rQuat)},\"grip\":{rGrip:F3}}},");

        // Левая рука
        sb.Append($"\"left\":{{\"pos\":{V3(lPos)},\"quat\":{Q(lQuat)},\"grip\":{lGrip:F3}}},");

        // Стики
        sb.Append($"\"stick_r\":{V2(stickR)},");
        sb.Append($"\"stick_l\":{V2(stickL)},");

        // Аварийная кнопка
        sb.Append($"\"emergency\":{(emergency ? "true" : "false")}");

        sb.Append("}");
        return sb.ToString();
    }

    // ── Отладочный GUI (только в редакторе) ──────────────────────────────
#if UNITY_EDITOR
    private void OnGUI()
    {
        GUIStyle style = new GUIStyle(GUI.skin.label) { fontSize = 14 };
        GUI.color = _isConnected ? Color.green : Color.red;
        GUI.Label(new Rect(10, 10, 400, 25),
            _isConnected
                ? $"VRDataSender: ПОДКЛЮЧЁН ({serverIP}:{serverPort})"
                : "VRDataSender: НЕ ПОДКЛЮЧЁН",
            style);
    }
#endif
}
