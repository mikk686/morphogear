using RosMessageTypes.Std;
using Unity.Robotics.ROSTCPConnector;
using UnityEngine;
using UnityEngine.InputSystem;

public class ROScmd : MonoBehaviour
{
    private ROSConnection _ros;
    public string TopicName = "/morphogear_sudo_cmd";
    public int angle = 0; // Set a default value or adjust in Inspector

    void Start()
    {
        // FIX 1: Initialize the ROS connection instance before using it
        _ros = ROSConnection.GetOrCreateInstance();
        _ros.RegisterPublisher<StringMsg>(TopicName);
    }

    void Update()
    {
        SendCommand();
    }

    // Перечисление для контроля главного направления
    private enum MainDirection { None, Forward, Backward, Left, Right }
    private MainDirection _currentMain = MainDirection.None;
    void SendCommand()
    {
        var kb = Keyboard.current;
        if (kb == null) return;

        // 1. ОПРЕДЕЛЯЕМ ГЛАВНУЮ КНОПКУ (какая была нажата первой)
        if (_currentMain == MainDirection.None)
        {
            if (kb.upArrowKey.wasPressedThisFrame) _currentMain = MainDirection.Forward;
            else if (kb.downArrowKey.wasPressedThisFrame) _currentMain = MainDirection.Backward;
            else if (kb.leftArrowKey.wasPressedThisFrame) _currentMain = MainDirection.Left;
            else if (kb.rightArrowKey.wasPressedThisFrame) _currentMain = MainDirection.Right;
        }

        // 2. СБРАСЫВАЕМ ГЛАВНУЮ КНОПКУ, ЕСЛИ ЕЁ ОТПУСТИЛИ
        if (_currentMain == MainDirection.Forward && !kb.upArrowKey.isPressed) _currentMain = MainDirection.None;
        if (_currentMain == MainDirection.Backward && !kb.downArrowKey.isPressed) _currentMain = MainDirection.None;
        if (_currentMain == MainDirection.Left && !kb.leftArrowKey.isPressed) _currentMain = MainDirection.None;
        if (_currentMain == MainDirection.Right && !kb.rightArrowKey.isPressed) _currentMain = MainDirection.None;

        // Если ничего не зажато, выходим
        if (_currentMain == MainDirection.None) return;

        StringMsg msg = new StringMsg();
        bool shouldPublish = true;

        // 3.ОБРАБАТЫВАЕМ ЛОГИКУ НА ОСНОВЕ «КТО ГЛАВНЫЙ»
        switch (_currentMain)
        {
            case MainDirection.Forward:
                // Главный вперед, подкручиваем боковыми
                if (kb.leftArrowKey.isPressed) msg.data = "forwardAuto_15";
                else if (kb.rightArrowKey.isPressed) msg.data = "forwardAuto_-15";
                else msg.data = $"forwardAuto_{angle}";
                break;

            case MainDirection.Backward:
                // Главный назад, подкручиваем боковыми
                if (kb.rightArrowKey.isPressed) msg.data = "backwardAuto_15";
                else if (kb.leftArrowKey.isPressed) msg.data = "backwardAuto_-15";
                else msg.data = $"backwardAuto_{angle}";
                break;

            case MainDirection.Left:
                // Главный влево, подкручиваем продольными (вперед/назад)
                if (kb.upArrowKey.isPressed) msg.data = "leftAuto_-15";
                else if (kb.downArrowKey.isPressed) msg.data = "leftAuto_15";
                else msg.data = $"leftAuto_{angle}";
                break;

            case MainDirection.Right:
                // Главный вправо, подкручиваем продольными (вперед/назад)
                if (kb.upArrowKey.isPressed) msg.data = "rightAuto_15";
                else if (kb.downArrowKey.isPressed) msg.data = "rightAuto_-15";
                else msg.data = $"rightAuto_{angle}";
                break;
        }

        if (shouldPublish)
        {
            _ros.Publish(TopicName, msg);
        }
    }
    //void SendCommand()
    //{
    //    var kb = Keyboard.current;
    //    if (kb == null) return; // Safety check in case no keyboard is connected

    //    // FIX 2: Declare the message OUTSIDE the ifs so it is accessible to Publish()
    //    StringMsg msg = new StringMsg();
    //    bool shouldPublish = false;

    //    // FIX 3: Replaced Python syntax with correct C# syntax
    //    if (kb.anyKey.isPressed)
    //    {
    //        if (kb.upArrowKey.isPressed)
    //        {
    //            if (kb.leftArrowKey.isPressed) msg.data = "forwardAuto_" + (15).ToString();
    //            else if (kb.rightArrowKey.isPressed) msg.data = "forwardAuto_" + (-15).ToString();
    //            else msg.data = "forwardAuto_" + angle.ToString();

    //            shouldPublish = true;
    //        }
    //        else if (kb.downArrowKey.isPressed) // Assuming 'back' means down arrow
    //        {
    //            if (kb.rightArrowKey.isPressed) msg.data = "backwardAuto_" + (15).ToString();
    //            else if (kb.leftArrowKey.isPressed) msg.data = "backwardAuto_" + (-15).ToString();
    //            else
    //                msg.data = "backwardAuto_" + angle.ToString();
    //            shouldPublish = true;
    //        }
    //        else if (kb.leftArrowKey.isPressed)
    //        {
    //            msg.data = "leftAuto_" + angle.ToString();
    //            shouldPublish = true;
    //        }
    //        else if (kb.rightArrowKey.isPressed)
    //        {
    //            msg.data = "rightAuto_" + angle.ToString();
    //            shouldPublish = true;
    //        }
    //    }

    //    // FIX 4: Only publish if a key was actually pressed and 'msg.data' was set
    //    if (shouldPublish)
    //    {
    //        _ros.Publish(TopicName, msg);
    //    }
    //}
}
