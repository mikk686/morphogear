// =============================================================================
//  UnityKeyboardSender.cs
//
//  Wire format (14 comma-separated fields, 2 chars each):
//      w,a,s,d,q,e,z,c,x,space,digit1,digit2,digit3,digit4
//      field[0] = '1' while held   [1] = '1' on press frame
// =============================================================================

using UnityEngine;
using UnityEngine.InputSystem;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Std;

public class UnityKeyboardSender : MonoBehaviour
{
    [Header("ROS")]
    public string topicName = "/keyboard_state";
    public bool streaming = true;
    public bool sendOnlyOnChange = false;

    private ROSConnection _ros;
    private string _lastPayload = "";

    void Start()
    {
        _ros = ROSConnection.GetOrCreateInstance();
        _ros.RegisterPublisher<StringMsg>(topicName);
    }

    void Update()
    {
        if (!streaming) return;
        var keyboard = Keyboard.current;
        if (keyboard == null) return;

        string payload = BuildPayload(keyboard);
        if (sendOnlyOnChange && payload == _lastPayload) return;

        _lastPayload = payload;
        _ros.Publish(topicName, new StringMsg(payload));
    }

    private string BuildPayload(Keyboard kb)
    {
        bool w = kb.wKey.isPressed; bool wDown = kb.wKey.wasPressedThisFrame;
        bool a = kb.aKey.isPressed; bool aDown = kb.aKey.wasPressedThisFrame;
        bool s = kb.sKey.isPressed; bool sDown = kb.sKey.wasPressedThisFrame;
        bool d = kb.dKey.isPressed; bool dDown = kb.dKey.wasPressedThisFrame;
        bool q = kb.qKey.isPressed; bool qDown = kb.qKey.wasPressedThisFrame;
        bool e = kb.eKey.isPressed; bool eDown = kb.eKey.wasPressedThisFrame;
        bool z = kb.zKey.isPressed; bool zDown = kb.zKey.wasPressedThisFrame;
        bool c = kb.cKey.isPressed; bool cDown = kb.cKey.wasPressedThisFrame;
        bool x = kb.xKey.isPressed; bool xDown = kb.xKey.wasPressedThisFrame;
        bool space = kb.spaceKey.isPressed; bool spaceDown = kb.spaceKey.wasPressedThisFrame;

        bool d1 = kb.digit1Key.wasPressedThisFrame;
        bool d2 = kb.digit2Key.wasPressedThisFrame;
        bool d3 = kb.digit3Key.wasPressedThisFrame;
        bool d4 = kb.digit4Key.wasPressedThisFrame;

        return string.Join(",", new[]
        {
            Field(w, wDown), Field(a, aDown), Field(s, sDown), Field(d, dDown),
            Field(q, qDown), Field(e, eDown), Field(z, zDown), Field(c, cDown), Field(x, xDown),
            Field(space, spaceDown),
            Field(false, d1), Field(false, d2), Field(false, d3), Field(false, d4)
        });
    }

    private static string Field(bool held, bool pressedThisFrame)
        => (held ? "1" : "0") + (pressedThisFrame ? "1" : "0");

    public void SetManualControl(bool manual)
        => _ros.Publish("morphogear_sudo_manual", new BoolMsg(manual));

    public void SendStringCommand(string cmd)
        => _ros.Publish("morphogear_sudo_cmd", new StringMsg(cmd));
}