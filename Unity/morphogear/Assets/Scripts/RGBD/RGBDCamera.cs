using System;
using Unity.Collections;
using UnityEngine;
using UnityEngine.Rendering;
using static UnityEngine.UI.ContentSizeFitter;

public class RGBDCamera : MonoBehaviour
{
    [Header("Sensor Settings")]
    [SerializeField, Min(0.1f)] private float _frequency = 10.0f; // Default 10 Hz
    [SerializeField] private Vector2Int _resolution = new Vector2Int(640, 480);
    //[SerializeField] private float _fov = 60.0f;
    [SerializeField] private float _minRange = 0.1f;
    [SerializeField] private float _maxRange = 10.0f;
    [SerializeField] private float _depthErrorScale = 0.005f;

    [Header("GPU Compute")]
    [SerializeField] private ComputeShader _pointCloudCompute;

    public NativeArray<uint> Depth16Data => _depth16Array;
    public Action OnSensorUpdated;

    private UnityEngine.Camera _depthCamera;
    private RenderTexture _depthRt;

    private GraphicsBuffer _depth16Buffer;
    private NativeArray<uint> _depth16Array;

    private int _pointsNum;
    private int _kernelID;
    private Vector3Int _dispatchGroups;
    private bool _isRunning = false;


    public enum FovMode { Vertical, Horizontal }

    [SerializeField] private FovMode _fovMode = FovMode.Horizontal;
    [SerializeField, Range(1f, 179f)] private float _fov = 87f;

    private void Awake()
    {
        _pointsNum = _resolution.x * _resolution.y;

        _depthRt = new RenderTexture(_resolution.x, _resolution.y, 32, RenderTextureFormat.Depth);

        GameObject depthCamObj = new GameObject("DepthCamera");
        depthCamObj.transform.SetParent(transform, false);
        _depthCamera = depthCamObj.AddComponent<UnityEngine.Camera>();

        //_depthCamera.fieldOfView = _fov;

        float aspect = (float)_resolution.x / _resolution.y;
        _depthCamera.aspect = aspect;

        _depthCamera.fieldOfView = _fovMode == FovMode.Horizontal
            ? 2f * Mathf.Atan(
                Mathf.Tan(_fov * Mathf.Deg2Rad * 0.5f) / aspect
              ) * Mathf.Rad2Deg
            : _fov;



        _depthCamera.nearClipPlane = _minRange;
        _depthCamera.farClipPlane = _maxRange;
        _depthCamera.enabled = false;
        _depthCamera.clearFlags = CameraClearFlags.SolidColor;
        _depthCamera.backgroundColor = Color.black;
        _depthCamera.targetTexture = _depthRt;

        InitializeCompute();
    }

    private void InitializeCompute()
    {
        _depth16Array = new NativeArray<uint>(_pointsNum, Allocator.Persistent);
        _depth16Buffer = new GraphicsBuffer(GraphicsBuffer.Target.Structured, _pointsNum, 4);

        _kernelID = _pointCloudCompute.FindKernel("CSMain");
        _dispatchGroups = new Vector3Int(Mathf.CeilToInt(_resolution.x / 8f), Mathf.CeilToInt(_resolution.y / 8f), 1);
        _pointCloudCompute.SetFloats("resolution", new float[] { _resolution.x, _resolution.y });
    }

    //private void OnEnable()
    //{
    //    _isRunning = true;
    //    //_ = StartSensorSequenceAsync();
    //}



    //bool flag = true;
    //async void LateUpdate()
    //{
    //    if (flag)
    //    {
    //        flag = !flag;
    //        return;
    //    }
    //    else
    //    {
    //        CaptureAndComputeAsync();
    //        flag = !flag;
    //    }
    //}

    private bool _captureInFlight;
    private float _nextCaptureTime;

    public Vector2Int Resolution => _resolution;

    private void OnEnable()
    {
        _isRunning = true;
        _nextCaptureTime = Time.realtimeSinceStartup;
    }

    private void LateUpdate()
    {
        if (!_isRunning || _captureInFlight)
            return;

        float now = Time.realtimeSinceStartup;
        if (now < _nextCaptureTime)
            return;

        // Schedule from now: missed captures are dropped, not caught up later.
        _nextCaptureTime = now + 1f / Mathf.Max(0.1f, _frequency);
        _ = CaptureGuardedAsync();
    }

    private async Awaitable CaptureGuardedAsync()
    {
        _captureInFlight = true;

        try
        {
            await CaptureAndComputeAsync();
        }
        catch (Exception exception)
        {
            if (_isRunning)
                Debug.LogWarning(
                    $"Depth capture failed: {exception.Message}", this);
        }
        finally
        {
            _captureInFlight = false;
        }
    }

    private async Awaitable CaptureAndComputeAsync()
    {
        if (!_isRunning || _depthCamera == null) return;

        _depthCamera.Render();

        Matrix4x4 gpuProjMatrix = GL.GetGPUProjectionMatrix(_depthCamera.projectionMatrix, false);

        _pointCloudCompute.SetMatrix("_InvProjMatrix", gpuProjMatrix.inverse);
        _pointCloudCompute.SetTexture(_kernelID, "DepthTexture", _depthRt);
        _pointCloudCompute.SetBuffer(_kernelID, "Depth16Buffer", _depth16Buffer);

        _pointCloudCompute.SetFloat("time", Time.time);
        _pointCloudCompute.SetFloat("noiseSigma", _depthErrorScale);
        _pointCloudCompute.SetFloat("maxRange", _maxRange);

        _pointCloudCompute.Dispatch(_kernelID, _dispatchGroups.x, _dispatchGroups.y, _dispatchGroups.z);

        var tcs = new AwaitableCompletionSource();
        AsyncGPUReadback.RequestIntoNativeArray(ref _depth16Array, _depth16Buffer, request =>
        {
            if (!request.hasError) tcs.SetResult();
            else tcs.SetCanceled();
        });

        await tcs.Awaitable;

        // SAFETY CHECK: Ensure we didn't exit Play Mode while waiting for the GPU!
        if (!_isRunning) return;

        OnSensorUpdated?.Invoke();
    }

    // Moved from OnDestroy to OnDisable for safer Unity Editor behavior
    private void OnDisable()
    {
        _isRunning = false;

        // Force GPU to finish before we delete the memory it is trying to write to
        AsyncGPUReadback.WaitAllRequests();
    }

    private void OnDestroy()
    {
        _isRunning = false;
        if (_depthRt != null) { _depthRt.Release(); _depthRt = null; }
        if (_depth16Buffer != null) { _depth16Buffer.Release(); _depth16Buffer = null; }
        if (_depth16Array.IsCreated) { _depth16Array.Dispose(); }
    }
}