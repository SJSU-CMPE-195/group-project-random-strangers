using UnityEngine;

public class OptiTrackPosePrinter : MonoBehaviour
{
    [Header("Print rate (Hz)")]
    public float printHz = 10f;

    [Header("Optional: also show in Scene as a label")]
    public bool drawLabel = true;

    float _nextPrintTime;

    void Update()
    {
        if (printHz <= 0f) return;

        if (Time.time >= _nextPrintTime)
        {
            _nextPrintTime = Time.time + (1f / printHz);

            Vector3 p = transform.position;
            Quaternion q = transform.rotation;

            // Unity uses meters for position; rotations are quaternions.
            Debug.Log($"[OptiTrack] {name}  pos(m)=({p.x:F3}, {p.y:F3}, {p.z:F3})  " +
                      $"rot(q)=({q.x:F4}, {q.y:F4}, {q.z:F4}, {q.w:F4})");
        }
    }

    void OnGUI()
    {
        if (!drawLabel) return;

        Vector3 p = transform.position;
        Quaternion q = transform.rotation;

        GUI.Label(
            new Rect(10, 10, 900, 60),
            $"{name}\npos(m): {p.x:F3}, {p.y:F3}, {p.z:F3}\nrot(q): {q.x:F4}, {q.y:F4}, {q.z:F4}, {q.w:F4}"
        );
    }
}