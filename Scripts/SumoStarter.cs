using System.Collections;
using UnityEngine;
using System.Diagnostics;
using System.Threading;
using System.Globalization;
using System.IO;

public class SumoStarter : MonoBehaviour
{
    [Header("Control")]
    [SerializeField]
    bool startSumoOnStart = true;

    [Header("Debug")]
    [SerializeField]
    public string ProcessID;
    public string error = null;
    public float dt = 0.1f;

    private Thread sumoThread = null;
    private Process process = null;

    void Start()
    {
        if (startSumoOnStart)
        {
            StartSumoThread();
        }
    }

    public void StartSumoThread()
    {
        // Don't start if our process is running
        if (process != null && !process.HasExited)
        {
            UnityEngine.Debug.LogWarning("socketServer.py already running (PID=" + process.Id + ")!");
            return;
        }

        // If a previous process is gone, clean up reference
        if (process != null && process.HasExited)
        {
            process.Dispose();
            process = null;
        }

        ThreadStart threadStart = new ThreadStart(StartSumo);
        sumoThread = new Thread(threadStart);
        sumoThread.Start();
    }

    private void StartSumo()
    {
        string scriptNameWithArgs = $"Assets/Sumonity/SumoTraCI/socketServer.py --dt {dt.ToString(new CultureInfo("en-US"))}";
        string venvPath = "Assets/Sumonity/SumoTraCI/venv/Scripts/activate.bat";
        string unityWorkspacePath = Path.GetDirectoryName(Application.dataPath);

        string fullVenvPath = Path.Combine(unityWorkspacePath, venvPath);
        string fullPythonScriptPath = Path.Combine(unityWorkspacePath, scriptNameWithArgs);

        ProcessStartInfo startInfo = new ProcessStartInfo();
        startInfo.FileName = "cmd.exe";
        startInfo.WorkingDirectory = unityWorkspacePath;
        startInfo.RedirectStandardOutput = true;
        startInfo.RedirectStandardError = true;
        startInfo.CreateNoWindow = true;
        startInfo.UseShellExecute = false;
        startInfo.Arguments = $"/c \"call {fullVenvPath} && python {fullPythonScriptPath}\"";

        process = new Process();
        process.StartInfo = startInfo;

        try
        {
            process.Start();
            ProcessID = process.Id.ToString();

            // Read output async to avoid blocking
            var outputThread = new Thread(() =>
            {
                string line;
                while ((line = process.StandardOutput.ReadLine()) != null)
                {
                    UnityEngine.Debug.Log(line);
                }
            });
            outputThread.Start();

            string err;
            while ((err = process.StandardError.ReadLine()) != null)
            {
                if (!string.IsNullOrEmpty(err))
                {
                    error += err + "\n";
                    UnityEngine.Debug.LogError(err);
                }
            }

            process.WaitForExit();
            outputThread.Join();
        }
        catch (System.Exception ex)
        {
            UnityEngine.Debug.LogError($"Failed to start or monitor Python process: {ex}");
        }
    }

    void OnApplicationQuit()
    {
        // Only terminate our tracked process
        if (process != null && !process.HasExited)
        {
            try
            {
                process.CloseMainWindow(); // May not do anything for CLI process
            }
            catch { }
            try
            {
                process.WaitForExit(2000);
            }
            catch { }

            if (!process.HasExited)
            {
                try
                {
                    process.Kill();
                    UnityEngine.Debug.LogWarning($"Force-killed Python process with PID {process.Id}");
                }
                catch { }
            }
        }

        // Wait for thread to finish
        if (sumoThread != null && sumoThread.IsAlive)
        {
            sumoThread.Join(2000);
        }

        // Optionally, close any SUMO-GUI processes by name
        UnityEngine.Debug.Log("Checking for open SUMO-GUI processes...");
        var processes = Process.GetProcesses();
        foreach (var proc in processes)
        {
            try
            {
                if (proc.ProcessName.ToLower().Contains("sumo-gui"))
                {
                    proc.Kill();
                    UnityEngine.Debug.Log("Closed SUMO-GUI process with ID: " + proc.Id.ToString());
                }
            }
            catch { }
        }
    }
}
