using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System.Diagnostics;
using System.Threading;
using UnityEditor;
using System.Globalization;

// © 2024 Johannes Lindner <johannes.lindner@tum.de>

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

    private Thread sumoThread { get; set; }
    private Process process { get; set; }

    void Start()
    {
        if (startSumoOnStart)
        {
            StartSumoThread();
        } 
    }


    public void StartSumoThread()
    {
        // Initialize Thread
        ThreadStart threadStart = new ThreadStart(StartSumo);
        sumoThread = new Thread(threadStart);
        sumoThread.Start();
    }


    void StartSumo()
    {
        string PYTHON_SCRIPT_PATH = "Assets/Sumonity/SumoTraCI/socketServer.py --dt " + dt.ToString(new CultureInfo("en-US"));
        string venvPath = "Assets/Sumonity/SumoTraCI/venv/Scripts/activate.bat";
        string unityWorkspacePath = System.IO.Path.GetDirectoryName(Application.dataPath);

        // Combine the Unity workspace path with the venv and Python script paths
        string fullVenvPath = System.IO.Path.Combine(unityWorkspacePath, venvPath);
        string fullPythonScriptPath = System.IO.Path.Combine(unityWorkspacePath, PYTHON_SCRIPT_PATH);

        // Define Process
        ProcessStartInfo startInfo = new ProcessStartInfo();
        startInfo.FileName = "cmd.exe"; // Use cmd.exe to execute the command
        startInfo.WorkingDirectory = unityWorkspacePath; // Set the working directory
        startInfo.RedirectStandardOutput = true;
        startInfo.RedirectStandardError = true;
        startInfo.CreateNoWindow = true;
        startInfo.UseShellExecute = false;

        // Use 'call' to activate the venv and then run your Python script
        startInfo.Arguments = $"/c \"call {fullVenvPath} && python {fullPythonScriptPath}\"";

        // Start Process
        process = new Process();
        process.StartInfo = startInfo;
        process.Start();
        ProcessID = process.Id.ToString();

        int errorCount = 0;
        int maxErrorsToLog = 5;
        
        bool activateDebug = false;

        while (!process.HasExited)
        {
            string output = process.StandardOutput.ReadLine();
            if (!string.IsNullOrEmpty(output))
            {
                try 
                {
                    // Log the output
                    if (activateDebug)
                    {
                        UnityEngine.Debug.Log(output);
                    }
                }
                catch (System.Exception ex)
                {
                    // Only log a limited number of errors to avoid spam
                    if (errorCount < maxErrorsToLog)
                    {
                        UnityEngine.Debug.LogError($"Error processing output: {ex.Message}");
                        errorCount++;
                    }
                    else if (errorCount == maxErrorsToLog)
                    {
                        UnityEngine.Debug.LogWarning("Suppressing further similar errors to avoid spam");
                        errorCount++;
                    }
                }
            }
        }

        error = process.StandardError.ReadToEnd();
        if (!string.IsNullOrEmpty(error))
        {
            UnityEngine.Debug.LogError(error);
        }
    }


    void OnApplicationQuit()
    {
        // 1. Kill Process
        process.Kill();
        // 2. Abort Thread
        sumoThread.Abort();


        // 3. Close SUMO or SUMO-GUI
        UnityEngine.Debug.Log("Closing SUMO or SUMO-GUI");
        var processes = Process.GetProcesses();
        foreach (var process in processes)
        {
            if (process.ProcessName.ToLower().Contains("sumo-gui"))
            {
                process.Kill();
                UnityEngine.Debug.Log("Closed SUMO-GUI process with ID: " + process.Id.ToString());
            }
        }

    }
}
