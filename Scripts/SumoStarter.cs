using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System.Diagnostics;
using System.Threading;
using UnityEditor;

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
        string PYTHON_SCRIPT_PATH = "Assets/Sumonity/Sumo/socketServer.py";
        string venvPath = "Assets/Sumonity/Sumo/venv/Scripts/activate.bat";
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

        UnityEngine.Debug.Log("logtest");
        // Use 'call' to activate the venv and then run your Python script
        startInfo.Arguments = $"/c \"call {fullVenvPath} && python {fullPythonScriptPath}\"";
        UnityEngine.Debug.Log("passes");

        // Start Process
        process = new Process();
        process.StartInfo = startInfo;
        process.Start();
        ProcessID = process.Id.ToString();

        while (!process.HasExited)
        {
            string output = process.StandardOutput.ReadLine();
            if (output != null)
            {
                UnityEngine.Debug.Log(output);
            }
        }

        error = process.StandardError.ReadToEnd();
        if (error != null)
        {
            UnityEngine.Debug.Log(error);
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
