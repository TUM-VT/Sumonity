using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System.Diagnostics;
using System.Threading;
using UnityEditor;

// � 2024 Johannes Lindner <johannes.lindner@tum.de>

public class SumoStarter : MonoBehaviour
{
    [Header("Control")]
    [SerializeField]
    bool startSumoOnStart = true;

    [Header("Debug")]
    [SerializeField]
    private string ProcessID;

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
        string PYTHON_SCRIPT_PATH = "Assets/SumoBridge/Sumo/socketServer.py";
        string venvPath = "Assets/SumoBridge/Sumo/venv/bin/activate";
        string unityWorkspacePath = System.IO.Path.GetDirectoryName(Application.dataPath);

        // Combine the Unity workspace path with the venv and Python script paths
        string fullVenvPath = System.IO.Path.Combine(unityWorkspacePath, venvPath);
        string fullPythonScriptPath = System.IO.Path.Combine(unityWorkspacePath, PYTHON_SCRIPT_PATH);

        // Define Process
        ProcessStartInfo startInfo = new ProcessStartInfo();
        startInfo.FileName = "/bin/bash"; // Use bash to execute the command
        startInfo.WorkingDirectory = unityWorkspacePath; // Set the working directory
        startInfo.RedirectStandardOutput = true;
        startInfo.RedirectStandardError = true;
        startInfo.CreateNoWindow = true;
        startInfo.UseShellExecute = false;

        // Use 'source' to activate the venv and then run your Python script
        startInfo.Arguments = $"-c \"source {fullVenvPath} && python {fullPythonScriptPath}\"";

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

        string error = process.StandardError.ReadToEnd();
        if (error != null)
        {
            UnityEngine.Debug.Log(error);
        }
    }


    void Close()
    {
        // 1. Kill Process
        process.Kill();

        // 2. Abort Thread
        sumoThread.Abort();

        // 3. Close SUMO or SUMO-GUI
        var sumo_gui_processes = Process.GetProcessesByName("sumo-gui");
        if (sumo_gui_processes != null && sumo_gui_processes.Length > 0)
        { 
            foreach (var process in sumo_gui_processes) { process.Kill(); }
            UnityEngine.Debug.Log(sumo_gui_processes.Length.ToString() + " SUMO-GUI Process closed!");
        }

        var sumo_processes = Process.GetProcessesByName("sumo");
        if (sumo_processes != null && sumo_processes.Length>0)
        {
            foreach (var process in sumo_processes) { process.Kill(); }
            UnityEngine.Debug.Log(sumo_processes.Length.ToString() + " SUMO Process closed!");
        }
    }


    void OnApplicationQuit()
    {
        Close(); 
    }
    
    
}
