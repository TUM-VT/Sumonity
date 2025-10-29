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
            // Clean up any existing processes before starting
            CleanupExistingProcesses();
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
        CleanupExistingProcesses();
    }

    private void CleanupExistingProcesses()
    {
        UnityEngine.Debug.Log("Cleaning up processes...");

        // 1. Kill the main process if it exists and hasn't exited
        if (process != null)
        {
            try
            {
                if (!process.HasExited)
                {
                    UnityEngine.Debug.Log($"Killing main process with ID: {process.Id}");
                    process.Kill();
                    process.WaitForExit(3000); // Wait up to 3 seconds for graceful exit
                }
            }
            catch (System.Exception ex)
            {
                UnityEngine.Debug.LogWarning($"Error killing main process: {ex.Message}");
            }
            finally
            {
                try
                {
                    process.Dispose();
                }
                catch { }
                process = null;
            }
        }

        // 2. Abort the thread if it exists
        if (sumoThread != null && sumoThread.IsAlive)
        {
            try
            {
                UnityEngine.Debug.Log("Aborting SUMO thread");
                sumoThread.Abort();
                sumoThread.Join(2000); // Wait up to 2 seconds for thread to abort
            }
            catch (System.Exception ex)
            {
                UnityEngine.Debug.LogWarning($"Error aborting thread: {ex.Message}");
            }
            finally
            {
                sumoThread = null;
            }
        }

        // 3. Close SUMO or SUMO-GUI processes
        UnityEngine.Debug.Log("Closing SUMO or SUMO-GUI processes");
        try
        {
            var processes = Process.GetProcesses();
            foreach (var proc in processes)
            {
                try
                {
                    // Check if process hasn't exited before accessing properties
                    if (!proc.HasExited)
                    {
                        string processName = proc.ProcessName.ToLower();
                        if (processName.Contains("sumo-gui") || processName.Contains("sumo"))
                        {
                            UnityEngine.Debug.Log($"Closing {proc.ProcessName} process with ID: {proc.Id}");
                            proc.Kill();
                            proc.WaitForExit(2000);
                        }
                    }
                }
                catch (System.Exception ex)
                {
                    // Skip processes we can't access (access denied, etc.)
                    UnityEngine.Debug.LogWarning($"Could not check/kill process: {ex.Message}");
                }
                finally
                {
                    proc.Dispose();
                }
            }
        }
        catch (System.Exception ex)
        {
            UnityEngine.Debug.LogError($"Error during process cleanup: {ex.Message}");
        }

        // 4. Also kill any Python processes running socketServer.py
        try
        {
            var pythonProcesses = Process.GetProcesses();
            foreach (var proc in pythonProcesses)
            {
                try
                {
                    if (!proc.HasExited && proc.ProcessName.ToLower().Contains("python"))
                    {
                        // Try to get command line to see if it's our script
                        // Note: This requires additional permissions on some systems
                        try
                        {
                            string cmdLine = proc.MainModule?.FileName ?? "";
                            if (!string.IsNullOrEmpty(cmdLine))
                            {
                                UnityEngine.Debug.Log($"Found Python process: {proc.Id}");
                                // Kill it to be safe - you might want to be more selective here
                                proc.Kill();
                                proc.WaitForExit(2000);
                            }
                        }
                        catch
                        {
                            // Can't access command line, skip this process
                        }
                    }
                }
                catch (System.Exception)
                {
                    // Skip processes we can't access
                }
                finally
                {
                    proc.Dispose();
                }
            }
        }
        catch (System.Exception ex)
        {
            UnityEngine.Debug.LogWarning($"Error cleaning up Python processes: {ex.Message}");
        }

        UnityEngine.Debug.Log("Process cleanup completed");
    }
}
