# attach_controller.ps1
#
# This script finds the "Xbox Wireless Adapter for Windows" in the list of USB
# devices and automatically attaches it to WSL using usbipd.
#
# IMPORTANT: This script must be run in a PowerShell window with
# Administrator privileges.

# --- Configuration ---
$deviceName = "Xbox One Controller"

# --- Script Body ---

# 1. Check for Administrator privileges
if (-NOT ([Security.Principal.WindowsPrincipal][Security.Principal.WindowsIdentity]::GetCurrent()).IsInRole([Security.Principal.WindowsBuiltInRole] "Administrator")) {
    Write-Warning "This script needs to be run as an Administrator."
    Write-Host "Please re-launch PowerShell as an Administrator and run the script again."
    # Pause to allow the user to read the message before the window closes.
    if ($Host.Name -eq "ConsoleHost") {
        Read-Host -Prompt "Press Enter to exit"
    }
    exit
}

Write-Host "Searching for USB devices..."

try {
    # 2. Get the list of all USB devices from usbipd
    $deviceList = usbipd list

    # 3. Find the line containing the target device
    $controllerLine = $deviceList | Where-Object { $_ -match $deviceName }

    if ($null -ne $controllerLine) {
        Write-Host "Found device: $controllerLine"

        # 4. Extract the BUSID from the line (it's the first word)
        $busId = ($controllerLine -split '\s+')[0]

        if ($busId) {
            Write-Host "Extracted BUSID: $busId"
            Write-Host "Attempting to attach device to WSL..."
            usbipd bind --busid $busId
            # 5. Execute the attach command
            usbipd attach --wsl --busid $busId

            Write-Host "-----------------------------------------------------"
            Write-Host "Script finished. Check the output above for status."
            Write-Host "You can now run your C++ application inside WSL."
            Write-Host "-----------------------------------------------------"
        }
        else {
            Write-Error "Could not extract BUSID from the device line."
        }
    }
    else {
        Write-Warning "Could not find a device named '$deviceName'."
        Write-Host "Please ensure the adapter is plugged in and try again."
    }
}
catch {
    Write-Error "An error occurred while running usbipd. Is it installed correctly?"
    Write-Error $_.Exception.Message
}

# Pause if running in a console, so the user can see the output.
if ($Host.Name -eq "ConsoleHost") {
    Read-Host -Prompt "Press Enter to continue"
}
