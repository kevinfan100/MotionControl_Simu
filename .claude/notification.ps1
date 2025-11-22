# Claude Code Notification Hook for Windows
# Displays Windows notifications when Claude Code needs attention

param(
    [string]$Title = "Claude Code",
    [string]$Message = "Claude needs your attention - check terminal"
)

try {
    # Play notification sound
    $sound = New-Object System.Media.SoundPlayer('C:\Windows\Media\notify.wav')
    $sound.PlaySync()

    # Try Windows Toast Notification first (Windows 10/11)
    try {
        [Windows.UI.Notifications.ToastNotificationManager, Windows.UI.Notifications, ContentType = WindowsRuntime] | Out-Null

        $template = [Windows.UI.Notifications.ToastNotificationManager]::GetTemplateContent([Windows.UI.Notifications.ToastTemplateType]::ToastText02)
        $toastXml = [xml]$template.GetXml()

        $toastXml.GetElementsByTagName("text")[0].AppendChild($toastXml.CreateTextNode($Title)) | Out-Null
        $toastXml.GetElementsByTagName("text")[1].AppendChild($toastXml.CreateTextNode($Message)) | Out-Null

        $toast = [Windows.UI.Notifications.ToastNotification]::new($toastXml)
        [Windows.UI.Notifications.ToastNotificationManager]::CreateToastNotifier("Claude Code").Show($toast)

        exit 0
    }
    catch {
        # Fallback to Balloon Tip notification
        Add-Type -AssemblyName System.Windows.Forms

        $balloon = New-Object System.Windows.Forms.NotifyIcon
        $balloon.Icon = [System.Drawing.SystemIcons]::Information
        $balloon.BalloonTipIcon = [System.Windows.Forms.ToolTipIcon]::Warning
        $balloon.BalloonTipTitle = $Title
        $balloon.BalloonTipText = $Message
        $balloon.Visible = $true

        $balloon.ShowBalloonTip(10000)

        # Keep script alive to show balloon
        Start-Sleep -Seconds 10

        $balloon.Dispose()
    }
}
catch {
    exit 1
}

exit 0
