param(
    [string]$Distro = "Ubuntu",
    [string]$RepoPath = "/mnt/m/CodingProjects/spaceship-simulator",
    [string]$CodexCommand = "codex"
)

$command = "cd '$RepoPath' && $CodexCommand"
wsl.exe -d $Distro bash -lc $command
