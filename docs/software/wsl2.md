# Running under WSL2

The software runs in WSL2, but WSL2 sits behind a virtual network adapter, which causes three specific problems. Running natively on Windows avoids all of them, so prefer that if you can.

> Not tested by the implementer (no WSL on the build machine). The commands are the standard ones; adjust if your setup differs.

## 1. USB serial is not available

WSL2 does not see USB devices by default.

- **Recommended: use the Wi-Fi transport.**
- Alternative: install `usbipd-win` on Windows, then in an administrator PowerShell `usbipd list`, `usbipd bind --busid <id>`, `usbipd attach --wsl --busid <id>`. The ESP32 then appears in WSL as `/dev/ttyUSB0`. It must be re-attached after every replug.

## 2. Phones cannot reach the web app

WSL2 uses NAT: other devices on the LAN cannot open ports inside WSL. Forward the port from Windows (administrator PowerShell):

```powershell
netsh interface portproxy add v4tov4 listenport=8080 listenaddress=0.0.0.0 connectport=8080 connectaddress=$(wsl hostname -I).Trim().Split(" ")[0]
```

```powershell
New-NetFirewallRule -DisplayName "exxeta-arm web" -Direction Inbound -Protocol TCP -LocalPort 8080 -Action Allow
```

Then open `http://<Windows IP>:8080` on the phone. **The address and QR code printed in the terminal show the WSL address and will not work from a phone**: use the Windows PC's LAN address instead.

The WSL address changes at every Windows restart; remove the rule with `netsh interface portproxy delete v4tov4 listenport=8080 listenaddress=0.0.0.0` and add it again.

## 3. Talking to the arm over Wi-Fi

- **Outgoing UDP works**: the PC sends first and the arm answers to the same address, which NAT allows. So the Wi-Fi transport works without port forwarding.
- **mDNS does not work**: `exxeta-arm.local` will not resolve. Enter the arm's **IP address** in the app (the firmware CLI command `status` prints it, as does your router).

## Mirrored networking (Windows 11 22H2 or newer only)

Putting `networkingMode=mirrored` under `[wsl2]` in `%UserProfile%\.wslconfig` makes WSL share the Windows network, which removes problems 2 and 3. It is not available on Windows 10.
