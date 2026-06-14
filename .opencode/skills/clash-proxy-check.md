# Clash 代理连通性检查与修复

## 问题诊断流程

### 1. 检查 Clash 进程状态
```powershell
Get-Process -Name "clash*", "verge*" -ErrorAction SilentlyContinue | Format-Table Name,Id,StartTime
```
- `clash-verge.exe`：GUI 界面
- `verge-mihomo.exe`：Clash 核心，负责代理转发

### 2. 检查 Clash 监听端口
```powershell
$core = Get-Process -Name "verge-mihomo" -ErrorAction SilentlyContinue
if ($core) { Get-NetTCPConnection -State Listen | Where-Object OwningProcess -eq $core.Id }
```
默认 mixed-port: `7897`。

### 3. 检查系统代理设置
```powershell
Get-ItemProperty -Path "HKCU:\Software\Microsoft\Windows\CurrentVersion\Internet Settings" -Name ProxyEnable,ProxyServer
```
- `ProxyEnable=1` 且 `ProxyServer=127.0.0.1:7897` → 系统代理已启用

### 4. 检查 Clash 配置和日志
```powershell
# 运行配置
Get-Content "$env:APPDATA\io.github.clash-verge-rev.clash-verge-rev\clash-verge.yaml"

# 当前激活的订阅
Get-Content "$env:APPDATA\io.github.clash-verge-rev.clash-verge-rev\profiles.yaml"

# Clash Verge 日志
Get-Content "$env:APPDATA\io.github.clash-verge-rev.clash-verge-rev\logs\latest.log"
```

### 5. 测试订阅 URL
```powershell
# 直接测试订阅后端（绕过 redirect/聚合服务）
curl.exe --noproxy "*" -sL --connect-timeout 10 -m 20 "https://jmssub.net/members/getsub.php?service=1299307&id=0baef8b7-c6d9-461c-85c1-d7a349add261"
```
- 若返回 Base64 编码数据 → 订阅仍有效，解码即可获取节点
- 若返回 `"No nodes were found!"` → 订阅已过期

### 6. 测试代理连通性
```powershell
# 注意：代理节点可能很慢（30s+），设置足够长的超时
curl.exe -x http://127.0.0.1:7897 -s --connect-timeout 30 -m 60 https://www.google.com
```

### 7. 测试 SSH 直连
```powershell
ssh -T -o ConnectTimeout=10 git@github.com
```
SSH（端口 22）可能不被 GFW 封锁，是 Git 推送的备选方案。

## 实际诊断结论（2026-06-12，最后更新 2026-06-14）

### 后续修复（2026-06-14）
订阅 IP 持续漂移，上次更新的 IP 隔天失效。核心问题在于 profiles.yaml 中的订阅 URL
`https://suosuo.de/hOAQlDo` 已死（400 Bad Request / No nodes found），Clash Verge 更新时
覆盖 clash-verge.yaml 中的正确 IP。

**修复**：直接修改 profiles.yaml 中 `R8JZpQ1TJuZ7` 的 URL 为直连地址，使 Clash Verge
能自动拉取最新节点。

| 检查项 | 结果 |
|--------|------|
| `clash-verge.exe` 进程 | 运行中 |
| `verge-mihomo.exe` 进程 | 运行中 |
| 监听端口 | 7897 (mixed-port) |
| 系统代理 | 已启用 → `127.0.0.1:7897` |
| 订阅 URL (suosuo.de) | `400 Bad Request` / 超时 |
| 订阅后端 (jmssub.net) | **有效**，返回 6 个节点 |
| 代理节点 IP | clash-verge.yaml 中为**过期 IP** |
| Google 通过代理 | 可访问（延迟 ~1.2-30s，不稳定） |
| SSH 直连 GitHub | **正常** |
| Git http.proxy | 未配置 |
| 国内站点直连 | 正常 |

**根本原因**：
1. Clash 订阅短链接 (suosuo.de) 失效，Clash Verge 无法自动更新
2. clash-verge.yaml 中的节点 IP 是**旧的**（订阅后端返回了新 IP）
3. 代理节点延迟高且不稳定，部分外网请求会超时
4. SSH（端口 22）可直连 GitHub，不受 GFW 影响

## 解决方案

### 方案 A：更新 Clash 节点 IP（已执行）
订阅后端返回了新 IP，已写入 clash-verge.yaml 并重启核心。

**历史 IP 变化**（订阅每次返回不同 IP）：
```
第一批 (6/12):         第二批 (6/12):         第三批 (6/14):
ss1: 67.216.221.120    ss1: 67.216.217.54     ss1: 67.216.217.54
ss2: 67.216.220.108    ss2: 67.216.218.250    ss2: 67.216.218.250
vmess3: 93.179.101.174 vmess3: 93.179.100.183 vmess3: 93.179.100.183
vmess4: 185.212.57.111 vmess4: 185.212.59.92  vmess4: 185.212.59.92
vmess5: 162.248.77.54  vmess5: 104.193.8.22   vmess5: 104.193.8.22
vmess801: 65.49.195.22 vmess801: 199.115.230.84 vmess801: 199.115.230.84
```

### 方案 B：修复 Clash Verge 订阅 URL
在 Clash Verge GUI 中将订阅 URL 从 `https://suosuo.de/hOAQlDo` 替换为直连地址：
```
https://jmssub.net/members/getsub.php?service=1299307&id=0baef8b7-c6d9-461c-85c1-d7a349add261
```

### 方案 C：Git 使用 SSH 推送（推荐，不受代理影响）
```powershell
# SSH 直连 GitHub 通常不受 GFW 影响
git remote set-url origin git@github.com:shifeng2026/revowait-sixdof.git

# 验证
ssh -T -o ConnectTimeout=10 git@github.com

# 推送
git push -u origin main
```

### 方案 D：禁用系统代理（访问国内站点时）
```powershell
Set-ItemProperty -Path "HKCU:\Software\Microsoft\Windows\CurrentVersion\Internet Settings" -Name ProxyEnable -Value 0
```

### 方案 E：延长 curl 超时（代理慢但可用）
```powershell
curl.exe -x http://127.0.0.1:7897 -s --connect-timeout 30 -m 60 https://github.com
```

## 快速验证脚本
```powershell
$proxy = "http://127.0.0.1:7897"
Write-Host "=== Clash 进程 ===" -ForegroundColor Cyan
Get-Process -Name "clash*","verge*" -ErrorAction SilentlyContinue | Format-Table Name,Id
Write-Host "=== 系统代理 ===" -ForegroundColor Cyan
Get-ItemProperty -Path "HKCU:\Software\Microsoft\Windows\CurrentVersion\Internet Settings" -Name ProxyEnable,ProxyServer
Write-Host "=== 连通性测试 ===" -ForegroundColor Cyan
curl.exe --noproxy "*" -s --connect-timeout 5 https://www.baidu.com -o $null -w "Baidu(直连): %{http_code}\n" 2>$null
curl.exe -s --connect-timeout 30 -m 60 -x $proxy https://www.google.com -o $null -w "Google(代理30s): %{http_code}\n" 2>$null
ssh -T -o ConnectTimeout=5 -o StrictHostKeyChecking=no git@github.com 2>&1 | Select-String "successfully"
Write-Host "=== Clash 配置关键信息 ===" -ForegroundColor Cyan
Select-String -Path "$env:APPDATA\io.github.clash-verge-rev.clash-verge-rev\clash-verge.yaml" -Pattern "^(mode|mixed-port):"
```
