# 控制代码主机自测

在 Linux/WSL 安装 g++ 后，从工程目录执行 `bash tests/run.sh`。
Windows 可执行：

```powershell
wsl.exe -d Ubuntu -- bash /mnt/d/lxj/26RB/exam1/omni_skeleton/tests/run.sh
```

测试直接编译生产代码中的双板 Robot、Chassis/Gimbal、PID、协议、DT7、电机驱动和 IMU/Mahony；硬件 IMU 采样、时钟、IRQ 与四个 CMSIS-DSP 操作使用主机替身。通过 AddressSanitizer/UndefinedBehaviorSanitizer 检查。编译产物在独立临时目录，退出时清理，不进入固件。

- 协议双向往返、坏包不续期、100ms 边界、tick 回绕；DBUS 通道和拨杆校验。
- 前进/侧移/旋转、云台系变换、同比限速、PID 限幅与复位。
- 达妙 MIT 初始化、速度低四位、使能/失能、急停刚度归零；yaw 跨 ±π、pitch 软件限位。
- 实际双板首次启动、三模式、云台世界 yaw 保持、急停、遥控超时、双向/单向断链、电机超时/故障、IMU 未就绪、恢复目标对齐。
- 实际 IMU 校准、零漂扣除、倾斜收敛、校准中移动/噪声、非法采样和初始化失败。

主机测试不验证 HAL 中断/DMA/CAN 时序、真实接线、机械限位、PID 实车效果或看门狗实际复位。现场步骤见上级 `exam1/考核核对与最终汇总.md`。
