# IMU 数据处理指南

本指南描述如何将 `MEMS_imu_data.txt` 转换为 IE 软件兼容的 `.imr` 格式文件，用于高精度位置解算。

---

## 准备工作
- ​**操作系统**：Windows 7/10/11  
- ​**必要文件**：
  - `MEMS_imu_data.txt`（原始数据文件）
  - `main.exe`（格式转换程序）

---

## 操作流程

### 1. 复制并重命名文件
也可以手动操作，不使用命令。
```cmd
copy "MEMS_imu_data.txt" "bigimu_out.txt"
```
### 2. 在目录下运行 `main.exe`
```cmd
main.exe
```
